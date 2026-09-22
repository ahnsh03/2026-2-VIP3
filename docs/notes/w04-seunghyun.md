# 4주차 (09/21~09/27) 안승현 — ASMC 자산 이식

> **문서 역할:** 기록 — 그때 그랬다. 현재 설정으로 읽지 말 것
> **담당:** 안승현
> **최종 수정:** 2026-09-23

## 한 일

2026-ASMC(자율주행 대회 레포)에서 자율주차에 쓸 수 있는 부분을 팀 레포로 옮기고
**rosbridge 단일 전송 · morai_msgs 26.R1 · 4카메라(후방 포함)** 기준으로 맞췄다.

패키지 7개, 단위 테스트 **171개 통과** (ROS·GPU 없이 호스트에서).
자세한 범위는 [porting-from-asmc.md](../porting-from-asmc.md).

## 발견한 것 — 설계에 영향이 큰 순서

### 1. KATRI 맵에 주차면 기하가 없다. 그런데 원본에는 있었다

`global_info.json` 의 `mgeo_file_hash` 에 MORAI 가 내보낸 39개 파일의 sha256 이 남아 있는데,
`parking_space_set.json` 이 **내용 있는 파일로** 기록돼 있고 우리 폴더에는 없다.
빈 리스트(`4f53cda1…`)나 빈 파일(`e3b0c442…`)과 해시가 다르다.

```
parking_space_set.json  0d410f76591ad9bd028c8950fe79fb07453511446caebecf6a9156280783f25b
object_set.json         ee3a8f6aed261977a884aa2fb6e930f6e1bd8470f0ea26bd8d376360a82ae125
```

**모라이·교수님께 요청하면 주차면 GT 를 손으로 찍지 않아도 된다.** 팀에서 1인이 메일 한 통
쓰는 일인데 레버리지가 가장 크다. 받았는지는 1초에 확인된다:

```bash
python3 src/vip3_hd_map/scripts/inspect_katri_mgeo.py --map-dir "$VIP3_DATA/KATRI 맵 데이터 자료" --check
```

로더가 MORAI `ParkingSpace` 스키마를 그대로 받도록 미리 만들어 뒀다. 받으면 코드 수정 없이
레이어가 켜진다.

### 2. `/ctrl_cmd` 가 네트워크 프리셋에 없다

`VIP3_network_v1.json` 21개 항목을 전수 확인했다. PUBSUB_TYPE `771`
(`MoraiCmdController`) 행이 없고, `EgoCtrlConfig` 가 전부 `commType: 0`,
`Topic: "/dafault_topic"` 이다. **이 상태로는 차를 움직일 수 없다.**

MORAI UI 에서 Ego Ctrl Cmd 를 ROS `/ctrl_cmd` 로 켜고 프리셋을 다시 export 해야 한다.
그리고 런타임에 `ctrl_mode = 3 (External)` 을 호출해야 MORAI 가 `/ctrl_cmd` 를 무시하지
않는다. `vip3_gear_node` 가 기동 시 그걸 한다.

### 3. 임시 센서셋의 근거리가 사각이다 — 숫자로

`tools/analyze_sensor_set_coverage.py` 로 계산했다 (시뮬 불필요).

- **차 반경 1.65 m 안쪽 지면은 어느 카메라에도 안 잡힌다**
- 전방 카메라는 **4.50 m 앞부터** 본다 (pitch 2°, 높이 1.2 m)
- 후방은 2.70 m 뒤부터
- **후좌 130° / 후우 230° 대각에 사각 쐐기**가 있다 (각각 12.65 m / 12.85 m).
  좌우 카메라(FOV 130°)와 후방(FOV 90°) 사이가 벌어져서 생긴다.
  **후진 주차에서 차가 실제로 향하는 방향이다**
- 0~2 m 링 가시율 **9.1 %**, 2~4 m 69.7 %

MORAI pitch 부호 규약과 무관한 결론이다 — 부호를 반대로 두면 0~2 m 가 **0 %** 로 더 나빠진다.
근거는 [sensors.md](../sensors.md) §4.

### 4. rosbridge 대역폭이 최대 미지수

ASMC 가 UDP 브리지를 직접 만든 이유가 카메라 처리량이다. 현재 설정
(1280×720 ×2 + 640×480 ×2, 각 20 Hz)이면 base64 포함 약 10 MB/s 를 파이썬 rosbridge 가
직렬화해야 한다. **20 Hz × 4대가 안 나올 가능성이 높다.**

완화 순서는 [simulator.md](../simulator.md) §5 에 적어 뒀다 (ujson 은 이미 이미지에 넣었다).

## 기능이 바뀐 곳 — 그대로 쓰면 안 되는 것

1. **`drivable_bev` 에 binary lane 경로 추가.** ASMC 는 4-class 경로만 있었는데 VIP3 기본
   체크포인트는 binary 라 그대로 두면 노드가 영원히 아무것도 발행하지 않는다
2. **융합 가중치를 등방으로.** ASMC 의 `rear_fade_start_m=−4` 가 후방 카메라 담당 영역
   자체를 깎아서 후방 뷰 평균 quality 가 0.3854 → 0.2583 (×0.670) 이었다
3. **density 정규화를 뷰별로.** front/rear focal 640 px vs left/right 149 px 라 전역
   정규화면 겹치는 셀의 95.8 %를 전방이 가져간다. 주차선을 보는 건 측면 카메라다
4. **BEV 격자** `x[−5,20] res 0.10` → `x[−10,10] res 0.05`.
   15 cm 주차선은 0.10 m 격자에서 안 보인다
5. **`simulator_gt` 게이트 신설.** 시뮬만 아는 정보를 담으려면 프로파일이 명시해야 한다

### 고친 버그 2건

- `smoke_twinlite_data_loss.py` device mismatch
- `evaluate_twinlite.py` 가 v6 4-class 체크포인트를 평가할 수 없었다

### 정정 1건

`lane_type 535` 를 처음에 "주차금지"로 적었는데 근거가 없었다. 27개가 `(82.2, 1190.2)`
주변에 흩어져 있고 centroid 기준 반경 std/mean = 0.536, 방위 12분할 중 2칸이 비어 있어
**링이 아니다.** 주차 규격도 아니다. `UNVERIFIED_535` 로 둔다. 주차면이 아니라는 결론은
어느 쪽이든 같다.

## 안 한 것

- **실기 연동을 한 번도 안 했다.** 위 171개 테스트는 전부 오프라인이다
- 주차칸 검출 · 경로 생성 · 제어 — 기반 논문 확정 후
- 주차 성공 판정(`vip3_eval`) — [roadmap.md](../roadmap.md) §2

## 다음

[roadmap.md](../roadmap.md) §4 의 1블록 9항목. 앞의 6개가 전부 "코드가 아니라 확인"이고,
**전부 설계를 되돌릴 수 있는 정보**라 코드를 더 쓰기 전에 먼저 한다.
