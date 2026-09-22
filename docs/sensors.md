# 센서셋 — VIP3_sensor_set_v1 (임시)

> **문서 역할:** 정본 — 지금 쓰는 센서 구성과 그 한계
> **담당:** 강도균 (센서셋) · 안승현 (기하·BEV)
> **기계 정본:** `config/VIP3_sensor_set_v1_ros.json`, `config/vip3_topics.yaml`
> **최종 수정:** 2026-09-23

## 1. 상태

**이 센서셋은 임시다.** 기반 논문과 인지 모델이 정해지면 바꾼다. 지금은 "무엇이든 하나 정해서
코드가 돌게 만드는" 용도다. 아래 §4의 사각 분석이 다음 센서셋 설계의 근거다.

원본은 로컬 루트 `../data/VIP3_sensor_set_v1.json`(UDP 설정)이고, 저장소의
`config/VIP3_sensor_set_v1_ros.json`은 이를 **ROS 전송으로 바꾸고 토픽·frame_id를 팀 규약으로
채운 것**이다. 변환은 재현 가능하다:

```bash
python3 tools/make_ros_sensor_set.py \
  --input  "../data/VIP3_sensor_set_v1.json" \
  --output "config/VIP3_sensor_set_v1_ros.json"
```

MORAI Sensor 설정에서 이 파일을 Import 한다.

> `commType: 3` 이 ROS 를 뜻한다고 보고 변환했다. 근거는 `VIP3_network_v1.json`의 ROS 항목이
> 전부 `netType=3, commType=3`이고, MORAI gRPC proto 의 `NetworkCommType` 이
> `UNSPECIFIED=0, UDP=1, TCP=2, ROS=3, ...` 이기 때문이다. **센서 열거가 같은지는 시뮬레이터에서
> 한 번만 확인하면 된다** — UI 에서 카메라 하나를 손으로 ROS 로 바꿔 Export 한 뒤
> `cameraList[0].cc.commType` 값을 보면 끝난다. 다르면 `--comm-type <값>` 으로 다시 돌린다.

## 2. 구성

base_link = **뒷바퀴축 중심**, x 전방 · y 좌 · z 상 (REP-103).

| ID | 센서 | 위치 (x,y,z) m | 회전 (roll,pitch,yaw) deg | 해상도 / FOV | 토픽 | frame | 주기 |
|---:|---|---|---|---|---|---|---:|
| 1 | front camera | 1.900, 0.000, 1.200 | 0, 2, 0 | 1280×720 / 90° | `/cam_front/image_jpeg/compressed` | `cam_front` | 20 Hz |
| 2 | left camera | 1.150, 0.650, 1.200 | 0, 10, 70 | 640×480 / 130° | `/cam_left/image_jpeg/compressed` | `cam_left` | 20 Hz |
| 3 | right camera | 1.150, −0.650, 1.200 | 0, 10, 290 | 640×480 / 130° | `/cam_right/image_jpeg/compressed` | `cam_right` | 20 Hz |
| 4 | **rear camera** | −0.100, 0.000, 1.200 | 0, 2, 180 | 1280×720 / 90° | `/cam_rear/image_jpeg/compressed` | `cam_rear` | 20 Hz |
| 5 | Lidar3D | 0.800, 0.000, 1.450 | 0, 0, 0 | — | `/velodyne_points` | `velodyne` | 10 Hz |
| 6 | GPS | 0.350, 0.000, 1.250 | 0, 0, 0 | — | `/gps` | `gps` | 5 Hz |
| 7 | IMU | 0.000, 0.000, 0.000 | 0, 0, 0 | — | `/imu` | `imu` | 50 Hz |

카메라 1~3은 ASMC `ASMC_sensor_set_v2` 와 **완전히 같다.** 4번만 다르다 — ASMC 는
(0, 0, 1.400) pitch 328° 의 하향 카메라였고, VIP3 는 후방 카메라다.

## 3. 내부 파라미터 — JSON 값을 믿지 말 것

센서셋 JSON 의 `focalLengthpixel: 320.0` 은 **네 카메라 모두 틀렸다.** 해상도와 FOV 로 다시
계산한다:

```
fx = fy = (width / 2) / tan(horizontal_fov / 2)
cx, cy = width / 2, height / 2        # JSON principalPoint 와 우연히 같다
```

| 카메라 | fx (px) | JSON 값 | 배율 오차 |
|---|---:|---:|---:|
| front / rear | **640.000** | 320.0 | 2.00× |
| left / right | **149.219** | 320.0 | 0.47× |

`lensDistortion`은 네 대 모두 `[0,0,0]` 이라 왜곡 보정이 필요 없다. 핀홀 모델이 **근사가 아니라
정확하다.** BEV 호모그래피(`src/perception/drivable_bev/`)가 이 규칙을 그대로 쓴다.

## 4. 이 센서셋의 한계 — 숫자로

`tools/analyze_sensor_set_coverage.py` 로 계산한다 (시뮬레이터 불필요):

```bash
python3 tools/analyze_sensor_set_coverage.py \
  --sensor-set "../data/VIP3_sensor_set_v1.json" \
  --image /tmp/coverage.png
```

주차 격자 x[−10,10] y[−8,8], 지면 z = −0.35 m 기준:

| 반경 | 어느 카메라든 보이는 비율 | front | left | right | rear |
|---|---:|---:|---:|---:|---:|
| 0–2 m | **9.1 %** | 0 % | 4.6 % | 4.5 % | **0 %** |
| 2–4 m | 69.7 % | 0 % | 27.7 % | 27.7 % | 14.3 % |
| 4–6 m | 91.9 % | 11.9 % | 31.7 % | 31.7 % | 24.8 % |
| 6–8 m | 94.9 % | 19.0 % | 32.9 % | 32.9 % | 24.8 % |

방위별 최근접 가시 지면 거리:

| 방향 | 최근접 | 보는 카메라 |
|---|---:|---|
| 정면 (0°) | 4.50 m | front |
| 좌측 (90°) | 1.75 m | left |
| **후좌 대각 (130°)** | **12.65 m** | ← **사각** |
| 정후방 (180°) | 2.70 m | rear |
| **후우 대각 (230°)** | **12.85 m** | ← **사각** |
| 우측 (270°) | 1.75 m | right |

**결론 세 가지.**

1. **차 반경 1.65 m 안쪽 지면은 어느 카메라에도 안 잡힌다.** 주차칸에 들어가는 마지막 구간이 전부
   여기다.
2. **전방 카메라는 4.5 m 앞부터 본다.** pitch 2° 에 높이 1.2 m 라 그 이상 가까운 노면이 화각 밖이다.
3. **후좌·후우 대각 130°/230° 에 사각 쐐기가 있다.** 좌/우 카메라(화각 130°)와 후방
   카메라(화각 90°) 사이가 벌어져서 생긴다. 후진 주차에서 차가 실제로 향하는 방향이다.

이 결론은 MORAI 의 pitch 부호 규약과 무관하다 — 부호를 반대로 두면 0–2 m 가시율이 9.1 % 에서
**0 %** 로 더 나빠질 뿐이다 (`--pitch-sign -1` 로 확인 가능).

![VIP3_sensor_set_v1 지면 가시영역](images/sensor-set-v1-coverage.png)

위 그림은 x[−10,10] y[−8,8] 격자다. 위가 전방. 초록 = front, 주황 = left, 파랑 = right,
자홍 = rear, 흰색 = 2대 이상 중복, **검정 = 어느 카메라도 못 보는 곳**, 가운데 빨간 점 =
base_link. 좌우 아래로 뻗은 검은 쐐기 두 개가 후좌·후우 대각 사각이고, 가운데 검은 덩어리가
근접 사각이다.

### 다음 센서셋(v2)에서 할 것

우선순위 순:

1. **후방 fisheye 또는 하향 카메라 추가.** 주차 슬롯 검출 논문 대부분이 AVM(어라운드뷰) 4대 합성
   영상을 입력으로 가정한다. 기반 논문이 정해지는 순간 이 항목의 스펙이 정해진다.
2. **rear pitch 를 2° → 15~20° 로.** 후방 최근접이 2.70 m → 약 1.3 m 로 개선된다. 한 줄 수정으로
   얻는 가장 싼 이득이다.
3. 좌우 카메라를 뒤로 옮기거나 yaw 를 키워 130°/230° 쐐기를 메운다.
4. Semantic 카메라 4대 추가 (§5).

> 현재 BEV 호모그래피는 `0 < FOV < 180` 을 요구한다 (`drivable_bev/calibration.py`). **진짜
> fisheye(≥180°)는 핀홀 모델로 못 다룬다** — 어안 모델을 따로 넣어야 한다. 센서셋을 바꾸기 전에
> 이 비용을 같이 계산할 것.

## 5. Semantic 카메라 (강도균 파트)

MORAI Capture Mode 를 쓰면 **장착된 RGB 카메라마다** Intensity / Semantic / Instance / Depth PNG
가 한 번에 나온다. 즉 **학습 GT 만 필요하면 Semantic 카메라를 따로 달 필요가 없다.**

실시간 ROS Semantic 스트림이 필요할 때만 추가한다. 그때 지킬 것:

- `m_SensorUniqueID` 는 8, 9, 10, 11 (기존 1~7 과 충돌 금지)
- **`cc.cameraType: 1`** 이 Semantic (RGB 는 0)
- `pos` / `rot` / 해상도 / FOV 를 짝이 되는 RGB 카메라와 **완전히 동일하게** 복사
- 토픽은 `/sem_front/...`, `/sem_left/...`, `/sem_right/...`, `/sem_rear/...`
- **무손실이어야 한다.** 마스크 생성기가 클래스 색을 정확히 일치 비교하므로 JPEG 압축이 한 번이라도
  끼면 전부 깨진다. 안티에일리어싱도 꺼야 한다.
- MORAI 팔레트에 **주차칸 선 전용 클래스가 없다.** 주차 슬롯 라인은 `white_lane (255,255,255)`
  또는 `yellow_lane (255,255,0)` 로 칠해질 가능성이 높다. **이게 실제로 무슨 색인지 확인하는 것이
  팀 전체의 가장 싼 결정적 실험이다** — KATRI 주차장에서 캡처 한 장이면 된다.

## 6. 확인해야 할 것 (첫 연동 때)

- [ ] 센서 `commType` ROS 열거값이 3 인가
- [ ] 카메라 토픽에 `/compressed` 가 자동으로 붙는가, 아니면 전체 문자열을 써야 하는가
- [ ] `rostopic hz` 로 카메라 4대 실제 수신율. rosbridge 는 base64 JSON 이라 20 Hz × 4대가
      안 나올 가능성이 높다 → [simulator.md](simulator.md) §5 의 완화 순서
- [ ] `header.stamp` 가 0 이 아닌가, sim time 인가 wall time 인가
- [ ] MORAI pitch 양수가 렌즈 아래인가 위인가 (지면 격자 오버레이로 확정)
- [ ] base_link 원점이 정말 뒷바퀴축 중심인가 (`/Service_MoraiMapSpec` 또는 LiDAR 지면 비교)
- [ ] 지면 z 가 −0.35 m 가 맞는가 (KATRI 평지에서 LiDAR 평면 피팅)
