# data_collection — 수집·rosbag·학습 데이터셋

> **문서 역할:** 패키지 가이드 — 실행 절차
> **담당:** 김동현
> **토픽 정본:** [config/vip3_topics.yaml](../../config/vip3_topics.yaml) · [docs/simulator.md](../../docs/simulator.md)

MORAI 에서 데이터를 받아 TwinLiteNet+ 학습에 바로 쓸 수 있는 형태까지 만든다.
ASMC 의 631줄짜리 문서에서 대회 전용·UDP·음영구간 연구 부분을 걷어낸 것이다.

## 경로가 두 개다 — 먼저 어느 쪽인지 정한다

| 목적 | 경로 | 왜 |
|---|---|---|
| **학습 GT** (drivable/차선/주차선 마스크) | **Capture Mode** | MORAI 가 카메라마다 Intensity + **Semantic** + Instance + Depth PNG 를 무손실로 쓴다. 라벨이 공짜로 나온다 |
| 연결 확인 · 가벼운 스냅샷 | `sync_collector` | ROS 토픽을 그대로 저장. 카메라는 JPEG 라 **라벨 원본으로 못 쓴다** |
| 오프라인 반복 재생 | `rosbag` recorder/replay | 시뮬 없이 추론·BEV 를 반복 돌릴 때 |

**Capture Mode 를 쓰면 Semantic 카메라를 따로 달 필요가 없다.** 장착된 RGB 카메라 4대
각각에 대해 한 트리거로 네 장이 나온다.

## 명령 6개 — 이 순서 그대로

```bash
# 0) 연결 확인 — /Ego_topic 만 받아 본다
roslaunch data_collection sync_collector.launch profile:=dry_run

# 1) 학습 GT 수집. MORAI 에서 Capture Mode 를 켜고 휠/키보드로 수동주행한다.
roslaunch data_collection capture_collector.launch \
  capture_hz:=1.0 \
  run_id:=20260930_143000_katri_slotA_sunny_sim11am_wheel

# 2) ★필수★ 무결성 검사 후 복사 (WSL 호스트에서, ROS 불필요)
python3 scripts/sync_capture_data.py --run-id <run_id> --dry-run
python3 scripts/sync_capture_data.py --run-id <run_id> --jobs 4

# 3) 정차 중복 프레임 선별
python3 scripts/curate_perception_frames.py --run-id <run_id> --output-name curation_v1

# 4) 마스크 굽기 (원본 해상도 → 모델 입력 cache)
python3 scripts/bake_perception_masks.py --run-id <run_id> --stage all \
  --policy config/perception/target_policy_v1_katri_4view.json \
  --curation-name curation_v1

# 5) 학습용 split manifest 게시
python3 scripts/build_perception_dataset.py --dataset-name vip3_katri_parking_v1 \
  --train-run <run_a> --train-run <run_b> --val-run <run_c>
```

### ⚠ 2번의 `--dry-run` 은 건너뛰면 안 된다

ASMC 는 gRPC 로 저장 완료 응답을 받아서 실패를 자동 감지했다. VIP3 는 ROS
`/SaveSensorData` 만 쓰는데, **publish 가 리턴하면 무조건 `success: true`** 다. MORAI 쪽
디스크가 가득 차거나 권한이 없어 저장이 실패해도 manifest 에는 성공으로 남는다.

`--dry-run` 이 프레임당 18개 파일의 존재·크기·해상도를 전수 검사한다. **이게 유일한
실패 감지 수단이다.**

성공으로 기록됐는데 파일이 없으면 manifest 를 손으로 고치지 말고 명시적으로 제외한다:

```bash
python3 scripts/sync_capture_data.py --run-id <run_id> --exclude-sequence <run_id>:<seq>
```

제외 이력이 `dataset.json` 에 남는다.

## run_id 규약

```
<YYYYMMDD>_<HHMMSS>_katri_<슬롯>_<날씨>_sim<시각>_<조작>
20260930_143000_katri_slotA_sunny_sim11am_wheel
```

평가 스크립트가 run_id 에서 조건 축을 파싱한다. 주차는 (날씨 × 시각) 보다
**(슬롯 위치 × 진입 방향 × 인접 차량 유무)** 가 의미 있는 축이므로, 축이 정해지면
`scripts/evaluate_twinlite.py` 의 `_condition()` 을 같이 고친다.

## 산출물 구조

```
$VIP3_DATA/
├── capture_runs/<run_id>/          [1] trigger lineage 만
│     meta.json · capture_manifest.jsonl · summary.json
├── datasets/<run_id>/              [2] 실제 파일
│     frames/{intensity,semantic,instance,depth}/{front,left,right,rear}/NNNNNN.png
│     state/{gps,imu}/NNNNNN.txt
│     lineage/  manifest.jsonl  dataset.json  _SUCCESS
│   └── derived/                    [3] 선별 + 마스크
│         curation_v1/
│         vip3_targets_native_v1/     원본 해상도 canonical mask
│         vip3_twinlite_384x640_v1/   모델 입력 cache (재생성 가능)
└── dataset_versions/<name>/        [4] 학습용 split
      train.jsonl · val.jsonl · test.jsonl · dataset.json · _SUCCESS
```

각 단계는 **자기 manifest 와 `_SUCCESS` 마커를 가지고, 상위 단계 산출물을 수정하지
않는다.** 그래서 어느 단계든 지우고 다시 만들 수 있다.

### split 절대 규칙

**`run_id` 단위로 배타 배정한다. 프레임 단위 무작위 split 금지.**
1 Hz 연속 프레임은 거의 같은 그림이고, 주차는 정차·크립 구간이 많아 더 심하다.
프레임 랜덤 split 은 val 을 무의미하게 만든다. `build_perception_dataset.py` 가
run 중복을 검사해서 거부한다.

## 프로파일

### 수집 (`config/profiles/`)

| 이름 | Hz | 채널 |
|---|---:|---|
| `dry_run` | 10 | ego 만 — 첫 연결 확인 |
| `state_only` | 20 | ego + gps + imu — pose 로그 |
| `parking_core` | 5 | 4카메라 + lidar + ego + objects + gps/imu |
| `full` | 10 | 전부, `max_frames: 300` soft cap |

### rosbag (`config/bag_profiles/`)

| 이름 | 종류 | 시뮬 GT |
|---|---|---|
| `parking_raw` | raw | ✗ |
| `camera_raw` | raw | ✗ |
| `lidar_raw` | raw | ✗ |
| `state_raw` | raw | ✗ |
| `parking_gt` | raw | **○** `/Object_topic`, `/CollisionData` |
| `semantic_gt` | raw | **○** semantic 카메라 |
| `bev_result` | result | ✗ 인지 출력만 |

**`simulator_gt` 플래그를 지킨다.** 시뮬레이터만 아는 정보(`/Object_topic`,
`/CollisionData`, `/sem_*`)를 담으려면 프로파일이 명시해야 하고, 안 하면 로드가 실패한다.
섞이면 인지 성능 측정이 "시뮬이 정답을 알려준 것"이 되어 의미가 없어진다.
라벨링·평가에는 얼마든지 쓴다 — 다만 명시하고 쓴다.

```bash
roslaunch data_collection bag_recorder.launch profile:=parking_raw weather:=sunny sim_hour:=11
roslaunch data_collection bag_replay.launch bag_path:=/data/bags/<run>/<run>.bag
rosservice call /vip3_bag_replay/start     # 추론 노드를 다 띄운 뒤에 시작한다
```

`bag_replay` 는 **일시정지 상태로 준비**한다. 그래야 추론 노드가 뜨기 전 프레임을
놓치지 않는다.

## Semantic 라벨 — 알아야 할 것

마스크 생성기는 Semantic PNG 의 **클래스 색을 정확히 일치 비교**한다.

- **무손실이어야 한다.** JPEG 가 한 번이라도 끼면 전부 깨진다.
- **안티에일리어싱을 끈다.** 경계에 중간색이 생기면 팔레트 밖 색이 되어 프레임이 거부된다.
- MORAI 26.R1 팔레트 26색 중 차선 관련: `white_lane(255,255,255)`,
  `yellow_lane(255,255,0)`, `blue_lane(0,178,255)`, `stopline(255,0,0)`,
  `crosswalk(76,255,76)`, `road_edge(178,178,178)`, `asphalt(127,127,127)`.

**⚠ 팔레트에 주차칸 선 전용 클래스가 없다.** 주차 슬롯 라인이 `white_lane` 으로 칠해지는지
`yellow_lane` 인지, 아니면 아예 안 칠해지는지 **아직 아무도 모른다.**

```bash
# KATRI 주차장에서 캡처 한 번 → 실제로 무슨 색인지 확인
python3 scripts/audit_semantic_capture.py --run-id <run_id>
```

**이게 팀 전체에서 가장 싸고 가장 결정적인 실험이다.** 답에 따라 target policy, 마스크
생성기, 학습 클래스 정의가 전부 갈린다. 첫 수집에서 반드시 돌리고 결과를
[docs/katri-map.md](../../docs/katri-map.md) 에 적는다.

## cam4 는 후방이다

ASMC 의 4번 카메라는 하향 카메라였고 **학습에서 제외**돼 있었다. VIP3 의 4번은 후방
카메라이고 **후진 주차의 핵심 뷰**라 다른 뷰와 동등하게 학습에 들어간다.

- 광학이 전방과 같다 (1280×720, FOV 90) → letterbox 기하도 전방과 같다
  (`720×1280 → 360×640`, pad top/bottom 12 px)
- 프레임당 표본이 3 → **4** 가 된다 (디스크·학습시간 +33 %)
- `stopline` / `road_marking` 의 front-only 가정을 걷어냈다. 후진 중에는 뒤쪽 선이
  판단 근거다

나중에 후방 fisheye 를 추가하면 뷰 이름만 목록에 더하면 된다 — 파이프라인이 view-list
기반이라 그 외 코드 변경이 없다.

## 가져오지 않은 것

gRPC 클라이언트 · UDP/oracle/shadow/competition bag 프로파일 · 대회 규정용
privileged/deployment 게이트 · 신호등 채널 · streaming GT 실험 · `monitor_transport_pilot.py`.

gRPC 를 버려서 잃은 것: 저장 완료 응답(위 `--dry-run` 으로 대체), `morai_sim_time`
(항상 `null`), 날씨·시각 자동 기록(런치 인자로 대체).

## 검증

```bash
cd /root/ws
PYTHONPATH=src/data_collection/src \
  python3 -m unittest discover -s src/data_collection/test -p 'test_*.py'
```

33개 통과 (2026-09-23, ROS·시뮬 불필요).

## 확인해야 할 것

- [ ] rosbridge 로 카메라 4대가 몇 Hz 로 오는가 → `collect_hz` 와 `capture_hz` 결정
- [ ] `/velodyne_points` 에 `intensity` 필드가 실제로 있는가
      (`rostopic echo -n1 /velodyne_points | head -40`)
- [ ] MORAI 가 Semantic 을 무손실로 주는가
- [ ] **주차칸 선이 Semantic 에서 무슨 색인가** (위)
- [ ] `/Object_topic` GT 를 최종 시연에 써도 되는지 교수님·모라이 확인
