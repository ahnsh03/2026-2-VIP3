# data_collection — 수집·rosbag·학습 데이터셋

> **문서 역할:** 패키지 가이드 — 실행 절차
> **담당:** 김동현 — **ROS bridge 연결·수집을 즉시 병행 개발**한다 ([roadmap.md](../../docs/roadmap.md) §3)
> **최종 수정:** 2026-11-10
> **토픽 정본:** [config/ros/topics.yaml](../../config/ros/topics.yaml) · [docs/simulator.md](../../docs/simulator.md)

현재 목표는 MORAI 원본 카메라·상태·시간 정보를 기록하고 재생해 SVM·SS-PSD 개발에 제공하는 것이다. 기존 Capture/semantic-mask 도구는 TwinLite용으로 이식한 경로이며 **SS-PSD 방향 접점 라벨을 자동 생성하는 구현이 아니다.** SS-PSD 학습용 BEV·point/slot 라벨 변환은 [최신 설계](../../docs/ss-psd-sensor-bev-baseline.md)에 따라 추가한다.

**261110 센서셋 적용:** `parking_raw`/`parking_gt` 및 `parking_core`는 새 4어안 구성으로 변경했다. 카메라는 모두 1280×960·10Hz이고 LiDAR/GPS/IMU는 필수로 기다리지 않는다. 실제 수신 결과와 SVM 미리보기는 [센서 가이드](../../docs/sensors.md)를 따른다.

**Capture Mode 이식 코드는 아직 v1 전용이다.** `capture_sync.CAMERAS`는 기존 ID·해상도와 GPS/IMU 파일을 기대하므로, 새 어안 export에 아래 `sync_capture_data.py` 경로를 그대로 사용하면 안 된다. 현재 새 센서셋 수집은 ROS `parking_raw` bag 경로를 사용한다. Capture Mode를 다시 쓸 때는 run의 센서셋별 ID/해상도/모달리티 어댑터를 추가한다.

## 경로가 두 개다 — 먼저 어느 쪽인지 정한다

| 목적 | 경로 | 왜 |
|---|---|---|
| **SS-PSD 수집** | **원본 rosbag → 시간 정렬 → SVM → 직접 주석** | 현재 우선 경로. BEV·방향 접점 라벨 변환은 개발 대상이며 카메라/상태 시각·설정 버전 보존 |
| 기존 semantic-mask 학습 데이터 | **Capture Mode** | Intensity·Semantic·Instance·Depth 파일을 활용. 의미 마스크는 SS-PSD point/slot 라벨과 다름 |
| 연결 확인 · 가벼운 스냅샷 | `sync_collector` | JPEG도 품질을 확인해 수동 주석 원본으로 사용할 수 있으나 동기화·압축 손실을 확인해야 함 |
| 오프라인 반복 재생 | `rosbag` recorder/replay | 시뮬 없이 추론·BEV 를 반복 돌릴 때 |

**Capture Mode 를 쓰면 Semantic 카메라를 따로 달 필요가 없다.** 장착된 RGB 카메라 4대
각각에 대해 한 트리거로 네 장이 나온다.

## 기존 Capture·semantic-mask 경로의 명령

아래는 기존 구현의 사용법이다. SS-PSD 수집에는 원본 rosbag 재생을 먼저 확보하고 별도 BEV·point/slot 라벨 경로를 연결한다.

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

# 3) 정차 중복 프레임 선별  (--output-name 기본값 curation_v1)
python3 scripts/curate_perception_frames.py --run-id <run_id>

# 4) 마스크 굽기 (원본 해상도 → 모델 입력 cache)
python3 scripts/bake_perception_masks.py --run-id <run_id> --stage all \
  --policy config/perception/target_policy_v1_katri_4view.json

# 5) 학습용 split manifest 게시  (--dataset-name 기본값 vip3_katri_parking_v1)
python3 scripts/build_perception_dataset.py \
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

# GT 가 든 bag 은 기본 차단된다. 라벨링·오프라인 분석용으로만 연다.
roslaunch data_collection bag_replay.launch bag_path:=... allow_simulator_gt:=true
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

대회 게이트(`privileged_gt`)는 위 `simulator_gt` 로 **대체**한 것이지 그냥 지운 게 아니다.
다만 이식할 때 라이브러리만 고치고 `bag_replay_node.py` 의 import 를 안 고쳐서 재생
노드가 기동 즉시 `ImportError` 로 죽는 상태가 한동안 있었다 — `tools/check_node_imports.py`
와 `test_bag_support.py::SimulatorGtGateTest` 가 그 회귀를 막는다.

gRPC 를 버려서 잃은 것: 저장 완료 응답(위 `--dry-run` 으로 대체), `morai_sim_time`
(항상 `null`), 날씨·시각 자동 기록(런치 인자로 대체).

## 검증

```bash
cd /root/ws
PYTHONPATH=src/data_collection/src \
  python3 -m unittest discover -s src/data_collection/test -p 'test_*.py'
```

44개 통과 (2026-09-23, ROS·시뮬 불필요). 그중 `test_view_contract.py` 8개가
파이프라인 계약을 지킨다 — 뷰 이름이 단계 간에 어긋나 **후방 데이터가 조용히 버려지던**
버그와, 단계마다 데이터셋/캐시 이름이 달라 **build 결과를 train 이 못 찾던** 버그의 회귀 가드다.

## 확인해야 할 것

- [ ] rosbridge 로 카메라 4대가 몇 Hz 로 오는가 → `collect_hz` 와 `capture_hz` 결정
- [ ] `/velodyne_points` 에 `intensity` 필드가 실제로 있는가
      (`rostopic echo -n1 /velodyne_points | head -40`)
- [ ] MORAI 가 Semantic 을 무손실로 주는가
- [ ] **주차칸 선이 Semantic 에서 무슨 색인가** (위)
- [ ] `/Object_topic` GT 를 최종 시연에 써도 되는지 교수님·모라이 확인
