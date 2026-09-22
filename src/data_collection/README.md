# data_collection — Capture·rosbag·동기화·인지 데이터셋

> **역할:** Active Package Guide — 현재 수집·bag·dataset 재현 절차
>
> 인지 전체 경계는 [Perception 개발·통합 가이드](../../docs/packages/asmc_perception/README.md),
> label·split 정본은 [dataset contract](../../docs/contracts/perception/dataset-labeling-split.md)를
> 따른다. 이 문서의 상세 명령과 모드 차이는 Phase 4 분리 전까지 그대로 보존한다.

> **역할:** Active Module — Capture GT, raw/result rosbag, 동기화, mask/cache/dataset lineage
> **담당:** 안승현
> **데이터 정본:** [인지 데이터 라벨링·분할 계약](../../docs/contracts/perception/dataset-labeling-split.md)
> **전체 로드맵:** [카메라·MGeo·LiDAR 융합 로드맵](../../docs/packages/asmc_perception/roadmap.md)
> **rosbag 정본:** [rosbag 재현 평가 가이드](../../docs/operations/rosbag-evaluation.md)
> **문서 허브:** [docs/README.md](../../docs/README.md)

## 수집 경로 선택

| 목적 | 사용할 경로 | 지위 |
|---|---|---|
| 카메라 drivable/road-marking 학습 GT | Capture Mode → 파일 동기화 → native mask → model cache → version manifest | 현재 정본 |
| 일반 ROS 토픽 snapshot | `sync_collector.launch` profile | 보조·레거시 경로 |
| 팀 공용 재현 bag | allowlist profile + raw/result recorder/replay | `camera_raw`·`full_raw`·paused replay·result smoke 완료 |
| 음영구간 판제 handoff 결과 | `shadow_boundary_result` | typed boundary·camera/lane diagnostics 최소 기록 |
| shared-family read-only 판제 adapter A/B | `shadow_boundary_family_adapter_result` | family 입력·adapter 진단 필수, strip marker 선택 기록; deployment 금지 |
| 음영구간 v3 판제 인수 replay | `shadow_camera_perception_handoff_result` | family·drivable score·coverage와 3단 diagnostics 필수; 제어 승격과 무관 |
| 고주기 RGB sequence | v2 + Simulation Time Sensor Record | 장착된 일반 센서만 저장. 시계열 학습·재생 후보 |
| 고주기 supervised sequence | 최소 RGB+Semantic GT 센서셋 + Simulation Time Sensor Record | 별도 pilot 후 채택. v2 Sensor Record만으로 Semantic GT가 생기지 않음 |
| 고주기 Streaming GT | GT UDP bridge source-time + privileged bag | 종료된 실험. profile은 재현용으로만 보존 |

Capture Mode와 gRPC는 26.R1.h3 개발·학습용이다. 본선 배포 추론은 허용 UDP 센서만
사용하며 Semantic/Instance GT, Capture gRPC와 oracle 토픽을 사용하지 않는다.

이 README에는 실행 절차를 두고, palette/ignore/split 같은 정책과 현재 run 구성은
데이터 계약 문서에서 관리한다.

## MORAI 모드와 기록 방식 구분

다음 설정은 하나의 mode 선택지가 아니라 서로 독립된 축이다.

| 축 | 선택지 | 의미 |
|---|---|---|
| 시간 진행 | Real Time / Simulation Time | wall time 기반 진행 또는 simulation time 기준 진행 |
| 외부 tick | Sync Off / Sync On | simulator 자체 진행 또는 외부 ROS tick마다 진행 |
| 센서 주기 동기화 | Off / On | 센서별 주기 유지 또는 활성 센서를 공통 frame rate로 맞춤 |
| Capture | Off / On | 현재 frame의 센서 파일 저장 기능 |
| 통신 | UDP / ROS / gRPC | runtime sensor/control 및 개발 API transport |
| 기록 | Capture files / rosbag | 학습 GT 파일 또는 ROS message stream 기록 |

Capture Mode는 Real/Simulation Time과 별개로 켜는 저장 기능이다. 외부 Sync On은 ROS
`SyncModeCmd` tick을 기다리며 wheel/keyboard 수동주행에 적합하지 않다. Time mode를
전환하면 simulation time이 초기화될 수 있으므로 전환 전후를 같은 run으로 기록하지 않는다.
상세 동작은 MORAI [Time Manager](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/morai-sim-time-management-function),
[Synchronous Mode message](https://morai-sim--drive-user-manual--en-22-r2.scrollhelp.site/msdume2/synchronous-mode-ros-message-specifications),
[Capture Sensor Data](https://morai-sim--drive-user-manual--en-22-r2.scrollhelp.site/msdume2/capture-sensor-data)를
기준으로 한다.

목적별 기본 선택은 다음과 같다.

| 목적 | 기본 방식 |
|---|---|
| 현재 카메라 단일-frame 학습 | Real Time + Capture 1 Hz |
| 카메라 단일-frame 보강 | Capture 1 Hz. 2/5/10 Hz 요청 pilot은 실효 1.4~1.7 Hz로 포화 |
| 수동 휠 다양화 | Real Time + Capture |
| 고주기 RGB 시계열 파일 | v2 + Simulation Time + Sensor Record. 필요 시 Sensor Synchronization 10 Hz pilot |
| 고주기 RGB–Semantic 학습 pair | 최소 RGB+Semantic 센서셋 + Simulation Time + Sensor Record. pairing 검증 필수 |
| 센서 생성 skew를 배제한 알고리즘 검증 | 필요 시 Simulation Time Sensor Record + offline replay pilot |
| 실제 지연·drop·jitter 평가 | Real Time UDP + raw rosbag |
| 모델 A/B 비교 | 같은 raw bag 재생, result는 모델별 분리 |
| 기존 LiDAR clustering/OBB 튜닝 | PointCloud·GPS·IMU·Ego·개발용 Object GT raw bag |
| 학습형 LiDAR 객체 검출 | 실제 Intensity point cloud + Instance/3D box GT Capture |

Capture는 요청마다 시뮬레이션이 약 0.5초 멈춰 wheel/MPC temporal 평가에 적합하지 않다.
실제 비동기 성능과 모델 A/B에는 Real Time UDP raw rosbag을 사용한다. 과거 GT Connect
stream은 Capture를 대체하지 못했고 Streaming/ROS GT transport 실험은 종료했다.

### Simulation Time의 Sensor Record

Time Manager 창의 `Synchronization Mode`는 **장착 센서들의 생성 시각·주기를 공통화하는
센서 동기화 옵션**이다. 외부 알고리즘이 매 tick을 보내는 `External Sync`와는 다른 설정이다.
External Sync가 꺼진 Simulation Time에서도 Sensor Record와 센서 동기화는 사용할 수 있다.
동기화를 10Hz로 켜면 활성 센서는 UI에 입력한 공통 10Hz를 기준으로 생성·저장되고, 끄면
카메라 20Hz, LiDAR 10Hz, GNSS 5Hz처럼 센서별 설정 주기를 따른다. 5Hz GNSS를 공통 10Hz로
올렸을 때 새 측정과 hold/repeat 중 무엇으로 저장되는지는 버전별 pilot에서 확인한다.

Sensor Record는 rosbag이 아니라 시뮬레이터가 센서별 원시 파일을 Windows에 직접 저장하는
기능이다. 기본 경로는 다음과 같다.

```text
{MORAI 설치 경로}\MoraiLauncher_Win_Data\SaveFile\SensorData\
├── CAMERA_<sensor_id>\
├── LIDAR_<sensor_id>\
├── GPS_<sensor_id>\
└── IMU_<sensor_id>\
```

현재 개발 PC의 실행 파일은 `C:\MoraiLauncher_Win\MoraiLauncher_Win.exe`이고 실제 저장
루트는 아래 경로다. 2026-09-14 v2 Sensor Record 실측에서는 `CAMERA_1~4`, `LIDAR_5`,
`GPS_6`, `IMU_7`만 갱신됐고, 과거 GT 센서셋이 만들었던 `CAMERA_8~10`, `LIDAR_11`은
갱신되지 않았다. 즉 Sensor Record는 현재 장착·활성화된 센서의 출력을 저장할 뿐,
Capture처럼 장착하지 않은 Semantic/Instance 정답을 자동 생성하지 않는다.

```text
C:\MoraiLauncher_Win\MoraiLauncher_Win_Data\SaveFile\SensorData
```

파일명 시간은 wall time이 아니라 simulation time이다. Time mode 또는 Sync On/Off 전환 시
simulation time이 0으로 초기화될 수 있으므로 record 종료 후 파일을 run별 디렉터리로 즉시
복사·정본화하고, 서로 다른 session의 같은 simulation timestamp를 한 run으로 합치지 않는다.
External Sync service의 `sensor_capture` 결과는 별도
`SensorData\SynchronousMode\<SyncMode 시작 시각>`에 저장될 수 있으므로 UI Sensor Record와
구분한다.

용도는 다음처럼 나눈다.

| 분석 목적 | Sensor Record + sensor sync | Real Time UDP rosbag |
|---|---|---|
| 장착된 3카메라·LiDAR 간 생성 시각 정합 | 권장 | 실제 skew를 포함한 보조 기준 |
| RGB/Semantic 학습 pair | RGB와 Semantic 센서를 함께 장착한 최소 GT 센서셋에서만 권장 | source stamp gate를 통과한 pair만 사용 |
| 센서 skew를 배제한 알고리즘 상한 평가 | source-time bag/replay를 함께 쓸 때 권장 | 실제 운용 기준 |
| 실제 처리 지연·drop·jitter·bridge 부하 | 부적합 | 정본 |
| 판제까지 포함한 실제 운용 거동 | 부적합 | 정본 |

즉 Simulation Time + 10Hz Sensor Record는 **장착 센서의 생성 시각을 맞춘 통제 입력**을
만드는 데 유용하지만, simulator-side 파일 기록이라 UDP/bridge/ROS queue/추론 지연은
재현하지 않는다. 기존 live 추론은 이 파일을 직접 읽지 않는다. 알고리즘 입력 skew를
실제로 제거할 필요가 생기면 Sensor Record 파일용 offline replay pilot을 별도로 만든다.
과거 `timestamp_mode:=source` GT bag 경로는 종료됐고, `receive` stamp를 사용하는 현행
Real Time UDP bag에는 네트워크 도착 시각 차이가 남는다.

따라서 결과 명칭도 구분한다. 향후 Simulation Time sync 결과는
**controlled/ideal-input 알고리즘 검증**, Real Time UDP bag 결과는 **deployment/운용
검증**이다. 전자는 새 pilot을 통과해도 후자를 대체하지 않는다. 고주기 supervised
segmentation이 실제로 필요해질 때만 `RGB front/left/right + Semantic front/left/right +
필요한 상태 센서`의 최소 sync-training 센서셋을 별도 version으로 검토한다. 정식 채택 전
30초 pilot에서 센서별 파일 수, timestamp 단조성, 10Hz 공통 frame key, RGB–Semantic
pairing과 GNSS 반복 여부를 먼저 검사한다.

근거는 MORAI [Time Manager](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/morai-sim-time-management-function),
[센서 검출 데이터와 저장 경로](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/26.R1/-33),
[Synchronous Mode 메시지](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R1.0/synchronous-mode-2)를 따른다.

## 재현 가능한 raw/result rosbag

SIM 버전과 transport가 metadata에서 구분되도록 Real Time UDP 운용 profile을 실행한다.

```bash
./scripts/udp_bridge.sh --simulator 26r1
./scripts/udp_bridge.sh --simulator competition
```

기본 녹화는 scenario와 controller만 입력한다. KST 시작 시각, 초기 날씨·시뮬레이션 시간,
map, Git 상태, sensor set/transport, topic/type/rate, bridge diagnostics와 bag hash는 자동으로
기록된다.

```bash
# 대회 입력 재현용. bridge는 기본 receive mode
roslaunch data_collection bag_recorder.launch \
  profile:=full_raw scenario:=comp_sample controller:=wheel
```

`lidar_dev_gt_udp`와 `dev_oracle_udp`는 과거 실험을 재현할 때만 사용하며 신규 운용·학습의
권장 명령에는 포함하지 않는다. direct ROS GT 구현은 현재 `main`에 없고 백업 브랜치에만 있다.

저장 위치는 `/data/rosbags/raw/<auto_run_id>/`이며 raw는 덮어쓰지 않는다. 잘못된 closed
run은 `reject_bag_run.py --run-id ... --reason ...`으로 `/data/rosbags/rejected/`에 옮긴다.

```bash
# live bridge를 끈 뒤 1) paused replay controller로 sim clock 준비
roslaunch data_collection bag_replay.launch \
  bag_path:=/data/rosbags/raw/<run_id> required_profile:=camera_raw

# 2) 추론 노드와 result recorder를 띄운 뒤 재생 시작
rosservice call /asmc_bag_replay/start

# 외부 audits/에 rate, timestamp, pairing과 Instance schema 기록
rosrun data_collection audit_rosbag_run.py \
  /data/rosbags/raw/<run_id> \
  --json-out /data/rosbags/audits/<run_id>/audit.json
```

result replay는 실행 순서가 중요하다. 추론과 result recorder를 replay controller보다 먼저
띄우면 wall clock을 사용해 지연 통계가 깨질 수 있다. result recorder는 이를 막기 위해
`/use_sim_time=true`가 아닌 상태에서는 시작하지 않는다. 상세한 4터미널 순서와 실제 smoke
기준선은 [전용 가이드](../../docs/operations/rosbag-evaluation.md#4-replay와-result-lineage)를
따른다.

profile 목록, result bag/checkpoint lineage와 종료된 GT transport 실험의 한계는
[전용 가이드](../../docs/operations/rosbag-evaluation.md)를 따른다.

## ROS topic snapshot collector (보조 경로)

Capture Mode를 쓰기 전 만든 일반 토픽 수집기다. 최신 메시지를 래치한 뒤 timer에서
snapshot할 뿐 **MORAI 외부 Sync 제어기가 아니다**. 상태·LiDAR 등 profile 기반 snapshot이
필요할 때 사용할 수 있지만 현재 카메라 학습 GT의 정본은 아니다.

### 방침

**시뮬에는 규정 센서를 다 달아도 되고**, 디스크에는 **profile로 켠 채널만** 쓴다.

| Profile | 용도 |
|---------|------|
| `dry_run` | `/Ego_topic`만 — 첫 연동 |
| **`perception_core`** | **snapshot 기본** — cam×3 + lidar + ego + objects + TL(optional) |
| `lidar_gt` | Det3D |
| `state_only` | GPS/IMU + ego |
| `full` | 전채널 · **짧은 런만** (`max_frames` 기본 300) |

“나중에 뭐 쓸지 몰라서 항상 full” → I/O·디스크로 수집이 죽음.
기존 snapshot `config/profiles/lidar_gt.yaml`은 ID 11 Streaming rosbag profile이 아니다.
과거 GT bag은 `config/bag_profiles/lidar_dev_gt_udp.yaml`로 재현할 수 있으나 현행 권장
수집 경로는 아니다.

### 환경변수

```bash
export AIM_PROJECT=~/projects/2026-ai-sw-mobility-competition
export ASMC=$AIM_PROJECT/2026-ASMC
export ASMC_DATA=${ASMC_DATA:-$AIM_PROJECT/data}
```

선택: `ASMC_COLLECT_ENABLE=cam_extra,gps` / `ASMC_COLLECT_DISABLE=tl_status`

### 실행

```bash
cd "$ASMC"
source devel/setup.bash   # catkin build data_collection 후

# 1) ego dry-run
roslaunch data_collection sync_collector.launch profile:=dry_run

# 2) 기본 인지 수집
roslaunch data_collection sync_collector.launch profile:=perception_core

# 3) full 쇼트런
roslaunch data_collection sync_collector.launch profile:=full max_frames:=100
```

저장: `$ASMC_DATA/datasets/<run_id>/` (`meta.yaml`, `manifest.jsonl`, `sensors/`, `state/`, `gt/`).
`$ASMC_DATA`는 팀 Git 레포 안이 아니라 레포와 나란한 워크스페이스 `data/`를 권장한다.
Docker에서는 ROS 컨테이너에 `/data`로 쓰기 가능, 학습 컨테이너에는 `/data`로 읽기 전용 마운트된다.

## Capture Mode 파이프라인 (현재 카메라 GT 정본)

`capture_mode_collector_node.py`는 차량 제어 명령을 발행하지 않는다. 따라서
`mpc_controller_node`가 `/ctrl_cmd`를 발행하는 자동주행과 MORAI Game Wheel/Keyboard 수동주행에서
동일하게 사용할 수 있다.

### 준비

1. MORAI 26.R1.H3에서 K-City 2025와 일반 수집은 `config/ASMC_sensor_set_v2.json`,
   Semantic/Instance GT 수집은 `config/ASMC_sensor_set_GT.json`을 로드한다.
2. Sensor Capture Mode를 활성화한다.
3. 아래 둘 중 하나를 연결한다.
   - 권장: MORAI gRPC server `7789` — 성공/실패 응답과 sim time을 받을 수 있다.
   - 폴백: MORAI ROS Network에 `/SaveSensorData` subscriber를 추가한다.
4. ROS 컨테이너에서 UDP bridge를 실행한다. 휠 주행에서도 `/Ego_topic` 기록을 위해
   bridge는 유지하되 MPC 등 `/ctrl_cmd` 발행 노드는 끈다.

### 센서셋 정본과 LiDAR GT

| 파일 | 용도 |
|---|---|
| `config/ASMC_sensor_set_v1.json` | 과거 run 재현용 |
| `config/ASMC_sensor_set_v2.json` | 현재 26.R1.H3 개발 기본값 |
| `config/ASMC_sensor_set_GT.json` | v2 + Semantic Camera×3 + Instance LiDAR, 개발·학습 전용 |

v1→v2의 유효 변경은 rear camera pitch `340°→328°`다. GT 센서셋의 LiDAR는 다음처럼
동일 위치·주기에서 입력과 정답을 분리한다.

| 용도 | Sensor ID | 위치 | 주기 | 타입 | UDP Dest |
|---|---:|---|---:|---|---:|
| runtime/학습 입력 | 5 | `(0.8, 0, 1.45)m` | 10 Hz | Intensity (`0`) | 9299 |
| 학습 정답 | 11 | `(0.8, 0, 1.45)m` | 10 Hz | Instance (`2`) | 9311 |

ID 5의 실제 intensity만 detector 입력으로 사용하고 ID 11의 instance point와
`_instance.txt`/`_instance_8Points.txt`는 class, 3D box, 상대속도와 unique ID 정답으로만
사용한다. GT value를 입력 feature로 사용하지 않는다. 공용 bridge는 9311을
항상 독립 수신해, GT가 송신할 때 `/velodyne_points_instance`의 `instance_id uint32`로
발행한다. v2 센서셋에서는 해당 포트가 데이터 없이 대기한다. 과거 Streaming GT에는
5–11 pairing gap이 있었으므로 detector 학습 정답은 Capture 파일을 사용한다. 일반 Real
Time raw bag은 clustering/tracking과 통합 성능 재현에 사용한다.

### 실행

```bash
cd "$ASMC"
source devel/setup.bash

# 권장 시작값: 1 Hz, Ctrl-C 전까지 계속 수집
roslaunch data_collection capture_mode_collector.launch capture_hz:=1.0

# 첫 QA: 정확히 100 capture 후 종료
roslaunch data_collection capture_mode_collector.launch \
  capture_hz:=1.0 max_captures:=100 run_id:=kcity_capture_001

# 정차 중 중복 프레임 제외: /Ego_topic이 반드시 필요
roslaunch data_collection capture_mode_collector.launch \
  capture_hz:=1.0 only_when_moving:=true require_ego:=true min_speed_mps:=0.2
```

Docker 안에서 MORAI가 Windows host에 있으면 다음처럼 주소를 덮어쓴다.

```bash
roslaunch data_collection capture_mode_collector.launch \
  grpc_address:=127.0.0.1 capture_hz:=1.0
```

### `run_id` 규칙

한 run은 주행 도중 `scenario/variant`, 날씨, simulator time, controller,
sensor profile이 바뀌지 않는 연속 수집 단위로 잡는다. 하나라도 바뀌거나 수집을
종료했다가 다시 시작하면 새 `run_id`를 사용한다.

```text
<YYYYMMDD>_<HHMMSS>_kcity_<scenario>_<weather>_sim<hh><am|pm>_<controller>
```

맨 앞 시각은 **수집 시작 KST**이고 `sim<hh><am|pm>`은 MORAI 환경 시각이다.
수집 시작 시각을 초 단위 고유 키로 사용하므로 별도 `runNNN`은 붙이지 않는다.
환경 시각을 확인하지 못한 run만 `sim_unknown`으로 기록한다.

예시:

```text
20260903_213829_kcity_comp_sample_sunny_sim11am_wheel
20260904_000940_kcity_comp_sample_foggy_sim1pm_wheel
20260904_101530_kcity_jam_static_block_sunny_sim3pm_mpc
20260904_111500_kcity_merge_base_foggy_sim5pm_mpc
```

- 영문 소문자, 숫자, `_`만 사용한다. 구현상 `-`도 허용하지만 `_`로 통일한다.
- `train/val/test`는 나중에 run 단위로 배정하므로 이름에 넣지 않는다.
- 중단 후 이어 찍더라도 기존 ID를 재사용하지 않고 새 수집 시작 시각으로 생성한다.
- rejected run을 포함해 과거 ID를 재사용하지 않는다. Windows `SensorData`의 같은
  custom filename과 충돌하거나 원본 lineage를 혼동할 수 있다.
- 현재 collector의 `meta.json`은 Hz/backend/controller snapshot은 남기지만 MORAI
  scenario 파일명, 날씨, NPC 구성은 자동 수집하지 않는다. 자동화 전까지는 run ID와
  별도 수집 대장에 scenario 파일/hash, weather/time, actor 구성, 특이사항을 기록한다.

동일한 `run_id`의 `$ASMC_DATA/capture_runs/<run_id>`가 이미 있으면 안전을 위해 시작이
실패한다. 디렉터리를 지우고 재사용하지 말고 새 수집 시작 시각을 사용한다.

기본 `backend:=auto`는 gRPC에 먼저 연결하고, 열려 있지 않으면 연결된
`/SaveSensorData` ROS subscriber로 전환한다. 특정 방식을 강제하려면
`backend:=grpc` 또는 `backend:=ros_topic`을 지정한다. ROS 방식은 publish 이후의
저장 완료 응답이 없으므로 100-frame SensorData 파일 QA가 특히 중요하다.

MORAI 이미지·GT는 Windows의 `SensorData`에 저장되는데 **루트가 두 개로 갈린다.**
`is_custom_file_name = true`인 캡처에서 카메라·GPS·IMU는
`C:\MoraiLauncher_Win\SensorData`로, 3D LiDAR는
`C:\MoraiLauncher_Win\MoraiLauncher_Win_Data\SaveFile\SensorData\LIDAR_5`로 간다.
모라이 자체 동작이며 센서셋에 저장 경로 필드가 없어 설정으로 바꿀 수 없다. 아래
`sync_capture_mode_data.py`의 `--sensor-root` 기본값은 앞의 루트만 가리키므로
**LiDAR는 동기화 대상이 아니다.** LiDAR Capture 형식과 과거 실측 상세는 repository 밖의
개인 분석 기록으로 분리돼 있으며, 팀 공용 파이프라인이 이를 사용하게 될 때 별도 versioned
contract와 importer를 이 저장소에 추가한다.

Linux에는 다음 lineage만 `$ASMC_DATA/capture_runs/<run_id>/` 아래 기록한다.

- `meta.json`: Hz, gRPC 대상, motion gate 등 run 설정
- `capture_manifest.jsonl`: custom filename, wall/ROS/MORAI sim time, Ego snapshot,
  최근 `/ctrl_cmd`, RPC 결과와 latency
- `summary.json`: 성공·실패·motion skip 집계

최근 `/ctrl_cmd`가 있으면 manifest의 `control_source=ros_ctrl`, 없으면
`manual_or_unknown`으로 기록한다. 이는 제어권을 변경하지 않고 사후에 MPC/휠 run을
구분하기 위한 메타데이터다.

### 주행 종료 후 Windows → WSL 동기화

동기화는 주행과 MORAI를 종료한 뒤 **WSL 호스트에서** 실행한다. 원본은 삭제하지 않으며,
manifest의 성공 capture마다 Camera 4대 × 4종 + GPS + IMU = 18파일이 모두 있는지와
PNG 원본 해상도를 먼저 검사한다.

```bash
cd "$ASMC"

# 복사 없이 전체 무결성 검사
python3 scripts/sync_capture_mode_data.py \
  --run-id kcity_auto_sunny_003 \
  --dry-run

# 검사 통과 후 복사. 중단되면 같은 명령으로 이어받는다.
python3 scripts/sync_capture_mode_data.py \
  --run-id kcity_auto_sunny_003 \
  --jobs 4
```

여러 run은 `--run-id`를 반복한다. 완료 데이터는
`$ASMC_DATA/datasets/<run_id>/`에 저장되고 `_SUCCESS`, `dataset.json`,
`manifest.jsonl`이 생성된다.

성공으로 기록됐지만 실제 파일 일부가 누락된 capture는 원본 manifest를 고치거나 파일을
삭제하지 않는다. 확인된 sequence를 명시적으로 제외하면 `dataset.json`에 원래 성공 수,
제외 수·sequence·custom name이 남는다.

```bash
python3 scripts/sync_capture_mode_data.py \
  --run-id <run_id> \
  --exclude-sequence <run_id>:128 \
  --exclude-sequence <run_id>:129
```

존재하지 않거나 성공하지 않은 sequence를 지정하면 동기화를 중단한다. 같은 옵션으로
`--dry-run`을 먼저 실행하고, 실제 복사에도 동일한 제외 목록을 사용한다.

```text
datasets/<run_id>/
  frames/{intensity,semantic,instance,depth}/{front,left,right,traffic_light}/
  state/{gps,imu}/
  lineage/{meta.json,summary.json,capture_manifest.jsonl}
  manifest.jsonl
  dataset.json
  _SUCCESS
```

TwinLiteNet+ 입력은 `intensity/{front,left,right}`, 라벨 원본은 대응하는
`semantic/{front,left,right}`다. `traffic_light`는 별도 신호등 모델용이며,
`instance`·`depth`는 향후 작업을 위해 원본으로 보존한다.

라벨 팔레트, `384×640` paired letterbox, run-based split과 QA 계약은
[`docs/contracts/perception/dataset-labeling-split.md`](../../docs/contracts/perception/dataset-labeling-split.md)를
따른다.

객체 포함 run의 Semantic 전수 감사는 `scripts/audit_semantic_capture.py`로 수행한다.
run/view별 공식 class pixel 수와 양성 sample 수, 미등록 색, actor-Instance 대응률 및
대표 QA overlay를 함께 기록한다. 대회 학습 클래스에서 빠지는 색은
`--require-zero-class blue_lane`처럼 명시해, 이후 데이터에 다시 나타나면 감사 명령이
실패하도록 한다.

### Native target + TwinLite cache bake

동기화가 끝난 run의 Silver PNG는 수정하지 않고, versioned derived 경로에
원본 해상도의 `lane/drivable/stopline` canonical mask를 먼저 생성한다. road-marking
v3 정책은 여기에 `background=0, white=1, yellow=2, stopline=3` categorical mask도
추가한다. 이후
TwinLite cache 단계가 이를 `384×640`으로 letterbox하고 `valid` mask와 복원
geometry를 만든다. 두 단계 모두 head 아래에서 `front/left/right` 폴더를 분리한다.

```bash
# 1) 모델 독립 native target
python3 scripts/bake_perception_masks.py \
  --data-root "$AIM_PROJECT/data" \
  --run-id kcity_drive_001 --stage native

# 2) 완료된 native target에서 TwinLite cache
python3 scripts/bake_perception_masks.py \
  --data-root "$AIM_PROJECT/data" \
  --run-id kcity_drive_001 --stage twinlite

# run 전체에 고르게 분포한 8-frame 육안 검사용 preview
python3 scripts/bake_perception_masks.py \
  --data-root "$AIM_PROJECT/data" \
  --run-id kcity_drive_001 --stage all --sample-frames 8 \
  --native-output-name perception_targets_native_preview_v1 \
  --output-name twinlite_384x640_preview_v2 --write-previews
```

기본 출력은 `derived/perception_targets_native_v1/`과
`derived/twinlite_384x640_v2/`이다. 완료 전에는 최종 디렉터리가 노출되지 않으며,
기존 파생물을 교체하려면 `--overwrite`를 명시해야 한다. `{0,1}` canonical mask는
일반 이미지 뷰어에서 거의 검게 보이므로 육안 검사는 `qa_overlays/<view>/`를 사용한다.

객체 포함 v2 데이터는 먼저 정차 근접 중복을 비파괴 방식으로 선별한 뒤 frozen 정책을
명시해 bake한다. 기본 정책은 pose 변화 `0.01 m` 미만이 3 frame 이상 이어질 때 그룹으로
보고 시작/끝, actor·stopline 변화 frame, 매 5번째 frame을 남긴다. 원본 `frames/`와
동기화 manifest는 삭제하거나 수정하지 않는다.

```bash
# --run-id는 대상 신규 run마다 반복
python3 scripts/curate_perception_frames.py \
  --data-root "$AIM_PROJECT/data" \
  --run-id <run_id> \
  --output-name curation_v2

python3 scripts/bake_perception_masks.py \
  --data-root "$AIM_PROJECT/data" \
  --run-id <run_id> \
  --stage all \
  --policy config/perception_target_policy_v2.json \
  --curation-name curation_v2 \
  --native-output-name perception_targets_native_v2 \
  --output-name twinlite_384x640_v3
```

v2 native에는 `lane`, `drivable`, `stopline`, `drivable_ignore`,
`actor_occupancy_gt`가 저장된다. v3 cache의 drivable valid는 geometric valid에서
actor ignore를 뺀 값이다. 기존 actor 없는 run은 `--curation-name` 없이 같은 v2 정책과
출력 이름으로 bake해 모든 frame을 유지한다.

2026-09-06 road-marking 준비본은 기존 산출물을 덮지 않고 다음 이름으로 생성한다.

```bash
python3 scripts/bake_perception_masks.py \
  --data-root "$AIM_PROJECT/data" \
  --run-id <run_id> \
  --stage all \
  --policy config/perception_target_policy_v3.json \
  --curation-name curation_v2 \
  --native-output-name perception_targets_native_v3 \
  --output-name twinlite_384x640_marking_v1
```

이 정책은 binary `lane`을 white/yellow presence 호환 target으로 유지하면서 별도
`road_marking` 4-class target과 all-view stopline valid를 저장한다. Blue는 MORAI 공식
팔레트 검사에는 남겨 두지만 대회 target에서는 제외하며, 한 픽셀이라도 발견되면 bake를
중단한다. `curation_v2`가 없는 기존 baseline run에는 해당 옵션만 빼고 실행한다.

Actor 픽셀에 non-drivable loss를 주는 B 파생본은 원본과 A를 덮지 않고
`config/perception_target_policy_v3_actor_negative.json`,
`perception_targets_native_v3_actor_negative`,
`twinlite_384x640_marking_actor_negative_v1` 이름으로 생성한다.

### Run-based 학습 데이터셋 manifest

각 run과 `front/left/right` 폴더는 물리적으로 합치지 않는다. 완료된 모델 캐시를
검사한 뒤, 여러 run을 논리적으로 묶는 버전 manifest만
`$ASMC_DATA/dataset_versions/<dataset_name>/`에 게시한다. manifest의 모든 파일 경로는
`$ASMC_DATA` 기준 상대경로이며, RGB는 복사하지 않고 원본 Intensity PNG를 참조한다.

```bash
python3 scripts/build_perception_dataset.py \
  --data-root "$AIM_PROJECT/data" \
  --dataset-name twinlite_morai_v1 \
  --train-run kcity_drive_001 \
  --train-run kcity_drive_002 \
  --train-run kcity_auto_sunny_003 \
  --val-run kcity_drive_003
```

기본 활성 head는 하위 호환을 위해 `lane,drivable`이다. 4-class 학습 버전은
`--training-head drivable --training-head road_marking`을 명시한다. `stopline`과 binary
`lane` target도 비교·확장을 위해 manifest에 보존할 수 있다. 생성기는 run이 두 split에 중복되지 않는지, 모든
RGB/mask가 존재하는지, mask 크기가 `384×640`인지, binary 값이 `{0,1}`인지,
road-marking 값이 `{0,1,2,3}`인지 전수 검사한다. 기존 dataset version을 교체할 때만
`--overwrite`를 명시한다.

조건 전환 등 명확한 결함 frame은 원본/cache를 삭제하지 않고 capture frame 전체를
manifest에서 제외한다. 같은 frame의 front/left/right가 함께 빠지며 dataset metadata에
제외 목록과 sample 수가 남는다.

```bash
python3 scripts/build_perception_dataset.py ... \
  --exclude-frame <run_id>:0 \
  --exclude-frame <run_id>:1
```

2-head A 이력 버전은 `twinlite_morai_v4_comp8_train_val_sample_b`이다.
기존 12개 run 전체를 train 26,001 samples로, 이후 별도 한 바퀴에서 조건별로 나눈
`comp_sample_b` 6개 run을 val 1,404 samples로 구성한다. 첫 sunny 11am의 실제 날씨 전환
4 capture(12 samples)는 명시적으로 제외했다. test는 아직 비워 둔다.
정확한 run 목록과 manifest hash는
`$AIM_PROJECT/data/dataset_versions/twinlite_morai_v4_comp8_train_val_sample_b/dataset.json`을
기준으로 한다. 이전 v2/v3 dataset은 학습 전 기준 이력으로 보존한다.

Road-marking/A-B 준비 버전은
`twinlite_morai_v5_marking_prep_actor_ignore`이다. v4와 같은 train 26,001/val 1,404
sample 및 4-frame 제외를 유지하고, 활성 head도 아직 `lane,drivable`이다. 차이는
`road_marking` 4-class와 all-view `stopline`을 future target으로 연결하고 Blue 0-pixel
정책을 적용했다는 점이다. 이 버전을 actor-ignore(A) 기준으로 삼고, B는 split과 모델
초기값을 고정한 채 drivable actor 픽셀의 valid 정책만 바꾼
`twinlite_morai_v5_marking_actor_negative`다. 통제 평가에서 B가 actor leak을 크게 줄이고
일반 drivable/lane IoU가 회귀하지 않아 현재 live의 잠정 기본으로 채택했다.

다음 4-class 학습 manifest는 `twinlite_morai_v6_road_marking_actor_negative`다. B와
같은 26,001/1,404 split, 4-frame 제외와 cache를 사용하고 활성 head만
`drivable,road_marking`으로 바꾼다. dataset metadata의 `target_encodings`와 각 sample의
encoding은 `background=0, white_lane=1, yellow_lane=2, stopline=3`으로 고정된다.

기존 체크포인트를 학습 없이 새 validation에 평가하는 명령은 다음과 같다.

```bash
python3 scripts/evaluate_twinlite.py \
  --data-root /data \
  --dataset-version twinlite_morai_v4_comp8_train_val_sample_b \
  --split val \
  --checkpoint artifacts/perception_eval/runs/\
twinlite_medium_morai_v1_uniform_b12_e10_20260902/best_mean.pt \
  --output-dir artifacts/perception_eval/baselines/\
twinlite_medium_morai_v1_on_v4_sample_b_val_20260904
```

이 명령은 `eval()`과 `torch.no_grad()`만 사용하고 optimizer를 만들지 않으며,
`metrics.json`에 `training_performed=false`, 체크포인트 SHA-256과 실제 사용한 EMA/model
state를 기록한다.

### Hz 정책

- 기본값은 **1 Hz**다. 현재 4 Camera + LiDAR + GPS + IMU 1세트 23파일의 실측 쓰기
  완료가 약 0.50~0.57초였기 때문에 안전 여유를 둔 값이다.
- 2 Hz 이상은 `1 → 2 → 5 → 10 Hz`, 각 100회 순서로 파일 누락·부분 파일·SIM FPS를
  검사한 뒤 사용한다. 2 Hz를 넘기면 노드도 미검증 경고를 출력한다.
- 스케줄이 밀려도 catch-up capture를 연속 호출하지 않는다.
- gRPC 실패가 기본 3회 연속 발생하면 자동 중단한다.

실제 저장 없이 manifest·motion gate만 확인하려면 `dry_run:=true`를 사용한다.

## ROS topic snapshot collector 참고

아래 내용은 `sync_collector.launch`에만 적용된다. Capture Mode 기반 현재 학습 정본과
혼동하지 않는다.

### 선행 조건

1. MORAI + UDP bridge → 내부 ROS 토픽 (`aim_ws-va` `morai_*_bridge`)
2. 카메라 토픽 이름이 브릿지와 다르면 `config/profiles/*.yaml`의 `topics:` 수정
3. 대회 ROS UI에는 TL Get·센서 ROS가 없을 수 있음 → `missing[]`로 기록되고 계속 진행

### 모드

현재: `latest_timer` (최신 메시지 래치 후 `collect_hz` 스냅샷).
Time Manager sync tick은 구현되지 않은 후속 후보이며, MORAI
[Synchronous Mode](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/26.R1/synchronous-mode)를
기준으로 별도 pilot 뒤 채택한다.

### lane/drivable 수집 구현 상태

Capture Mode 동기화본 → native target → TwinLite cache → run-based dataset manifest
경로는 구현되었다. 아래 항목은 Capture Mode 이전의 ROS 토픽 기반 collector에 남아 있는
제약이며, 현재 학습 정본에는 적용하지 않는다.

- RGB 카메라는 저장하지만 Semantic 카메라 구독·paired timestamp가 없음
- writer는 카메라별 원본 header stamp/shape/camera_id를 manifest에 기록하지 않음
- `latest_timer`는 RGB↔Semantic 정합을 보장하지 않음
- 카메라 blob을 `.jpg`로 저장하므로 Semantic 무손실 PNG/ID map writer가 별도로 필요
- `lane_capture` / `lane_stream_gt` profile은 아직 없음

과거 YOLOPv3 WP와 Bronze/Silver/Gold 수집 설계는 현재 계획이 아니다. 현재 구현 순서와
후속 gate는 [인지 통합 로드맵](../../docs/packages/asmc_perception/roadmap.md), native 원본·mask·split
규칙은 [데이터셋 계약](../../docs/contracts/perception/dataset-labeling-split.md)을 따른다.
