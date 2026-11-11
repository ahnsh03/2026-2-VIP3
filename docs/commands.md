# 실행 명령 모음

기준일 **2026-11-11**. 팀원이 구현한 실행법은 이 문서에 모은다. **주차 기본 컨테이너는 `vip3-parking-ros`, 이미지 `vip3/parking-ros:noetic`, ROS Noetic**이다. 기본 이미지에는 PyTorch/CUDA가 없다.

| 목적 | 바로가기 |
|---|---|
| 처음 설치·컨테이너 생성 | [환경 구성](#setup) |
| 매번 연결·센서 hz 확인 | [bridge](#bridge) |
| SVM 표시·마스크 검사·저장 | [SVM](#svm) |
| 수집·bag 감사·재생 | [데이터](#collection) |
| 상태·기어·지도·평가 | [개발 패키지](#packages) |
| 과거 TwinLite·핀홀·Capture 도구 | [선택 실행](#legacy) |
| SS-PSD 외부 코드 | [인지 연결 준비](#ss-psd) |
| 종료·점검·추가 양식 | [운영](#operations), [작성 규칙](#contributing) |

`호스트`는 WSL/리눅스 터미널, `컨테이너`는 `docker exec`로 들어간 터미널이다. 호스트의 `./scripts/...`·`python3 tools/...`는 **저장소 루트 `2026-2-VIP3`**에서 실행한다. 상위 프로젝트 폴더에는 이 파일들이 없다. 컨테이너 입장 시 작업 위치는 `/root/ws`이므로 다시 `cd`할 필요 없다. `<...>`는 실제 값으로 바꿀 자리이며 그대로 실행하지 않는다.

<a id="setup"></a>
## 1. 환경 구성 — 호스트

Docker Engine 또는 Docker Desktop의 WSL 연동, Compose v2, Git, Python3가 필요하다. 기존 clone이 있으면 clone 단계는 생략한다.

```bash
git clone --recurse-submodules https://github.com/ahnsh03/2026-2-VIP3.git
cd 2026-2-VIP3
git submodule update --init --recursive

export VIP3_CONTAINER_NAME=vip3-parking-ros
export MORAI_BRIDGE_PORT=9090
./scripts/docker_ros_up.sh build       # 이미지 빌드·검증; 컨테이너 생성 아님
./scripts/docker_ros_up.sh up          # 컨테이너 생성/시작
./scripts/docker_ros_up.sh build-ws    # catkin 워크스페이스 빌드
./scripts/docker_ros_up.sh status
```

새 터미널에서는 호스트의 어느 디렉터리에서든 다음으로 입장한다. ROS·워크스페이스 환경은 interactive bash에 자동 적용된다. 빌드 직후 이미 열려 있던 셸은 `source /root/ws/devel/setup.bash`를 실행한다.

```bash
docker exec -it vip3-parking-ros bash
```

repo는 `/root/ws`, 호스트의 형제 `../data`는 `/data`로 연결된다. 소스 변경은 즉시 보이며 메시지·패키지·설치 항목 변경 후에는 다시 `build-ws`한다. Dockerfile/의존성을 바꾼 경우에만 이미지를 다시 빌드한다.

### 9090이 예약된 PC만: 19090

팀 정본은 9090을 유지한다. 포트 예외는 **호스트에서 컨테이너 생성 전에** 설정하고 MORAI에 로컬 프리셋을 Import한다. 기존 컨테이너의 포트를 바꾸는 `up`은 컨테이너를 재생성할 수 있으므로 실행 중인 작업을 먼저 끝낸다.

```bash
export VIP3_CONTAINER_NAME=vip3-parking-ros
export MORAI_BRIDGE_PORT=19090
python3 tools/export_local_morai_presets.py --port 19090 --output-dir ../data/local_morai
./scripts/docker_ros_up.sh up
```

생성 파일은 `261110_VIP3_Sensor_26.R1.H3_local19090.json`, `261110_VIP3_Network_26.R1.H3_local19090.json`이다. 컨테이너 안에서 생성한다면 `--output-dir /data/local_morai`를 쓴다. 셸의 환경변수만 바꿔서는 Docker 포트 매핑이 바뀌지 않는다.

<a id="bridge"></a>
## 2. MORAI 연결과 진단

**터미널 A / 컨테이너** — 실행 상태를 유지한다.

```bash
./scripts/rosbridge.sh
```

MORAI에서 `R_KR_PR_K-city_2025`, `2023_Hyundai_ioniq5`를 선택한다. [Sensor JSON](../config/morai/261110_VIP3_Sensor_26.R1.H3.json)과 [Network JSON](../config/morai/261110_VIP3_Network_26.R1.H3.json)을 Import하고, 네트워크·카메라 ROS **Connect → Play**를 누른다. 기본 접속은 Windows `127.0.0.1:9090`이다. 위 예외 PC는 두 로컬 19090 프리셋을 사용한다.

기본 **5초마다** 카메라·상태의 수신율, 마지막 수신 이후 시간, `OK/MISSING/STALE` 진단이 출력된다. 측정 hz는 설정 hz와 다를 수 있다. 진단 주기는 시작할 때 변경한다.

```bash
# 대체 실행: 10초마다 요약
MORAI_TOPIC_LOG_INTERVAL=10 ./scripts/rosbridge.sh
# 대체 실행: 요약 노드 끄기
MORAI_TOPIC_MONITOR=false ./scripts/rosbridge.sh
```

**터미널 B / 컨테이너** — bridge가 실행된 뒤 확인한다.

```bash
python3 scripts/verify_morai_topics.py --duration 15 --profile runtime
python3 scripts/verify_morai_topics.py --duration 15 --profile cameras --json-out /data/sensor-check.json
rostopic hz /cam_front/image_jpeg/compressed
# Ctrl+C 후 다른 view도 확인
rostopic echo -n 1 /Ego_topic
./scripts/diagnose_morai_rosbridge.sh
```

호스트 저장소 루트에서는 `./scripts/docker_ros_up.sh check`로 포트 매핑·도달 상태를, `./scripts/docker_ros_up.sh verify 15`로 수신을 확인한다. `MISSING`이면 MORAI의 네트워크·각 카메라 Connect/Play, 프리셋 토픽과 포트부터 확인한다.

직접 launch를 사용할 때 아래는 **스크립트 대신** 실행한다. 같은 포트에 bridge를 두 번 띄우지 않는다.

```bash
roslaunch vip3_bringup morai_rosbridge.launch port:="$MORAI_BRIDGE_PORT" monitor_interval:=5
```

<a id="svm"></a>
## 3. SVM 화면과 진단

**터미널 B / 컨테이너**, 4카메라 수신 후:

```bash
roslaunch drivable_bev fisheye_svm.launch
```

**터미널 C / 컨테이너** — WSLg/X 표시 환경에서:

```bash
vip3-ros-gui rqt-image /perception/svm/panel
# 아래는 다른 화면을 볼 때의 대안
vip3-ros-gui rqt-image /perception/svm/preview/compressed
rqt_image_view --clear-config /perception/svm/valid_mask _image_transport:=raw
```

기본 출력 상한은 **10Hz**다. 필요하면 `roslaunch drivable_bev fisheye_svm.launch publish_hz:=5`로 낮춘다. 센서 JSON이나 보정값을 바꿀 필요는 없다.

`panel`은 네 원본과 SVM·격자를 합친 화면이다. 검은 화면이면 bridge 진단, JPEG 해상도/frame, 네 영상의 시각차를 먼저 확인한다. 헤드리스 환경에서는 아래 저장 결과로 확인한다.

**기존 SVM을 Ctrl+C로 끝낸 뒤** 대체 실행한다. 같은 이름의 SVM 노드를 중복 실행하지 않는다.

```bash
roslaunch drivable_bev fisheye_svm.launch \
  vehicle_overlay:=false vehicle_mask:=false \
  save_dir:=/data/svm_checks/no_mask_01
```

첫 유효 묶음 한 번을 저장한다. 새 실험은 다른 디렉터리를 사용한다.

| 파일 | 용도 |
|---|---|
| `front.png`, `rear.png`, `left.png`, `right.png` | 사용한 원본 영상 |
| `svm_panel.png`, `svm_preview.png` | 사람이 보는 진단 결과 |
| `svm_projection.png` | 그림·문구 없는 투영; 차체 마스크는 적용 |
| `svm_unmasked.png` | `vehicle_mask:=false`에서만 저장하는 마스크 해제 투영 |
| `valid_mask.png`, `config.yaml`, `capture.json` | 유효 영역·설정·영상 시각·보간 자세·투영 상태 |

기본 `261111_kcity_1280x960_v1`은 **일부 정차 자세에서 검증한 preview**이며 clean BEV 학습/인지 토픽은 아직 없다. 마스크를 꺼도 차체 영역이 관측 가능한 지면으로 바뀌지 않는다. 기본 차량 그림을 다시 보려면 첫 명령으로 재시작한다.

호스트 저장소 루트에서도 `./scripts/docker_ros_up.sh bridge`, `svm`, `view`를 **각 터미널에서 하나씩** 실행할 수 있다.

<a id="svm-calibration"></a>
### SVM 자세·렌즈·정합 검사

**컨테이너 `/root/ws`**, SVM 실행 중 상태 확인:

```bash
rostopic echo /perception/svm/status
rostopic hz /perception/svm/preview/compressed
```

status의 `timing`에서 `input_age_ms`, `maps_ms`, `processing_ms`, `received_to_publish_ms`를 확인한다. 이 값은 로컬 수신 이후 시간이며 MORAI/네트워크/GUI 전체 지연이 아니다. 약 5초 주기 로그에도 출력한다. [성능 재현과 해석](system/svm-performance.md).

`calibration_profile: 261111_kcity_1280x960_v1`, `projection_surface_offset_m: 0.01`이 현재 기본값이다. 화면의 `EDGE FIT | PREVIEW`로 적용 여부를 확인한다. 센서 JSON을 다시 Import할 필요는 없으며 SVM만 재시작한다.

`pose compensated`는 자세·촬영 시차 반영을 뜻하며 렌즈/지면 보정 완료 판정이 아니다. 현재 MORAI quaternion 어댑터와 지면 가정은 [보정 문서](system/svm-calibration.md)에 설명했다.

앞에서 저장한 묶음을 **새 출력 경로**로 재생한다. 첫 명령은 저장된 평면·카메라 상대 자세를 그대로 적용하고, 두 번째는 정적 투영과 비교한다.

```bash
python3 tools/replay_fisheye_svm.py /data/svm_checks/no_mask_01 \
  --output-dir /data/svm_checks/replay_01
python3 tools/replay_fisheye_svm.py /data/svm_checks/no_mask_01 --static \
  --output-dir /data/svm_checks/static_01
```

`--config <새 YAML>`로 비교하면 저장된 자세·시차를 유지하고 새/기존 프로필의 투영면 오프셋 차이만 한 번 더한다. `--config` 없이 재생하면 당시 투영면을 그대로 쓴다.

각 카메라의 `*_warped.png`, `svm_projection.png`, `valid_mask.png`와 재생 조건을 저장한다. 이전 캡처에 `projection_state`가 없으면 `--static`만 가능하다.

실측 대응점 JSON을 준비한 뒤 후보를 추정한다. **아래 observations.json은 직접 작성해야 하는 입력**이며 형식·분할 규칙은 보정 문서를 따른다.

```bash
python3 tools/calibrate_fisheye_svm.py \
  --observations /data/calibration/observations.json \
  --output /data/calibration/candidate.yaml \
  --report /data/calibration/report.json
# 후보 파일이 생성된 경우 기존 SVM을 Ctrl+C로 종료한 뒤 비교 실행
roslaunch drivable_bev fisheye_svm.launch config:=/data/calibration/candidate.yaml
```

원본 설정과 기존 결과는 덮어쓰지 않는다. 기준 미달이면 report만 생성하고 종료 코드 2를 반환한다. 통과 후보도 다른 위치에서 검증한 뒤 정본에 반영한다.

<a id="collection"></a>
## 4. 원본 수집·감사·재생 — 김동현

bridge·MORAI 연결 후 **컨테이너**에서 실행한다. `scenario`와 `controller`는 녹화 필수 인자다. 날씨·시뮬 시간은 실제 설정을 적는다. run ID를 생략하면 자동으로 만들어 화면에 표시한다.

```bash
roslaunch data_collection bag_recorder.launch \
  profile:=parking_raw map_slug:=kcity2025 \
  scenario:=slot002_front controller:=manual weather:=sunny sim_hour:=11
```

Ctrl+C로 정상 종료해 bag과 메타데이터를 마감한다. 출력은 `/data/rosbags/raw/<run_id>/bags/*.bag`과 같은 run의 metadata·summary다. 기본 여유 공간 기준 10GiB, 파일 분할 4GiB다. 평가용 object/collision도 담으려면 위 명령의 `profile:=parking_gt`를 선택하고 인지 입력과 분리한다.

```bash
python3 src/data_collection/scripts/audit_rosbag_run.py \
  /data/rosbags/raw/<run_id> --strict \
  --json-out /data/analysis/<run_id>-audit.json
```

기본 camera rate 점검 기준은 8Hz다. run 전체·개별 파일 모두 검사 가능하며 rate·header 역행·bag/header 시차를 확인한다. 이 감사기는 JPEG 디코드 성공이나 네 RGB 카메라 사이의 묶음 시각차까지 검사하지 않는다. 그 부분은 SVM 재생 화면과 `capture.json`의 `span_ms`로 추가 확인한다.

### 재생

실제 MORAI 송신을 Disconnect하고 live bridge·상태·제어 노드를 종료한 별도 ROS 세션에서 재생한다. live와 bag의 같은 토픽을 동시에 발행하지 않는다. bag에는 `/ctrl_cmd`도 포함될 수 있으므로 연결된 시뮬에 재생 명령이 전달되지 않게 한다.

```bash
# 터미널 A: /use_sim_time을 설정하고 일시정지 상태로 준비
roslaunch data_collection bag_replay.launch \
  bag_path:=/data/rosbags/raw/<run_id> required_profile:=parking_raw
# 터미널 B: SVM 등 소비자 준비
roslaunch drivable_bev fisheye_svm.launch
# 터미널 C: 소비자가 준비된 뒤 재생 시작
rosservice call /vip3_bag_replay/start
```

GT 포함 bag은 명시적으로 `allow_simulator_gt:=true`를 추가한다. Ctrl+C로 종료하면 replay가 `/use_sim_time`을 복원한다.

### 간단한 snapshot

```bash
# ego만 보는 짧은 연결 확인
roslaunch data_collection sync_collector.launch \
  profile:=dry_run map:=R_KR_PR_K-city_2025 max_frames:=20
# 4어안+ego, 평가용 objects/collision 포함
roslaunch data_collection sync_collector.launch \
  profile:=parking_core map:=R_KR_PR_K-city_2025 max_frames:=30
```

`parking_core`는 5Hz latest-timer snapshot이며 정확한 네 영상 동기화를 보장하지 않는다. 현 센서셋에 없는 LiDAR/GPS/IMU를 기다리는 다른 프로파일을 기본으로 쓰지 않는다.

<a id="packages"></a>
## 5. 주차: 상태·기어·GT·평가 — 강도균·김동현

### 상태 관측

패키지 통합 이전 checkout을 사용했다면 컨테이너에서 `./scripts/build_ws.sh` 후 `source devel/setup.bash`를 한 번 실행한다. 상태·기어·GT·평가 코드는 `vip3_parking`에 모였으며 `/vehicle/*`, `/vip3_gear/set_gear` 이름은 유지한다.

```bash
# 컨테이너: 관측만, External 모드 전환 없음
roslaunch vip3_parking vehicle.launch start_gear_node:=false
rostopic echo -n 1 /vehicle/state
rostopic echo -n 1 /vehicle/odom
```

실행 중인 launch와 `rostopic`은 별도 터미널을 쓴다. source stamp가 0이면 노드가 거부한다. 필요한 개발 실험에서만 `stamp_mode:=receive`를 추가하고 수신 시각을 사용했다고 기록한다.

```bash
# 대체 실행: 기어 서비스까지 켬. 기본값은 External 제어 모드 전환 요청
roslaunch vip3_parking vehicle.launch
# External로 전환하지 않고 기어 노드를 준비할 때
roslaunch vip3_parking vehicle.launch set_external_on_start:=false
```

정차 상태에서 기어를 명시적으로 바꾸는 예:

```bash
rosservice call /vip3_gear/set_gear "{gear: 4}"  # D
rosservice call /vip3_gear/set_gear "{gear: 1}"  # P
```

코드는 P=1, R=2, N=3, D=4다. 응답 성공과 `/vehicle/gear`를 확인한다. 주차 제어 노드는 아직 없으며, 이 명령만으로 경로 추종은 시작되지 않는다.

### bringup과 TF

```bash
# 상태 관측을 bringup으로 켜는 대안; 위 vehicle.launch와 중복 실행하지 않음
roslaunch vip3_bringup vip3_bringup.launch static_tf:=false gear_node:=false
# MORAI가 카메라 TF를 발행하지 않는 별도 환경에서만
roslaunch vip3_bringup static_tf.launch
```

현재 MORAI의 `map→base_link`를 사용하며 `publish_map_tf` 기본값은 false다. 카메라 static TF도 같은 transform을 중복 발행하지 않는다. SVM만 볼 때는 이 bringup이 필요 없다.

### GT 읽기

```bash
# 호스트 저장소 루트 또는 컨테이너, ROS·전체 도로망 없이 실행
python3 tools/inspect_parking_gt.py
python3 tools/inspect_parking_gt.py --id A5256W000008 --id A5256W000009
```

원본 해시·31개 레코드 수·points 기반 크기와 방향을 확인한다. `--id`는 배열 순번이 아닌 정확한 `idx`다. 출력 angle은 180° 대칭인 긴 변 방향이며 입구/전방 진입을 정하지 않는다. 제공 좌표는 변경하지 않으므로 차량 pose와의 정합은 [데이터 문서](system/data.md)에 따라 확인한다. 전체 도로망·차선·RViz 지도 표시는 제거했다.

### 평가 함수와 메시지

`vip3_parking.evaluation`은 ROS 없는 평가 모듈이다. 실제 결과 데이터에 `evaluate_parking()`을 호출한다. 다음은 실행 확인용 테스트이며 실제 주차 성능 평가가 아니다.

```bash
PYTHONPATH=src/vip3_parking/src python3 -m unittest discover -s src/vip3_parking/test
rosmsg show vip3_msgs/VehicleState
rossrv show vip3_msgs/SetGear
rosmsg show morai_msgs/CtrlCmd
```

<a id="legacy"></a>
## 6. 기존 구현의 선택 실행

아래는 이전 프로젝트에서 이식한 기능이다. **현재 주차 기본 실행에 필요하지 않으며 179° 어안용 SS-PSD 경로와 구분한다.** 존재하는 코드를 찾을 수 있게 명령을 유지하되, GPU 의존성·구형 데이터·핀홀 설정은 별도로 준비해야 한다.

### TwinLite ROS 추론·시각화

ROS와 PyTorch, 해당 upstream 소스·checkpoint가 있는 **별도 추론 환경**에서 실행한다. 기본 주차 컨테이너에는 모델 의존성이 없다. 환경 구성 참고는 [legacy Dockerfile](../docker/ros-noetic/Dockerfile.legacy-twinlite)이며 현재 장비에서 재검증해야 한다.

```bash
# 한 뷰 또는 네 뷰 중 하나 선택
roslaunch camera_semantic_perception single_view.launch view:=front device:=cuda
roslaunch camera_semantic_perception four_view.launch device:=cuda
roslaunch camera_semantic_perception four_view_with_visualizer.launch device:=cuda
# 추론과 별도로 시각화만
roslaunch camera_semantic_perception single_view_visualizer.launch view:=front
roslaunch camera_semantic_perception four_view_visualizer.launch
# 공통 진입점: 위 래퍼와 중복 실행하지 않는다
roslaunch camera_semantic_perception semantic_inference.launch views:=front device:=cuda
roslaunch camera_semantic_perception semantic_visualizer.launch views:=front
```

기본 checkpoint는 `weights/twinlite/v5_binary_lane/best_mean.pt`다. `checkpoint:=...`로 변경한다. 도로·차선 segmentation 출력이며 주차 슬롯 검출 메시지가 아니다.

### 핀홀 BEV 코어

현재 어안 프리셋에 사용하지 않는다. **검증한 핀홀 보정 파일·그와 일치하는 센서 JSON**이 있어야 한다. 테스트 fixture를 실행 정본으로 사용하지 않는다.

```bash
roslaunch drivable_bev rgb_bev.launch camera_config:=<핀홀-보정.yaml>
roslaunch drivable_bev camera_bev.launch \
  camera_config:=<핀홀-보정.yaml> sensor_set_path:=<핀홀-센서.json>
roslaunch drivable_bev camera_bev_fusion.launch camera_config:=<핀홀-보정.yaml>
roslaunch drivable_bev camera_bev_visualizer.launch camera_config:=<핀홀-보정.yaml>
# 위 투영·융합·시각화와 TwinLite를 한 번에 켜는 대안
roslaunch drivable_bev four_view_bev.launch \
  camera_config:=<핀홀-보정.yaml> sensor_set_path:=<핀홀-센서.json>
```

### 구형 Capture·semantic-mask 데이터

현재 4어안 수집에는 적용하지 않는다. `capture_sync`가 기대하는 **기존 v1 카메라 ID·해상도와 GPS/IMU 파일 구성**을 갖춘 과거 run용이다. Capture publish 성공은 파일 저장 성공을 뜻하지 않아 동기화 전 `--dry-run` 검사가 필요하다.

```bash
# 컨테이너, MORAI Capture Mode 설정 후
roslaunch data_collection capture_collector.launch capture_hz:=1.0 run_id:=<run_id>
# 호스트 저장소 루트, 과거 Capture run을 검사·변환할 때
python3 scripts/sync_capture_data.py --run-id <run_id> --dry-run
python3 scripts/sync_capture_data.py --run-id <run_id> --jobs 4
python3 scripts/audit_semantic_capture.py --run-id <run_id>
python3 scripts/curate_perception_frames.py --run-id <run_id>
python3 scripts/bake_perception_masks.py --run-id <run_id> --stage all \
  --policy config/perception/target_policy_v1_katri_4view.json
python3 scripts/build_perception_dataset.py \
  --train-run <train_run> --val-run <val_run> --test-run <test_run>
```

이 결과는 semantic mask이며 SS-PSD 방향 접점·슬롯 라벨이 아니다.

### TwinLite 학습·평가

NVIDIA GPU Docker 환경과 upstream `../external/baselines/TwinLiteNetPlus`(커밋 `90f1b8695ae311d5123b05f8534b2e11e42499d2`), 위 dataset version이 있어야 한다. 별도 **학습 컨테이너**이며 ROS 주차 컨테이너와 다르다.

```bash
# 호스트 저장소 루트
./scripts/docker_train_up.sh build
./scripts/docker_train_up.sh up
./scripts/docker_train_up.sh shell
# 이후 학습 컨테이너 /root/ws
python3 scripts/smoke_twinlite_data_loss.py --dataset-version <dataset> --batch-size 2
python3 scripts/train_twinlite_tiny.py --dataset-version <dataset> \
  --selection-file config/perception/twinlite_tiny_overfit_samples.json --batch-size 2
python3 scripts/train_twinlite.py --dataset-version <dataset> --batch-size 2 --num-workers 0
python3 scripts/evaluate_twinlite.py --dataset-version <dataset> --split test \
  --checkpoint <checkpoint.pt> --output-dir artifacts/perception_eval/<run>
python3 scripts/render_twinlite_predictions.py --dataset-version <dataset> --split test \
  --checkpoint <checkpoint.pt> --output-dir artifacts/perception_eval/<run>/overlays
```

tiny selection 파일의 프레임이 해당 dataset에 있는지 먼저 확인한다. 학습 컨테이너의 `/data`는 읽기 전용, 학습 산출물은 Git 제외 경로 `artifacts/`에 쓴다. 분리 실행이 필요하면 호스트에서 다음을 사용한다.

```bash
./scripts/docker_training_job.sh start <job> -- python3 scripts/train_twinlite.py \
  --dataset-version <dataset> --batch-size 2 --num-workers 0
./scripts/docker_training_job.sh status <job>
./scripts/docker_training_job.sh logs <job> 50
./scripts/docker_training_job.sh list
```

<a id="ss-psd"></a>
## 7. SS-PSD 연결 준비 — 하수영

현재 팀 ROS launch는 없다. 공식 clone은 `../external/parking/CRPS-D`이며 주차 컨테이너에 자동 마운트되지 않는다. 별도 Python/PyTorch 환경에서 공식 requirements·공개 가중치를 준비한다. 확인한 upstream에는 다음 수정이 필요하다.

- `inference.py`의 `detect_image()`가 `/data/cylin/zzh/DMPR/VAE/output/`의 JPG를 읽고 `test/`에 저장한다. 입력 경로를 자기 데이터로 바꾸고 출력 폴더를 만든다. `--image_directory` 옵션은 없다.
- `inference.py`는 `cuda:4`, `ps_evaluate.py`는 `cuda:2`를 고정 사용한다. 단일 GPU에서는 코드의 device 선택을 고쳐야 한다. 아래는 먼저 배관을 확인하기 위한 **CPU 옵션**이며 GPU 성능 측정 명령이 아니다.
- 평가·학습 데이터 경로는 `yaml/data_root.yaml`에 있다. 라벨과 이미지 구성을 맞춘 뒤 실행한다.

```bash
# 외부 CRPS-D 저장소 루트에서, 의존성/입력/가중치를 준비한 뒤
python inference.py --mode image --detector_weights <공개-가중치> --inference_slot --disable_cuda
python evaluate.py --detector_weights <공개-가중치> --disable_cuda
python ps_evaluate.py --detector_weights <공개-가중치> --disable_cuda
```

위는 upstream CLI로, 팀 환경에서 성공 확인한 ROS 실행법을 뜻하지 않는다. `python train.py` 기본 설정은 6/8GB VRAM에 맞춘 지도 추가 학습 명령이 아니므로, 학습 경로와 batch·라벨 형식을 검증한 뒤 이 절에 확정 명령을 추가한다. [베이스라인 범위](project/baseline.md)

<a id="operations"></a>
## 8. 종료와 정적 점검

종료 순서는 제어·수집 → SVM·상태·뷰어 → bridge다. 각 실행 터미널에서 Ctrl+C로 끝내고 `exit`로 셸을 닫는다. 사용 중인 다른 프로젝트 컨테이너나 전체 ROS 프로세스를 일괄 종료하지 않는다.

```bash
# 호스트 저장소 루트, 이 프로젝트 컨테이너만
./scripts/docker_ros_up.sh stop       # 중지, 컨테이너 유지
# 다시 사용
./scripts/docker_ros_up.sh up
# 컨테이너 자체가 필요 없을 때만 (이름 지정된 build/devel 볼륨은 유지)
./scripts/docker_ros_up.sh down
```

정본 설정·실행 배선 변경 후 아래를 확인한다. 호스트에 PyYAML 등 검사 의존성이 없으면 컨테이너에서 실행한다.

```bash
python3 tools/check_sensor_preset.py
python3 tools/check_topic_contract.py
python3 tools/check_node_imports.py
python3 tools/check_launch_params.py
python3 scripts/verify_morai_topics.py --list  # ROS master 없이 계약만 확인
```

<a id="contributing"></a>
## 9. 팀원이 명령을 추가하는 양식

해당 파트 절에 아래 내용을 채우고, 입력 bag/설정 버전과 함께 검증한다. 여러 대안은 동시 실행인지 하나만 선택하는지 표시한다. 새 패키지가 없는데 예정 launch를 만들어 넣지 않는다.

```text
기능 / 담당:
확인 날짜 / 코드 커밋:
실행 위치: 호스트 저장소 루트 | 컨테이너 /root/ws | 별도 환경
선행 조건: bridge, 입력 토픽, 모델/데이터/보정 파일
실행 명령:
정상 출력: 토픽·화면·파일·서비스 응답
종료 방법:
확인한 조건 / 남은 제약:
```
