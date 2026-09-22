# Camera semantic perception

> 상태: Stage C1·C2 완료, Stage C3 구조화 diagnostics 구현
>
> 현재 인지 package 경계와 본선 입력 규칙은
> [Perception 개발·통합 가이드](../../../docs/packages/asmc_perception/README.md)를 함께 본다.
> 이 문서의 날짜별 실측은 해당 실행 시점의 기록이며 영구 성능 계약이 아니다.

MORAI의 전방 또는 전·좌·우 JPEG 카메라를 latest-only 방식으로 받아 TwinLiteNet+ 2-head
추론 결과를 표준 ROS 토픽으로 발행한다. Binary lane checkpoint와 4-class road-marking
checkpoint를 메타데이터로 자동 구분한다. 모델 로딩은 backend에 격리되어 이후 다른
segmentation 모델로 교체해도 ROS 입력·출력 배관을 유지한다.

## 현재 범위

- 선택 view: `front` 또는 `front,left,right`
- 학습과 동일한 RGB `[0,1]`, `384×640` letterbox
- front 1600×900 기준 상·하 12 px padding과 가변 원본 크기 자동 계산
- TwinLiteNet+ Medium, EMA 우선 체크포인트 로딩
- 체크포인트 SHA-256, epoch, upstream commit, weight 종류 시작 로그
- 원본 카메라 크기로 복원한 `mono8` 확률 또는 categorical class-ID 출력
- latest-only buffer와 최대 4 ms 3-view batch coalescing
- 2초 단위 FPS, stage별 평균과 queue/end-to-end p50·p95 구조화 diagnostics
- RGB와 drivable/secondary 출력을 원본 timestamp로 exact sync한 독립 visualizer
- view별 원본/drivable/lane 또는 road-marking 세로 panel과 전·좌·우 **3행×3열** mosaic
- 실행 중 바꿀 수 있는 head threshold와 날씨·시간 condition label

## 실행 컨테이너

UDP 브릿지는 `asmc-ros-noetic`, GPU 추론은 `asmc-perception-train`에서 실행한다. 두
컨테이너는 Docker Desktop host network의 같은 ROS master `127.0.0.1:11311`을 사용한다.
학습용 컨테이너에는 공식 TwinLiteNet+ checkout이 `/opt/baselines/TwinLiteNetPlus`로
읽기 전용 마운트된다.

체크포인트와 학습 run은 Git에 포함되지 않는다. 기본 launch는 actor 픽셀을 non-drivable
negative로 학습한 B 후보 `twinlite_medium_morai_v5_actor_negative_init_ema_b12_e18_20260906`
의 `best_mean.pt`를 가리킨다. 다른 PC에서는 공유받은 체크포인트 경로를
`checkpoint:=...`로 지정해야 한다.
체크포인트 내부 upstream commit과 외부 checkout commit이 다르면 안전하게 중단한다.

```bash
# WSL
./scripts/docker_ros_up.sh up
./scripts/docker_perception_up.sh up

# ROS 컨테이너: 최초 또는 패키지 변경 후
docker exec -it asmc-ros-noetic bash -lc './scripts/build_ws.sh'

# 별도 터미널: UDP 브릿지
docker exec -it asmc-ros-noetic bash
source devel/setup.bash
./scripts/udp_bridge.sh --simulator 26r1

# 별도 터미널: front-only GPU 추론
docker exec -it asmc-perception-train bash
source /root/ws/devel/setup.bash
roslaunch camera_semantic_perception front.launch
```

3-view 실행:

```bash
roslaunch camera_semantic_perception three_view.launch \
  checkpoint:=/root/ws/artifacts/perception_eval/runs/<run>/best_mean.pt
```

노드는 ROS 구독을 열기 전에 기본 1회 GPU warmup을 수행한다. live 주행과 bag replay 모두
CUDA 첫 호출 지연으로 초반 프레임이 대량 교체되는 것을 줄이기 위한 것이며, 배관만 확인할
때는 `warmup_iterations:=0`으로 끌 수 있다. bag 기반 A/B 평가는 paused replay controller가
`/use_sim_time=true`를 준비한 뒤 추론 노드를 시작해야 한다.

2026-09-08 4-class 모델을 확인할 때는 다음 checkpoint를 명시한다. Backend가 checkpoint의
task spec을 읽고 secondary classifier를 4-class로 구성한다.

```bash
roslaunch camera_semantic_perception three_view.launch \
  checkpoint:=/root/ws/artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt
```

기존 actor-ignore A 후보를 다시 확인할 때는 다음처럼 launch 기본값만 덮어쓴다.

```bash
roslaunch camera_semantic_perception three_view.launch \
  checkpoint:=/root/ws/artifacts/perception_eval/runs/twinlite_medium_morai_v4_comp8_init_ema_b12_e18_20260904/best_mean.pt
```

CPU 배관 시험은 `device:=cpu`로 덮을 수 있지만 실시간 성능 기준으로 사용하지 않는다.

## 실시간 화면 확인

시각화 노드는 모델·CUDA를 직접 사용하지 않는다. GPU 추론은 학습 컨테이너에서 실행하고,
시각화와 `rqt_image_view`는 ROS 컨테이너에서 별도로 실행한다. 원본과 예측의 timestamp가
완전히 같은 경우만 패널을 만들기 때문에 서로 다른 장면이 겹치지 않는다.

전방 단독 모드에서는 추론을 실행한 다음 다른 터미널에서 다음을 실행한다.

```bash
# 터미널 1: GPU 추론 컨테이너
docker exec -it asmc-perception-train bash
source /root/ws/devel/setup.bash
roslaunch camera_semantic_perception front.launch

# 터미널 2: ROS 컨테이너
docker exec -it asmc-ros-noetic bash
source /root/ws/devel/setup.bash
roslaunch camera_semantic_perception front_visualizer.launch

# 터미널 3: ROS 컨테이너(X11/WSLg 화면)
docker exec -it asmc-ros-noetic bash
source /root/ws/devel/setup.bash
rqt_image_view /perception/camera/front/debug/panel/compressed
```

전·좌·우 4-class 모드는 추론과 별도의 ROS 컨테이너에서 다음 visualizer를 실행하고
모자이크를 연다. Binary 모델을 볼 때는 `secondary_mode:=lane`을 사용한다.

```bash
roslaunch camera_semantic_perception three_view_visualizer.launch \
  secondary_mode:=road_marking

rqt_image_view /perception/camera/debug/mosaic/compressed
```

3-view mosaic는 세로 모니터에서도 각 예측을 충분한 높이로 볼 수 있도록 **3행×3열**로
구성한다. 열은 `left → front → right`, 행은 `원본 → drivable → secondary` 순서다.
4-class secondary 행은 white lane=흰색, yellow lane=노란색, stopline=빨간색이고
background=검은색이다. Binary secondary 행은 기존처럼 lane을 노란색으로 표시한다.
Overlay는 rqt용 panel/mosaic에서 제외했다. 기본
front-only panel은 640×1080, 3-view mosaic는 1920×1080이며 최대 10 Hz로 JPEG 발행하므로
원본 확률 토픽의 처리율을 불필요하게 따라가지 않는다.
카메라별 종횡비가 다르면 각 타일을 늘이지 않고 검은 여백으로 행 높이를 맞춘다.
화면이 열리지 않으면 [Docker GUI 설정](../../../docs/operations/README.md)을 먼저 확인한다.

날씨·시간 표기와 threshold는 노드를 재시작하지 않고 바꿀 수 있다.

```bash
rosparam set /camera_semantic_visualizer/condition_label "Foggy 1pm"
rosparam set /camera_semantic_visualizer/drivable_threshold 0.55
rosparam set /camera_semantic_visualizer/lane_threshold 0.45
```

설정을 기본값으로 고정할 때는 `config/visualization.yaml`을 수정한다. condition label은 화면
검토용 메타데이터일 뿐 추론 입력으로 사용되지 않는다.

## 2026-09-05 1차 live 실측

환경은 26.R1.H3, RTX GPU, 입력 1280×720, TwinLiteNet+ Medium epoch 18 EMA이다.

| 모드 | 결과 |
|---|---|
| front-only | lane/drivable 모두 약 20 Hz, GPU infer 약 16~19 ms/batch |
| three-view | view별 약 18.5~19 Hz, 총 약 54~58 view-frame/s |
| three-view GPU infer | 약 23~25 ms/batch |
| 출력 | 입력 stamp/frame ID를 보존한 원본 크기 1280×720 `mono8` |

이는 파이프라인 연결과 처리 가능성을 보는 1차 평균 로그다. 워밍업 구간을 제외한 p50/p95,
frame age, VRAM과 다른 노드 동시 실행 시 지연은 Stage C3에서 다시 측정한다.

## 입력과 출력

| View | 입력 |
|---|---|
| front | `/image_jpeg/compressed` |
| left | `/cam_left/image_jpeg/compressed` |
| right | `/cam_right/image_jpeg/compressed` |

각 선택 view에 다음 `sensor_msgs/Image`, `mono8` 토픽을 발행하며 입력 메시지의 stamp와
frame ID를 보존한다. Drivable과 binary lane의 값 0~255는 confidence 0~1에 대응한다.
Road-marking은 argmax class ID `0=background, 1=white, 2=yellow, 3=stopline`이다. 내부
backend 계약은 4-class softmax를 유지하고, 외부에는 class ID와 해당 pixel의 최대 softmax
confidence만 발행한다. 전체 4-channel tensor를 보내지 않아 3-view ROS 대역폭 증가를
제한한다.

```text
/perception/camera/{view}/drivable/probability
/perception/camera/{view}/lane/probability              # binary checkpoint
/perception/camera/{view}/road_marking/class_id         # 4-class checkpoint
/perception/camera/{view}/road_marking/confidence       # 4-class max softmax, 0~255
```

Metric lane-vector 개발 launch에서만 다음 model-grid 출력을 켠다. 둘 다 `384×640`
`mono8`이고 기존 출력과 같은 source stamp를 보존한다. 3-view batch 기준 최대 10 Hz이며,
구독자가 없으면 추론 노드는 model-grid 배열 복사와 publish를 생략한다.

```text
/perception/camera/{view}/road_marking/model_grid/class_id
/perception/camera/{view}/road_marking/model_grid/confidence
```

전체 per-class 확률 tensor는 발행하지 않는다. native GT loss-budget에서 단순 softmax 평균이
얇은 foreground를 희석해 hard direct 기준보다 나빴기 때문이다. 필요하면 후속 A/B에서만
내부 표현으로 다시 검토한다.

성능 계측은 `diagnostic_msgs/DiagnosticArray`로 함께 발행한다.

```text
/perception/camera/diagnostics
```

여기에는 backend/config/checkpoint, view·입력 크기, view별 input/output/replaced frame,
decode/inference 오류, preprocess/inference/postprocess/publish 평균 시간, processing FPS,
queue/e2e p50·p95, source/receive/result stamp와 upstream timestamp fallback이 들어간다.
같은 raw bag으로 모델을 비교할 때 `camera_semantic_result` profile로 별도 result bag에
기록한다. 절차는 [rosbag 평가 가이드](../../../docs/operations/rosbag-evaluation.md)를 따른다.

시각화 노드는 다음 `sensor_msgs/CompressedImage` 디버그 토픽을 발행한다.

```text
/perception/camera/{view}/debug/panel/compressed
/perception/camera/debug/mosaic/compressed
```

publisher는 항상 advertise하지만 실제 panel/mosaic 구독자가 없으면 렌더링과 JPEG 인코딩을
건너뛴다. BEV와 lane-vector 개발 화면까지 한 번에 띄울 때는
[`drivable_bev` 개발 launch](../drivable_bev/README.md#실행)를 사용하고 `rqt_image_view`의
dropdown에서 필요한 토픽만 선택한다.

## 검증

```bash
PYTHONPATH=src/perception/camera_semantic_perception/src \
python3 -m unittest discover \
  -s src/perception/camera_semantic_perception/test -v

rostopic hz /perception/camera/front/drivable/probability
rostopic hz /perception/camera/front/lane/probability
rostopic echo /perception/camera/front/lane/probability -n1
rostopic hz /perception/camera/front/road_marking/class_id
rostopic echo /perception/camera/front/road_marking/class_id -n1
rostopic hz /perception/camera/front/road_marking/confidence
rostopic hz /perception/camera/front/debug/panel/compressed
rostopic echo /perception/camera/diagnostics -n1
```

2026-09-05 실제 MORAI 입력으로 구 2×2 panel/가로 mosaic의 연결을 확인했다. 이후 rqt
레이아웃은 front-only 640×1080, three-view 3×3 mosaic 1920×1080으로 변경했으며 unit
test를 통과했다. 실제 MORAI 화면과 JPEG debug Hz는 다음 live 실행에서 다시 확인한다.
추론 처리율 평가는 probability 토픽과 Stage C3 diagnostics를 사용한다.

## 다음 단계

1. 동일 raw bag과 독립 result bag으로 모델/checkpoint별 FPS·latency·drop 비교
2. 조건별 live gate로 front-only/3-view와 현 센서 해상도·추론 Hz 결정
3. 확률 마스크를 camera calibration으로 공통 ego-BEV에 투영
