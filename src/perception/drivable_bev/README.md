# drivable_bev — 카메라 semantic → base_link BEV

> **문서 역할:** 패키지 가이드 — 실행법과 입출력
> **담당:** 안승현
> **기하 정본:** `config/cameras_vip3_v1.yaml` · `config/bev_grid.yaml`
> **센서 정본:** `../../../config/VIP3_sensor_set_v1_ros.json` ([docs/sensors.md](../../../docs/sensors.md))

4대 카메라(전·좌·우·**후**)의 semantic 확률을 지면 평면 호모그래피로 `base_link` 격자에
투영하고 한 장으로 융합한다. ASMC `drivable_bev` 에서 기하 코어만 가져왔다.

## 빠른 시작

```bash
# 컨테이너 안에서
cd /root/ws && source devel/setup.bash

# 0) 모델 없이 기하만 확인 (가장 먼저 할 것)
roslaunch drivable_bev rgb_bev.launch
rqt_image_view /perception/bev/debug/rgb/compressed

# 1) 전체 파이프라인 (추론 + 투영 + 융합 + 시각화)
roslaunch drivable_bev four_view_bev.launch

# 후방만 따로
roslaunch drivable_bev camera_bev.launch views:=rear
```

## ⚠ 캘리브레이션 게이트 — 여기서 멈춰서 눈으로 확인한다

`rgb_bev.launch` 의 출력에 1 m 격자가 겹쳐 그려진다. **주차선·차선이 격자와 나란하고,
뷰 경계에서 꺾이지 않아야 한다.** 꺾이면 아래 중 하나가 틀린 것이다.

| 증상 | 원인 후보 |
|---|---|
| 전체가 앞뒤로 늘어남 | `ground_plane.z_at_origin_m` (현재 −0.35, KATRI 실측 전) |
| 좌우 뷰가 서로 어긋남 | 카메라 yaw 부호 |
| 가까울수록 휘어짐 | MORAI pitch 부호 규약 (양수 = 렌즈 아래) |
| 후방만 비어 있음 | rear yaw 180 이 설정에서 빠짐 — `test_rear_camera.py` 가 잡는다 |

`camera_bev_visualizer.launch` 의 `/perception/bev/debug/{view}/camera_grid_overlay/compressed`
는 같은 격자를 **원본 카메라 영상 위에** 그린다. 격자가 아스팔트에 얹히지 않으면 그 뷰의
외부 파라미터가 틀렸다. **이 화면이 맞기 전의 semantic BEV 는 전부 무의미하다.**

## 두 가지 secondary head

VIP3 기본 체크포인트(v5)는 binary(drivable + lane)라 `road_marking` 토픽이 아예 없다.
ASMC 원본은 4-class 경로만 있어서 그대로 두면 노드가 영원히 아무것도 발행하지 않는다.

| `secondary_head` | 입력 (뷰별) | 출력 (뷰별) | 체크포인트 |
|---|---|---|---|
| `lane` (기본) | `drivable/probability` + `lane/probability` | `drivable_probability`, `lane_probability` | `weights/twinlite/v5_binary_lane` |
| `road_marking` | `drivable/probability` + `road_marking/class_id` + `.../confidence` | `drivable_probability`, `road_marking/class_id`, `.../confidence` | `weights/twinlite/v6_road_marking` |

```bash
roslaunch drivable_bev four_view_bev.launch \
  secondary_head:=road_marking \
  checkpoint:=/root/ws/weights/twinlite/v6_road_marking/best_mean.pt
```

## 토픽

**입력** — `camera_semantic_perception` 이 발행한다. `{view}` = `front|left|right|rear`

```
/perception/camera/{view}/drivable/probability      sensor_msgs/Image mono8
/perception/camera/{view}/lane/probability          (lane 모드)
/perception/camera/{view}/road_marking/class_id     (road_marking 모드)
/perception/camera/{view}/road_marking/confidence   (road_marking 모드)
```

**출력**

```
/perception/bev/debug/{view}/drivable_probability   뷰별 투영
/perception/bev/debug/{view}/lane_probability        (lane 모드)
/perception/bev/debug/{view}/coverage                latched, 정적 가시 마스크
/perception/bev/camera/drivable_probability          융합 결과
/perception/bev/camera/lane_probability              (lane 모드)
/perception/bev/camera/coverage
/perception/bev/camera/source_count                  셀당 기여 뷰 수
/perception/bev/debug/source_view                    셀 소유 뷰 id (1 front / 2 left / 3 right / 4 rear)
/perception/bev/debug/rgb/compressed                 원본 RGB 서라운드 BEV
/perception/bev/diagnostics                          diagnostic_msgs/DiagnosticArray
```

입력 메시지의 `header.stamp` 를 그대로 보존한다. 뷰 간 정확 시각 동기(TimeSynchronizer)의
전제이므로 바꾸지 않는다.

## 격자

```
frame_id base_link,  x[-10, 10] m,  y[-8, 8] m,  resolution 0.05 m  ->  400 x 320 px
```

x 전방 · y 좌 (REP-103), 영상은 위가 전방 · 왼쪽이 좌측.
ASMC 주행용은 `x[-5,20] y[-8,8] res 0.10` 이었다. 주차는 차 주변 360도 근거리가 전부라
범위를 대칭으로 줄이고 해상도를 두 배로 올렸다 — **15 cm 주차선이 0.10 m 격자에서는
셀 1개 이하라 보이지 않는다.** 대신 셀 수가 40k → 128k 로 3.2배다. 느리면 첫 레버는
`resolution_m: 0.08`, 두 번째는 뷰별 debug publisher 끄기.

## ASMC 대비 바뀐 것

1. **후방 뷰 추가.** ASMC 는 전방 3대 전용이었다. 기하 코드는 원래 N-뷰 일반이라
   수식은 그대로고, 설정·융합 prior·시각화 순서만 바뀌었다.
2. **융합 prior 를 등방으로.** ASMC 는 `forward_fade_start_m=15 / rear_fade_start_m=-4` 로
   "앞으로 달린다"를 가중치에 박아뒀고, 이 때문에 후방 뷰 평균 quality 가
   0.3854 → 0.2583 (×0.670) 으로 깎였다. 지금은 `base_link` 로부터의 거리만 본다.
3. **density 정규화를 뷰별로.** front/rear 는 focal 640 px, left/right 는 149 px 라 지면
   샘플링 밀도가 ~18배 차이난다. 전역 정규화를 하면 겹치는 셀의 95.8 %를 전방이 가져간다.
   주차선을 실제로 보는 것은 측면 카메라다.
4. **binary lane 경로 추가** (위 표).
5. **shadow_*/boundary_*/invalid_*/planning_usability 계열 전부 제외.** K-City GPS 음영구간
   연구용이고 `asmc_msgs` 에 묶여 있었다. 19,241줄 중 약 3,600줄만 가져왔다.
6. **차선 벡터화 노드 제외.** ASMC `lane_vectorizer_node.py` 는 x 가 단조증가하는 선만
   추적한다(`lane_tracking.py` 의 `if dx <= 1e-6: continue`). 주차칸의 **진입선은 차축에
   수직**이라 구조적으로 못 잡는다. 라이브러리(`lane_evidence` / `lane_tracking` /
   `spline_fitting`)는 남겨 뒀으니 주차용 검출기는 융합 BEV 에서
   `connectedComponents` → `minAreaRect` 로 새로 쓰는 편이 맞다 (약 150줄).

## 검증

```bash
cd /root/ws
PYTHONPATH="src/perception/drivable_bev/src:src/perception/camera_semantic_perception/src" \
  python3 -m unittest discover -s src/perception/drivable_bev/test -p 'test_*.py'
```

67개 통과 (2026-09-23 기준, ROS·GPU 불필요). 그중 `test_rear_camera.py` 10개가 후방
카메라 기하의 회귀를 지킨다 — 후방 커버리지가 0 이 되거나 근거리 사각이 사라지면
(= 센서셋이 바뀌면) 여기서 먼저 깨진다.

## 아직 안 정해진 것

- `ground_plane.z_at_origin_m` KATRI 실측 (현재 ASMC 값 −0.35 잠정)
- MORAI pitch 부호 규약 확정 (지면 격자 오버레이로)
- 차량 실제 제원 — 특히 **후륜축 → 후범퍼 오버행**. 후진 주차에서 실제로 부딪히는 지점이다
- 지면 평면 가정의 한계: 주차된 옆 차는 카메라 시선 방향으로 길게 번진다. LiDAR 점유로
  보정하는 것이 정석이고 `/velodyne_points` 는 이미 있다
