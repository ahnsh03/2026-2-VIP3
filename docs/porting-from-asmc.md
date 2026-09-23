# ASMC에서 가져온 것 / 안 가져온 것

> **문서 역할:** 가이드 — 이식 범위와 치환 규칙
> **담당:** 안승현 · 장원태
> **원본:** [INHAautonav/2026-ASMC](https://github.com/INHAautonav/2026-ASMC) — 로컬 참조 clone `../external/2026-ASMC/`
> **최종 수정:** 2026-09-23

ASMC 는 **자율주행 대회**(K-City) 레포다. VIP3 는 **자율주차** 과목 프로젝트다. 목표도
전송 방식도 메시지 버전도 다르므로 그대로 복사하면 안 된다. 이 문서가 그 차이를 정리한다.

## 1. 네 가지 근본 차이

| | ASMC | VIP3 |
|---|---|---|
| 목표 | 자율주행 (K-City) | **자율주차** (KATRI) |
| 전송 | **UDP** 브리지 (직접 구현) | **rosbridge** WebSocket 하나 |
| 메시지 | `morai_msgs` **`beta_drive`** | **`26.R1`** @ `4c9be6f` |
| 카메라 4번 | 하향 (0, 0, 1.40) pitch 328, **학습 제외** | **후방** (−0.1, 0, 1.20) yaw 180, **학습 포함** |

## 2. 치환 규칙

코드를 더 가져올 때 기계적으로 적용한다.

### 메시지

```
CtrlCmd.steering                      ->  CtrlCmd.front_steer [rad]  (+ rear_steer = 0.0 명시)
EgoVehicleStatus.wheel_angle          ->  EgoVehicleStatus.front_steer_angle [deg]
(없음)                                 ->  EgoVehicleStatus.angular_velocity [deg/s]
(없음)                                 ->  distance_left/right_lane_boundary, cross_track_error
asmc_msgs.*                           ->  vip3_msgs.* 또는 stock ROS 메시지
```

**단위가 비대칭이다.** 명령은 rad·km/h, 상태는 deg·m/s. 자세한 것은
[msgs-26r1.md](msgs-26r1.md) §4.

### 이름

```
ASMC_DATA            -> VIP3_DATA          asmc_env.sh        -> vip3_env.sh
ASMC_WS_ROOT         -> VIP3_WS_ROOT       asmc-ros-gui       -> vip3-ros-gui
$ASMC                -> $VIP3              asmc_hd_map        -> vip3_hd_map
asmc-ros-noetic      -> vip3-ros-noetic    asmc_msgs          -> vip3_msgs
asmc/ros-noetic:dev  -> vip3/ros-noetic:dev
```

**바꾸지 않은 것** (이미 프로젝트 중립이고 바꾸면 연쇄 수정만 생긴다):
`camera_semantic_perception`, `twinlite_morai`, `drivable_bev`, `data_collection`,
노드 이름들, `/perception/...` 토픽 네임스페이스.

### 토픽

```
/image_jpeg/compressed            ->  /cam_front/image_jpeg/compressed
/cam_extra/image_jpeg/compressed  ->  /cam_rear/image_jpeg/compressed   (의미가 다르다!)
```

전방에 접두어를 붙여 4방향을 대칭으로 만들었다. MORAI 기본값(`/image_jpeg`)과 다르므로
센서셋에서 명시한다. 정본은 [config/vip3_topics.yaml](../config/vip3_topics.yaml).

### UDP 흔적

`udp_bridge.sh`, `probe_host_udp.py`, 포트 표(9290~9311), `morai_udp_bridge`,
`morai_sensor_bridge`, `/morai_sensor_bridge/diagnostics` 구독, `transport: udp` 프로파일,
`ASMC_SIMULATOR_PROFILE` — **전부 삭제한다.** bag 프로파일 로더가 `transport: udp` 를
보면 일부러 에러를 낸다. 복붙한 설정이 조용히 도는 것보다 낫다.

## 3. 가져온 것

| VIP3 | 원본 | 상태 |
|---|---|---|
| `src/perception/drivable_bev/` | `src/perception/drivable_bev/` | **부분** — 기하 코어만 (19,241 → 약 3,600줄) |
| `src/perception/camera_semantic_perception/` | 동일 | 거의 그대로 + 4뷰 |
| `src/perception/twinlite_morai/` | 동일 | 거의 그대로 (UDP·msgs 결합 0건이었다) |
| `src/data_collection/` | 동일 | **부분** — Capture + bag + 데이터셋 체인 |
| `src/vip3_hd_map/` | `src/asmc_hd_map/` | **재작성** — 좌표계·크롭 코어만 (19,241 → 약 1,200줄) |
| `src/vip3_vehicle_state/` | `src/vehicle_state/` | **재작성** — C++ → rospy, 기어 노드 신규 |
| `src/vip3_bringup/` | `src/integration_launch/` | 재작성 |
| `docker/` `scripts/` | 동일 | 부분 |
| `weights/twinlite/` | `artifacts/perception_eval/runs/` | v5·v6 체크포인트 2개 |

### 기능이 바뀐 곳 (그대로 쓰면 안 된다)

1. **`drivable_bev` 에 binary lane 경로 추가.** ASMC 는 4-class `road_marking` 경로만
   있었는데 VIP3 기본 체크포인트는 binary(drivable + lane) 라 그대로 두면 노드가 영원히
   아무것도 발행하지 않는다. `secondary_head: lane|road_marking` 으로 갈린다.
2. **융합 가중치를 등방으로.** ASMC 는 `forward_fade_start_m=15 / rear_fade_start_m=−4` 로
   "앞으로 달린다"를 박아뒀고, 이 때문에 후방 뷰 평균 quality 가 0.3854 → 0.2583 (×0.670)
   으로 깎였다. 주차에서는 앞뒤옆이 대등해야 한다.
3. **density 정규화를 뷰별로.** front/rear 는 focal 640 px, left/right 는 149 px 라 지면
   샘플링 밀도가 ~18배 차이난다. 전역 정규화면 겹치는 셀의 95.8 %를 전방이 가져간다.
4. **BEV 격자를 주차용으로.** `x[−5,20] y[−8,8] res 0.10` → `x[−10,10] y[−8,8] res 0.05`.
   **15 cm 주차선은 0.10 m 격자에서 셀 1개 이하라 안 보인다.**
5. **cam4 가 학습 뷰가 됐다.** ASMC 는 4번을 lane/drivable 학습에서 제외했다.
   front-only 였던 `stopline`/`road_marking` 가정도 all-view 로 바꿨다.
6. **대회 게이트 → GT 분리 게이트.** `privileged_gt`/`deployment_allowed` 는 대회 규정
   장치였다. 대신 `simulator_gt` 를 뒀다 — 시뮬만 아는 정보(`/Object_topic`,
   `/CollisionData`, `/sem_*`)를 담으려면 프로파일이 명시해야 한다. 섞이면 인지 평가가
   무의미해진다.

### 고친 버그 4건

- `smoke_twinlite_data_loss.py` — `valid_masks` dict 를 device 로 안 옮겨 `--device cuda` 에서
  device mismatch
- `evaluate_twinlite.py` — binary 하드코딩 때문에 **v6 4-class 체크포인트를 평가할 수 없었다**
- **파이프라인 단계별로 뷰가 어긋나 후방 카메라 데이터가 조용히 버려졌다.** 수집은 4뷰,
  curation/bake/dataset 은 각자 `("front","left","right")` 를 들고 있었다. 이제 뷰 이름의
  정본은 `capture_sync.CAMERAS` 하나고 `test_view_contract.py` 가 지킨다
- **`katri_map_viz_node` 의 `pose_source:=gps_imu` 가 아무것도 발행하지 않았다.** 파라미터
  검증도 하고 `/gps`·`/imu` 구독도 하는데 콜백이 최신 메시지를 저장만 하고 `_update()` 를
  부르지 않았다. 로그는 정상이고 마커만 안 뜨는, 눈으로는 원인을 못 찾는 형태였다.
  이식된 `live_pose.py` + `gps_transform.py` 를 실제로 연결해 고쳤다

## 4. 안 가져온 것

| 대상 | 왜 |
|---|---|
| `morai_udp_bridge` · `morai_sensor_bridge` | ROS 단일 전송 |
| `frenet_planner` · `mpc_controller` · `behavior_planner` | 고속 주행용. [roadmap.md](roadmap.md) §6 |
| `traffic_light` · `lidar+sensorfusion` | 주차 scope 밖 |
| `learning_by_cheating` · `morai_3d_detection` 학습부 | scope 밖 |
| `drivable_bev` 의 `shadow_*`/`boundary_*`/`invalid_*`/`planning_usability` | K-City GPS 음영구간 연구. `asmc_msgs` 결합 |
| ASMC 차선 벡터화 노드 | **x 단조증가 선만 추적한다** (`lane_tracking.py` 의 `if dx <= 1e-6: continue`). 주차칸 진입선은 차축에 수직이라 구조적으로 못 잡는다. 라이브러리(`lane_evidence`/`lane_tracking`/`spline_fitting`)는 남겼다 |
| `asmc_hd_map` 의 K-City 2025 큐레이션 | 대회 전용 (원본의 약 95 %) |
| `tools/perception_eval` | **스텁이다.** README 가 직접 "인터페이스·exit code 만 구현"이라고 적었고 `run_all.sh` 는 `echo stub done` 으로 끝난다 |
| gRPC (`tools/grpc_inha_univ`, `environment_grpc.py`) | Capture 는 ROS `/SaveSensorData` 로 충분하다 |
| 문서 `structure/` D1~D5 이전 보고서, `governance/`, `competition/` | 문서 12개인 팀에 순수 비용 |
| ultralytics/YOLO, open3d, wandb, grpcio | 위 항목들과 함께 빠졌다 |

### 가져왔다가 도로 지운 것

이식 직후 감사에서 **생산 호출자가 0 인 모듈**을 찾아 지웠다. 되살릴 때는 이 커밋의
부모에서 꺼내면 된다 — 파일과 테스트가 같이 들어 있다.

| 모듈 | 왜 지웠나 |
|---|---|
| `vip3_hd_map/spatial.py` | 호출자 0, 테스트 0 |
| `vip3_hd_map/local_crop.py` | 호출자 0. `nav_msgs/OccupancyGrid` 를 자르는데 저장소에 OccupancyGrid 를 내는 노드가 없다. 기본값도 K-City 주행 규모(120 m 사각, 0.2 m/셀)라 주차 BEV 격자(20×16 m, 0.05 m/셀)와 두 자릿수 차이다 |

반대로 **살려서 배선한 것**도 있다. `live_pose.py` + `gps_transform.py` 는 호출자가
0 이었는데, 지울 게 아니라 `katri_map_viz_node` 의 `pose_source:=gps_imu` 경로가
**절반만 이식돼 있던** 것이었다 (§3 '기능이 바뀐 곳' 참조).

### gRPC 를 버려서 잃은 것

1. **저장 완료 응답이 없다.** ROS publish 는 리턴하면 무조건 `success: true` 다.
   → `sync_capture_data.py --dry-run` 이 **선택이 아니라 필수 게이트**가 됐다
2. `morai_sim_time_*` 이 항상 `null` (스키마는 유지)
3. `rpc_latency_ms` 가 무의미 (로컬 publish 비용만 잰다)
4. bag 의 날씨·시각 자동 기록 → 런치 인자로 대체

## 5. 다시 가져올 때

1. `../external/2026-ASMC/` 를 `git pull --ff-only origin main` 으로 갱신
2. 파일을 복사한 뒤 **§2 치환 규칙을 전부 적용**
3. UDP·`asmc_msgs`·대회 전용 분기를 지운다
4. 3-뷰 가정(`("front","left","right")`)을 4-뷰로
5. **단위 테스트를 같이 가져오고 통과시킨다.** ASMC 코드는 테스트가 잘 돼 있어서
   이게 이식 검증의 주된 수단이다
6. 커밋 메시지에 원본 경로를 남긴다

```bash
# 전 패키지 테스트 (ROS·GPU 없이 호스트에서)
cd /root/ws
PKGS="src/perception/drivable_bev src/perception/camera_semantic_perception \
      src/data_collection src/vip3_hd_map src/vip3_vehicle_state"
# 패키지 간 import 가 있어서(vip3_hd_map -> vip3_vehicle_state) 전부 경로에 올린다.
ALL=$(for p in $PKGS; do printf "%s/src:" "$p"; done | sed 's/:$//')
for p in $PKGS; do
  echo "== $p"
  PYTHONPATH="$ALL" python3 -m unittest discover -s "$p/test" -p 'test_*.py' 2>&1 | tail -3
done
```

2026-09-23 기준 **193개 통과.**
