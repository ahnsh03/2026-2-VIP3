# 시스템과 인터페이스

## 현재 연결

```text
MORAI 26.R1.H3 (K-City 2025, Ioniq5)
  └─ ROS WebSocket → rosbridge (기본 9090)
       ├─ 4 × CompressedImage → fisheye_svm → 진단 panel/preview/valid_mask
       ├─ EgoVehicleStatus → vip3_vehicle_state → state/odom
       ├─ 카메라·ego·TF → data_collection → bag / 재생
       └─ event service ↔ vip3_gear

연결할 경로: clean BEV → SS-PSD → 주차 목표 → 경로·추종 → /ctrl_cmd
개발·평가 경로: 제공 주차 GT → 별도 목표 어댑터 → 같은 경로·추종
```

아래 표의 **예정 토픽은 아직 발행되지 않는다**. 실행 명령을 만들어진 것처럼 기재하지 않는다.

| 구현 상태 | 입출력 | 책임 |
|---|---|---|
| 구현 | `/cam_{front,rear,left,right}/image_jpeg/compressed`, `sensor_msgs/CompressedImage` | MORAI→SVM·수집 |
| 구현 | `/perception/svm/preview/compressed`, `panel`, `panel/compressed`, `valid_mask` | SVM 진단 |
| 구현 | `/Ego_topic` → `/vehicle/state` (`vip3_msgs/VehicleState`), `/vehicle/odom` | 상태 변환 |
| 구현 | `/vip3_gear/set_gear` (`vip3_msgs/SetGear`), `/vehicle/gear` | 기어 소유자 |
| 구현 | `/vip3_hd_map/global/*`, `/local/*`, `/debug/pose` | MGeo 시각화; 좌표 정합 보증 아님 |
| 예정 | `/parking/bev/image`, BGR `Image` 512² + valid mask·metric 설정 | clean BEV·시각 보존 |
| 예정 | 검출 슬롯: 입구 두 점, 안쪽 방향, confidence, stamp, `base_link` | 검출 메시지 타입은 생산자·소비자 합의 후 확정 |
| 예정 | `/parking/goal_gt`, `/parking/goal_perception` → `/parking/goal`, `PoseStamped` | 출처를 명시적으로 선택; 후륜축 목표, `map` |
| 예정 | `/parking/path`, `nav_msgs/Path` → 단일 제어기 → `/ctrl_cmd` | 전방 MVP. 속도·기어 정보는 Path와 별도 계약 필요 |

## 좌표와 단위

`base_link`는 **후륜축 기준**, x 전방·y 좌측·z 위쪽이다. 차량 중심은 여기서 x=1.5275m 앞이다. 목표 칸 중심을 그대로 후륜축 목표로 주면 차가 앞으로 밀린다. 목표 heading을 θ라 할 때 후륜축 목표는 `칸의 차량 중심 목표 − 1.5275 × (cosθ, sinθ)`로 계산한다.

| 항목 | 외부 단위 | 내부 처리 |
|---|---|---|
| Ego heading / angular_velocity | deg / deg/s | state의 yaw / yaw_rate는 rad / rad/s |
| Ego velocity | m/s | state는 m/s; body/world 성격은 움직이며 확인 |
| Ego front_steer_angle | deg | 조향 측정과 명령을 비교할 때 변환 |
| CtrlCmd front_steer | rad | 제어기에서 rad로 발행 |
| CtrlCmd velocity | km/h | 내부 m/s 목표를 명령 직전에 변환 |

메시지는 `morai_msgs` 26.R1, 커밋 `4c9be6f`를 사용한다. `vip3_msgs`는 상태와 기어 서비스만 제공한다. 명령 필드는 `rosmsg show morai_msgs/CtrlCmd`로 확인한다.

MORAI가 `map→base_link` TF를 보내므로 상태 노드의 `publish_map_tf` 기본값은 false다. 카메라 TF도 MORAI 출력과 수동 static TF를 중복 발행하지 않는다. SVM은 자체 설정으로 투영하므로 미리보기만 켤 때 bringup·static TF는 필요 없다.

## 시간과 제어 책임

- 현재 SVM은 버퍼에서 최대 시각차 60ms, 최대 수신 경과 0.5초 조건으로 네 영상을 고른다. header 시간이 없으면 수신 시각 기준으로 동작할 수 있다. 이것은 하드웨어 동기화를 보장하지 않는다.
- clean BEV·검출 결과에는 사용한 영상 시각과 출처를 남긴다. 움직이며 수집할 때 지연·시각차가 커지면 추론보다 먼저 입력을 점검한다.
- 상태 노드는 source stamp가 0이면 거부한다. `stamp_mode:=receive`는 수신 시각 사용을 명시한 개발용 대안이며 GT·영상 동기화를 해결해 주지 않는다.
- `/ctrl_cmd`는 제어기 한 개만 발행한다. 기어 요청은 `/vip3_gear/set_gear`로 모으고 실제 응답을 확인한다.
- 기어 노드는 기본적으로 시작 시 External 모드 전환을 요청한다. 관측만 할 때는 기어 노드를 끄거나 `set_external_on_start:=false`를 쓴다.
- 입력 만료·검출 실패·서비스 실패 시 정지하는 로직은 주차 제어기에 구현할 항목이다. 현재 bridge·SVM만으로 자율주차가 완성된 상태는 아니다.
