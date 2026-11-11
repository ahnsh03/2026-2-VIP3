# vip3_vehicle_state

연결·상태 배선 **김동현**, 제어 요청 정책 **강도균**, MORAI 설정 **안승현**.
[실행 명령](../../docs/commands.md#packages) · [좌표·단위 계약](../../docs/system/design.md)

| 노드 | 입력 | 출력 / 서비스 |
|---|---|---|
| `vip3_ego_state` | `/Ego_topic` | `/vehicle/state`, `/vehicle/odom`; 선택 map→base_link TF |
| `vip3_gear` | `/Service_MoraiEventCmd` | `/vehicle/gear`, `/vip3_gear/set_gear` |

`vehicle_state.launch`는 두 노드를 묶는다. **기본적으로 기어 노드가 시작 시 External 모드 전환을 요청**한다. 관측만 할 때 `start_gear_node:=false`, 기어 노드를 준비하되 모드 전환을 원하지 않으면 `set_external_on_start:=false`를 사용한다.

- `publish_map_tf=false`: 현재 MORAI TF와 중복하지 않는다.
- `stamp_mode=source`: 0 timestamp를 거부한다. `receive`는 수신 시각으로 대체하는 명시적 개발 옵션이다.
- `velocity_is_body_frame=true`: 움직임과 heading을 바꾸어 축 규약을 확인해야 한다.
- `signed_speed=true`: 후진 속도의 부호를 유지한다.
- 기어 코드 P=1, R=2, N=3, D=4. 서비스 응답과 실제 gear를 확인한다.

heading·각속도는 deg→rad, 상태 속도는 m/s로 변환한다. `CtrlCmd.front_steer`는 rad, 명령 velocity는 km/h다. 상태 노드는 제어 명령을 발행하지 않는다. 기어 노드는 기어·모드 요청을 한 곳으로 모으지만 전체 자율주차 상태기는 아직 없다.

순수 계산은 `src/vip3_vehicle_state/`, ROS 배선은 `scripts/`에 있다. GPS/IMU 변환 코어는 남아 있으나 별도 자기위치추정 노드로 연결되어 있지 않다.
