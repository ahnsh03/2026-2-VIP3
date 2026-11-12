# vip3_msgs

파트 간에 공유하는 ROS 메시지·서비스다. 외부 `morai_msgs`는 `src/morai_msgs`의 26.R1 커밋 `4c9be6f`를 사용하며 이 패키지에서 재정의하지 않는다.

| 타입 | 현재 생산자·소비자 | 의미 |
|---|---|---|
| [VehicleState.msg](msg/VehicleState.msg) | `vip3_parking` → `/vehicle/state` | header, 출처·유효성, x/y/yaw/speed/acceleration/yaw_rate |
| [SetGear.srv](srv/SetGear.srv) | `vip3_parking`의 `/vip3_gear/set_gear` | 기어 요청과 성공 여부·응답 기어·메시지 |

현재 상태 노드는 `/Ego_topic`을 사용하므로 `SOURCE_EGO_STATUS=2`를 발행한다. `SOURCE_GPS_IMU=1`은 메시지 상수로 남아 있지만 **해당 상태 변환 노드는 제거됐으며 현재 구현된 보조 입력이 아니다.** `valid`로 유효성을 구분하고 상세 pose/twist는 표준 `nav_msgs/Odometry`인 `/vehicle/odom`에서 받는다. 기어 상태 `/vehicle/gear`는 `std_msgs/Int32`다.

SS-PSD 슬롯·주차 목표용 커스텀 메시지는 아직 없다. 생산자와 소비자가 계약을 정할 때 표준 타입으로 충분한지 먼저 확인한다. 기존 타입의 필드 변경은 ROS1 MD5를 바꾸므로 모든 관련 노드를 함께 갱신·재빌드하고 [작업 노트](../../docs/notes/README.md)에 인수인계를 남긴다.

단위·좌표는 [시스템 계약](../../docs/system/design.md), 빌드·조회·실행 명령은 [commands.md](../../docs/commands.md#packages)를 따른다.
