# vip3_msgs

VIP3 팀 내부 ROS 메시지·서비스. **최소 집합만 둔다.**

## 1. 규칙

- 표준 메시지로 표현되는 것은 커스텀 타입을 만들지 않는다.
  `/vehicle/odom` 은 `nav_msgs/Odometry`, `/vehicle/gear` 는 `std_msgs/Int32` 다.
- MORAI 메시지(`morai_msgs`)는 submodule(`src/morai_msgs`, 26.R1 @ 4c9be6f)이다.
  이 패키지로 복사하거나 재정의하지 않는다.
- ROS1 은 타입의 **MD5** 로 publisher/subscriber 호환을 판단한다. 이미 쓰이고 있는
  타입의 필드를 바꾸면(순서 변경·타입 변경·이름 변경 포함) 상대 노드가 조용히 연결되지
  않는다. 필드를 **추가**하는 것도 MD5 를 바꾼다.
- 그래서 정보가 더 필요하면 기존 타입을 고치지 말고 **새 타입을 추가**하고,
  바꿔야 한다면 그 타입을 쓰는 모든 노드를 같이 다시 빌드한다.

## 2. 현재 타입

| 타입 | 쓰는 곳 | 왜 커스텀인가 |
|---|---|---|
| `msg/VehicleState.msg` | `vip3_vehicle_state` → `/vehicle/state` | `valid` 플래그 + `source` 출처 + 2D 상태(x, y, yaw, speed, acceleration, yaw_rate)를 한 메시지로 묶은 것. `nav_msgs/Odometry` 에는 "이 값 믿어도 되나"를 나타내는 필드가 없어서 stale/무효 구분이 불가능하다. 자세한 pose/twist 가 필요하면 `/vehicle/odom` 을 쓴다. |
| `srv/SetGear.srv` | `vip3_vehicle_state/vip3_gear_node` | 기어 변경은 스택에서 가장 위험한 쓰기다. 동기 응답(성공/실패 + 실제 기어)이 필요한데 `std_srvs` 에 정수 요청 서비스가 없다. 토픽으로 하면 늦게 연결된 publisher 의 첫 메시지가 조용히 버려진다(후진 명령 유실 = 주차 실패). |

`VehicleState.source` 의 `SOURCE_EGO_STATUS=2` 가 **VIP3 의 기본값**이다
(ASMC 에서는 개발 전용이었다). `SOURCE_GPS_IMU=1` 은 보조 경로다.

## 3. 안 가져온 것

ASMC `asmc_msgs` 의 나머지 24개 타입(Trajectory / BehaviorCommand / FrenetFeedback /
DetectedObject / Lane* / DrivableArea* / CompetitionCheckpoint / MissionZone …)은
**주행 대회용**이라 가져오지 않았다. 필요해지면 그때 쓰는 쪽에서 추가한다.

`GearState.msg` 도 만들지 않았다. 기어는 정수 하나라 `std_msgs/Int32` 로 충분하다.

## 4. 빌드

```bash
catkin_make --pkg vip3_msgs      # 또는 워크스페이스 전체 빌드
rosmsg show vip3_msgs/VehicleState
rossrv show vip3_msgs/SetGear
```
