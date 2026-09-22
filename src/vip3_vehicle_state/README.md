# vip3_vehicle_state — 차량 상태와 기어

> **문서 역할:** 패키지 가이드
> **담당:** 안승현
> **메시지 정본:** [docs/msgs-26r1.md](../../docs/msgs-26r1.md)

MORAI 26.R1 `/Ego_topic` 을 팀 내부 표현으로 정규화하고, **기어와 제어 모드를 소유**한다.

```bash
roslaunch vip3_vehicle_state vehicle_state.launch
```

## 노드 두 개

### `vip3_ego_state` — 상태 정규화

| | |
|---|---|
| 입력 | `/Ego_topic` (`morai_msgs/EgoVehicleStatus`) |
| 출력 | `/vehicle/state` (`vip3_msgs/VehicleState`), `/vehicle/odom` (`nav_msgs/Odometry`) |
| 선택 | `map → base_link` TF (`~publish_map_tf`, **기본 false**) |

**TF 는 기본으로 끈다.** MORAI 가 `/tf` 로 `map→base_link` 를 50 Hz 로 이미 쏜다
(`VIP3_network_v1.json` 의 TF2Publisher). 둘 다 켜면 RViz 에서 떨린다. 노드가 켜질 때 경고한다.

주요 파라미터:

| 이름 | 기본 | 뜻 |
|---|---|---|
| `~velocity_is_body_frame` | `true` | MORAI velocity 가 차량 기준인지. **실측 확인 필요** — 직진 중 로그의 `vy_body` 가 0 에서 멀면 틀린 것이다 |
| `~signed_speed` | `true` | `speed` 에 전후 부호를 넣는다. 후진 주차에 필요하다 (ASMC 는 `hypot` 이라 항상 ≥ 0 이었다) |
| `~stamp_mode` | `source` | `source` = `header.stamp` 가 0 이면 버린다 · `receive` = 수신 시각으로 대체. **MORAI rosbridge 가 stamp 를 안 채우면 `receive` 로 바꾼다** |
| `~publish_map_tf` | `false` | 위 참고 |

거부 사유는 `rospy.logwarn` 에 찍힌다. 가장 흔한 것은 `position == (0,0,0)` — MORAI 가 ego
스폰 전이나 시나리오 리로드 중에 보내는 패킷이고, 이걸 TF 로 쏘면 차가 원점으로 순간이동한다.

### `vip3_gear` — 기어·제어모드의 유일한 소유자

**이 노드 없이는 자율주차가 불가능하다.** 이유 두 가지:

1. MORAI 는 기동 시 키보드 제어 모드라 `/ctrl_cmd` 를 **조용히 무시한다.** 이 노드가 기동 때
   `ctrl_mode = 3 (External)` 로 바꾼다.
2. 기어는 토픽이 아니라 서비스로만 바꾼다. 두 노드가 각자 호출하면 후진 중에 D 로 튄다.
   그래서 writer 를 하나로 못박았다.

| | |
|---|---|
| 서비스 호출 | `/Service_MoraiEventCmd` (`morai_msgs/MoraiEventCmdSrv`) |
| 제공 서비스 | `~set_gear` (`vip3_msgs/SetGear`) |
| 출력 | `/vehicle/gear` (`std_msgs/Int32`, latched) |

```bash
# 후진
rosservice call /vip3_gear/set_gear "{gear: 2}"
# 주차 완료
rosservice call /vip3_gear/set_gear "{gear: 1}"
```

기어 코드: `1` P · `2` R · `3` N · `4` D (`morai_msgs/EventInfo` 와 같다).

노드 없이 손으로 확인할 때:

```bash
rosservice call /Service_MoraiEventCmd "{request: {option: 1,  ctrl_mode: 3}}"   # External
rosservice call /Service_MoraiEventCmd "{request: {option: 16, gear: 2}}"        # R
rosservice call /Service_MoraiEventCmd "{request: {option: 0}}"                  # 현재 상태 조회
```

`option` 은 비트마스크다 — `0x0001` ctrl_mode, `0x0010`(=16) gear. `option: 0` 은 아무것도
바꾸지 않고 현재 상태만 읽는 폴링이라, 이 노드가 1초마다 그걸로 시뮬과 동기를 맞춘다.

## 단위 규약 (제일 많이 틀리는 곳)

```
EgoVehicleStatus.heading            [deg]     ->  yaw      [rad]
EgoVehicleStatus.angular_velocity   [deg/s]   ->  yaw_rate [rad/s]
EgoVehicleStatus.velocity           [m/s]     ->  그대로
CtrlCmd.front_steer                 [rad]     <-  명령 쪽은 라디안
CtrlCmd.velocity                    [km/h]    <-  명령 쪽은 km/h
```

변환은 `src/vip3_vehicle_state/ego_state.py` 한 곳에서만 한다.

## 설계 규칙

`src/vip3_vehicle_state/*.py` 는 **rospy 를 import 하지 않는다.** 순수 계산만 두고 ROS 배선은
`scripts/*.py` 가 한다. 그래야 시뮬레이터 없이 단위 테스트가 돌고, 실제로 이 패키지의 검증은
전부 그렇게 한다.

```bash
cd /root/ws
PYTHONPATH=src/vip3_vehicle_state/src \
  python3 -m unittest discover -s src/vip3_vehicle_state/test -p 'test_*.py'
```

15개 통과 (2026-09-23).

## 확인해야 할 것

- [ ] `~velocity_is_body_frame` 이 맞는가 (직진 중 `vy_body` 로그)
- [ ] `header.stamp` 가 0 인가 → `stamp_mode:=receive`
- [ ] `/Service_MoraiEventCmd` 로 **주행 중** 기어 변경이 되는가, 정차 후에만 되는가
- [ ] `base_link` 원점이 정말 뒷바퀴축 중심인가 (`/Service_MoraiMapSpec` 또는 LiDAR 지면)
- [ ] 후진 시 `front_steer` 양수가 어느 쪽으로 도는가 (주차 제어기 부호 규약)

ASMC 의 GPS/IMU 추정 경로(`gps_transform.py`)는 라이브러리로만 가져왔고 노드는 만들지 않았다.
MORAI 가 `/Ego_topic` 으로 GT pose 를 주는 상황에서는 필요 없다. `/Ego_topic` 이 못 쓰게 되면
그때 노드를 붙인다.
