# morai_msgs 26.R1 — VIP3 기준

> **문서 역할:** 정본 — 우리가 쓰는 MORAI 메시지 버전과 필드
> **담당:** 안승현
> **기계 정본:** `src/morai_msgs/` (submodule, `26.R1` @ `4c9be6f`), `config/vip3_topics.yaml`
> **최종 수정:** 2026-09-23

## 1. 왜 이 문서가 필요한가

MORAI `morai_msgs`는 브랜치마다 **같은 이름의 메시지가 다른 필드**를 가진다. 특히 제어와 차량
상태가 다르다. 잘못된 브랜치로 빌드하면 컴파일은 되는데 런타임에 조용히 틀린 값이 들어간다.

**VIP3는 `26.R1` 브랜치 하나만 쓴다.** 대회용 `beta_drive`는 쓰지 않는다.

```bash
git submodule update --init --recursive
git -C src/morai_msgs rev-parse HEAD     # 4c9be6fcd90c4e6071001f4e196eea8443534c46
```

빌드 뒤 확인:

```bash
rosmsg show morai_msgs/CtrlCmd | grep steer            # front_steer / rear_steer 가 보여야 한다
rosmsg show morai_msgs/EgoVehicleStatus | grep -c angular_velocity   # 1 이어야 한다
```

`wheel_angle`이 보이면 `beta_drive`를 빌드한 것이다. 되돌린다.

## 2. 제어 — `morai_msgs/CtrlCmd` → `/ctrl_cmd`

```
int32   longlCmdType   # 1 Throttle · 2 Velocity · 3 Acceleration
float64 accel          # 0~1
float64 brake          # 0~1
float64 front_steer    # [rad]
float64 rear_steer     # [rad]
float64 velocity       # [km/h]  longlCmdType == 2 일 때만
float64 acceleration   # [m/s^2] longlCmdType == 3 일 때만
```

- **`steering` 필드는 없다.** ASMC(`beta_drive`) 코드를 가져올 때 가장 먼저 걸리는 지점이다.
- `rear_steer`는 쓰지 않더라도 **명시적으로 `0.0`을 채운다.** 안 채우면 값이 남는다.
- 주차는 저속이므로 `longlCmdType = 2`(Velocity)가 다루기 쉽다. 후진은 기어로 바꾸고 `velocity`는
  양수로 준다 — 음수 속도가 아니라 **기어가 방향을 정한다.**

## 3. 차량 상태 — `morai_msgs/EgoVehicleStatus` → `/Ego_topic`

주요 필드만:

| 필드 | 타입 | 단위 | 비고 |
|---|---|---|---|
| `position` | Vector3 | m | ENU. MGeo local frame과 원점이 같은지 **첫 연동에서 확인** |
| `velocity` | Vector3 | m/s | body frame 여부 미확인 |
| `angular_velocity` | Vector3 | **deg/s** | 26.R1 에 추가된 필드 |
| `heading` | float64 | **deg** | |
| `front_steer_angle` / `rear_steer_angle` | float32 | **deg** | |
| `accel` / `brake` | float32 | 0~1 | |
| `distance_left_lane_boundary` | float32 | m | 시뮬이 주는 공짜 GT |
| `distance_right_lane_boundary` | float32 | m | 〃 |
| `cross_track_error` | float32 | m | 차선 중심 기준, 우측 양수 |

- **`wheel_angle`은 없다.**
- 마지막 세 필드는 **주차장(차선 없는 구역)에서 0으로 채워질 수 있다.** 첫 연동에서
  `rostopic echo -n1 /Ego_topic` 으로 실제 값이 들어오는지 확인하고 여기에 결과를 적는다.
  살아 있으면 차선 피팅 검증과 인지 모델 평가에 그대로 쓸 수 있다.

## 4. 단위 함정 (제일 많이 틀리는 곳)

| 어디 | 무엇 | 단위 |
|---|---|---|
| `CtrlCmd.front_steer` | 명령 조향각 | **rad** |
| `EgoVehicleStatus.front_steer_angle` | 실측 조향각 | **deg** |
| `EgoVehicleStatus.heading` | 방위 | **deg** |
| `EgoVehicleStatus.angular_velocity` | 각속도 | **deg/s** |
| `EgoVehicleStatus.velocity` | 속도 | **m/s** |
| `CtrlCmd.velocity` | 목표 속도 | **km/h** |

**명령은 rad·km/h, 상태는 deg·m/s다.** 피드백 제어를 짤 때 한쪽만 변환하면 40배쯤 틀린 채로
그럴듯하게 동작한다. 변환은 코드 경계 한 곳에서만 하고 그 지점에 주석을 남긴다.

## 5. 기어 — `/Service_MoraiEventCmd` (`morai_msgs/MoraiEventCmdSrv`)

**후진 기어 없이는 주차가 불가능하다.** `EventInfo`:

```
int8  option      # 비트마스크: 0x0001 ctrl_mode · 0x0010 gear · 0x0100 lamps · 0x1000 set_pause
int32 ctrl_mode   # 1 Keyboard · 2 Gamepad · 3 External · 6 Auto agent
int32 gear        # -1 변경없음 · 1 Parking · 2 Reverse · 3 Neutral · 4 Drive
Lamps lamps
bool  set_pause
```

수동 확인:

```bash
# 외부 제어 모드로 전환 (이걸 안 하면 MORAI 가 /ctrl_cmd 를 무시한다)
rosservice call /Service_MoraiEventCmd "{request: {option: 1, ctrl_mode: 3}}"
# 후진
rosservice call /Service_MoraiEventCmd "{request: {option: 16, gear: 2}}"
# 주차
rosservice call /Service_MoraiEventCmd "{request: {option: 16, gear: 1}}"
# 현재 상태 조회 (option 0 이면 아무것도 안 바꾸고 응답만 받는다)
rosservice call /Service_MoraiEventCmd "{request: {option: 0}}"
```

코드에서는 `vip3_vehicle_state`의 기어 노드가 **유일한 기어 writer**다. 두 노드가 동시에 기어를
바꾸면 후진 중에 D로 튄다.

## 6. 브랜치 비교 (참고)

| | `26.R1` (VIP3) | `beta_drive` (ASMC·대회) |
|---|---|---|
| msg 수 | 91 | 101 |
| `CtrlCmd` 조향 | `front_steer` / `rear_steer` [rad] | `steering` 단일 |
| `EgoVehicleStatus` 조향 | `front_steer_angle` / `rear_steer_angle` [deg] | `wheel_angle` |
| `angular_velocity` | 있음 | 없음 |
| 차선 CTE 필드 | 있음 | 없음 |
| 동일 | `ObjectStatusList`, `GPSMessage`, `CollisionData`, `SaveSensorData`, `EventInfo` | |

ASMC에서 코드를 더 가져올 때는 위 표의 좌측으로 바꿔서 가져온다.
자세한 이식 규칙은 [porting-from-asmc.md](porting-from-asmc.md).
