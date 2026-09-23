# MORAI 연동 — rosbridge 단일 경로

> **문서 역할:** 정본 — 시뮬레이터 연결 절차·토픽·진단
> **담당:** 안승현 · 장원태
> **기계 정본:** `config/vip3_topics.yaml`, `config/VIP3_network_v1.json`, `config/VIP3_sensor_set_v1_ros.json`
> **최종 수정:** 2026-09-23

## 1. 전송 방식 — ROS 하나만 쓴다

```
MORAI SIM (Windows)  ──WebSocket──▶  rosbridge_server (컨테이너)  ──▶  우리 ROS 노드
                     ◀──────────────  /ctrl_cmd
```

**UDP 는 쓰지 않는다.**

### UDP 와 무엇이 다른가

어느 쪽이든 **팀원 코드에 도착하는 것은 ROS 토픽**이다 — ASMC 도 UDP 로 받아 ROS 토픽으로
다시 내보냈다. 그래서 인지·계획 노드를 쓸 때는 차이가 없다. 차이는 가운데 구간에 있다.

```
UDP (ASMC)   MORAI ─ 전용 바이너리, 센서마다 포트 ─▶ [우리가 짠 파서 1,443줄] ─▶ ROS 토픽
ROS (VIP3)   MORAI ─ ROS 메시지(JSON), 포트 하나 ──▶ [rosbridge 표준 패키지]  ─▶ ROS 토픽
```

| | UDP | ROS (rosbridge) |
|---|---|---|
| 메시지 형식 변환 | **우리 코드**가 바이트 배치를 안다. MORAI 버전이 바뀌면 파서도 바뀐다 (ASMC 제어 패킷은 55/59 바이트 두 형식을 다 처리했다) | **MORAI 가** morai_msgs 형식으로 보낸다. 맞출 것은 morai_msgs 버전 하나 |
| 포트 | 센서·기능마다 하나 — ASMC 는 15개 안팎 | 9090 하나 |
| Docker | UDP 포워딩이 불안정해 ASMC 는 host 네트워크 모드를 강제했다 | 9090/tcp 하나만 열면 된다 |
| 안 될 때 | 받는 쪽이 없어도 **조용히 안 온다.** 원인 후보가 설정·포워딩·파서·형식 변경으로 넓다 | 연결 실패가 드러나고, 후보가 연결·토픽 이름·msgs 버전으로 좁다 |
| 센서를 바꿀 때 | MORAI 포트 + 파서 설정 + 컨테이너 포트 | MORAI 에 토픽 이름 하나 |
| 제어 명령 | 손실 가능 — ASMC 는 같은 명령을 반복 전송했다 | TCP 라 한 번 보내면 도착한다 |
| 처리량 | 빠르다 | JSON + base64(+33 %)를 파이썬이 푼다. **카메라 4대에서 병목 가능** |

**MORAI 경험이 적은 팀원이 가장 막히는 곳이 설정과 고장 대응이라 ROS 로 간다.**
대가는 처리량 하나다 (§5). 다만 학습 데이터는 Capture Mode 가 Windows 디스크에 PNG 로
직접 쓰므로 네트워크를 타지 않고, 주차는 저속이라 10 Hz 면 충분하다.

> **되돌릴 조건:** 1블록 실측에서 **10 Hz × 카메라 4대도 안 나오면** 카메라만 UDP 로 받는
> 혼합 구성을 검토한다. MORAI 는 센서마다 통신 방식을 따로 정할 수 있고, ASMC 센서 파서는
> 토픽 이름만 바꿔 이식하면 된다. 그 전까지는 필요 없다.

rosbridge 는 **서버**, MORAI 가 **클라이언트**다. 따라서 **rosbridge 를 먼저 띄우고 MORAI 를
연결한다.** 순서를 바꾸면 연결이 안 된다.

## 2. 기동 순서

```bash
# 호스트 (WSL)
cd "$VIP3"
xhost +local:docker                 # RViz/rqt 쓸 때만
./scripts/docker_ros_up.sh up

# 컨테이너 터미널 A — 계속 띄워 둔다
docker exec -it vip3-ros-noetic bash
cd /root/ws && ./scripts/build_ws.sh && source devel/setup.bash
./scripts/rosbridge.sh
```

MORAI (Windows):

1. Map — **`R_KR_PG_KATRI`**
2. Network Settings — `config/VIP3_network_v1.json` Import → **Connect**
3. Sensor Settings — `config/VIP3_sensor_set_v1_ros.json` Import → **Connect**
4. **Ego Ctrl Cmd 를 ROS `/ctrl_cmd` 로 설정** (§4 — 아직 안 돼 있다)
5. Play

```bash
# 컨테이너 터미널 B
cd /root/ws && source devel/setup.bash
./scripts/wait_morai_topic.sh /Ego_topic 120
python3 scripts/verify_morai_topics.py --duration 30
roslaunch vip3_bringup vip3_bringup.launch
```

## 3. 토픽

정본은 `config/vip3_topics.yaml` 이다. 아래는 사람이 읽는 사본이므로 값이 다르면 YAML 이 이긴다.

**센서** (`VIP3_sensor_set_v1_ros.json`)

| 토픽 | 타입 | frame | 설정 Hz |
|---|---|---|---:|
| `/cam_front/image_jpeg/compressed` | `sensor_msgs/CompressedImage` | `cam_front` | 20 |
| `/cam_left/image_jpeg/compressed` | `sensor_msgs/CompressedImage` | `cam_left` | 20 |
| `/cam_right/image_jpeg/compressed` | `sensor_msgs/CompressedImage` | `cam_right` | 20 |
| `/cam_rear/image_jpeg/compressed` | `sensor_msgs/CompressedImage` | `cam_rear` | 20 |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | `velodyne` | 10 |
| `/gps` | `morai_msgs/GPSMessage` | `gps` | 5 |
| `/imu` | `sensor_msgs/Imu` | `imu` | 50 |

**네트워크** (`VIP3_network_v1.json`, 전부 ROS · 50 Hz)

| 방향 | 토픽 | 타입 | 주차에 |
|---|---|---|---|
| SIM → 우리 | `/Ego_topic` | `morai_msgs/EgoVehicleStatus` | **필수** |
| SIM → 우리 | `/Object_topic` | `morai_msgs/ObjectStatusList` | 유용 (주차된 차가 NPC 면 GT) |
| SIM → 우리 | `/CollisionData` | `morai_msgs/CollisionData` | **유용** (충돌 판정) |
| SIM → 우리 | `/tf` | `tf2_msgs/TFMessage` | `map`→`base_link` 를 공짜로 준다 |
| 우리 → SIM | `/SaveSensorData` | `morai_msgs/SaveSensorData` | Capture Mode 수집 |
| 우리 → SIM | `/ScenarioLoad` | `morai_msgs/ScenarioLoad` | 시작 자세 재현 후보 |
| 서비스 | `/Service_MoraiEventCmd` | `MoraiEventCmdSrv` | **필수** (기어) |
| 서비스 | `/Service_MoraiMapSpec` | `MoraiMapSpecSrv` | 맵 원점 확인 |

신호등(`/GetTrafficLightStatus`, `/IntscnTL_topic`, `/InsnStatus`)과 SyncMode 계열도 켜져 있지만
주차에는 쓰지 않는다.

## 4. ⚠ `/ctrl_cmd` 가 아직 없다

`VIP3_network_v1.json` 을 전수 확인한 결과 **차량 제어 채널이 설정돼 있지 않다.**

- PUBSUB_TYPE `771` (`MoraiCmdController`) 행이 21개 항목 어디에도 없다
- `EgoCtrlConfig` 가 전 항목에서 `commType: 0`, `rosConfig.Topic: "/dafault_topic"`

**이 상태로는 차를 움직일 수 없고, 자율주차도 불가능하다.** 첫 연동에서 아래를 해야 한다.

1. MORAI Network Settings → Ego(UNIQUEID 0) → **Ego Ctrl Cmd** 행
2. 통신 방식 **ROS**, Topic **`/ctrl_cmd`**, Bridge `127.0.0.1:9090`, Connect
3. 프리셋을 다시 Export 해서 `config/VIP3_network_v1.json` 을 **갱신하고 커밋**
4. Export 한 JSON 에서 `EgoCtrlConfig.commType == 3`, `_PUBSUB_TYPE == 771`,
   `rosConfig.Topic == "/ctrl_cmd"` 인지 확인

그리고 **런타임에 제어 모드를 External 로 바꿔야 한다.** 안 하면 MORAI 가 키보드 제어를 유지하고
`/ctrl_cmd` 를 조용히 무시한다:

```bash
rosservice call /Service_MoraiEventCmd "{request: {option: 1, ctrl_mode: 3}}"
```

`vip3_vehicle_state` 의 기어 노드가 기동 시 이걸 자동으로 한다.

## 5. ⚠ rosbridge 대역폭 — 가장 큰 미지수

rosbridge 는 `CompressedImage` 를 **base64 JSON** 으로 감싸 단일 WebSocket 으로 나르는 파이썬
서버다. 지금 설정의 개략 계산:

```
1280×720 JPEG(q90) ≈ 150 KB → base64 200 KB, 20 Hz → 4 MB/s   (front, rear 각각)
 640×480 JPEG(q90) ≈  45 KB → base64  60 KB, 20 Hz → 1.2 MB/s (left, right 각각)
합계 ≈ 10 MB/s + PointCloud2 10 Hz + Ego 50 Hz
```

ASMC 가 UDP 를 만든 이유가 정확히 이것이다. **20 Hz × 4대가 안 나올 가능성이 높다.**
막히면 아래 순서로 완화한다 (위에서부터, 비용이 싼 순):

1. 컨테이너 이미지에 **`ujson`** 이 설치돼 있다 — `rosbridge_library` 가 자동으로 집어 쓴다.
   (이미 적용돼 있음)
2. 카메라 `sensorPeriod` 0.05 → **0.1 (10 Hz)**. 주차는 5 km/h 미만이라 10 Hz 로 충분하다.
3. front/rear 해상도 1280×720 → 640×480 으로 좌우와 통일.
   **BEV 내부 파라미터가 같이 바뀌므로 `config/vip3_topics.yaml` 과 BEV config 를 같이 고친다.**
4. 수집할 때만 4대를 켜고, 평소에는 필요한 카메라만 켠다.

측정:

```bash
python3 scripts/verify_morai_topics.py --duration 30
rostopic hz /cam_front/image_jpeg/compressed
```

**이건 1블록(~10/04) 안에 반드시 실측한다.** 여기서 막히면 데이터 수집 설계 전체가 바뀐다.

## 6. 진단

```bash
./scripts/diagnose_morai_rosbridge.sh
rostopic list | grep -E 'cam_|velodyne|gps|imu|Ego'
rostopic echo -n1 /Ego_topic | head -20     # position 이 (0,0,0) 이 아닌지, heading 단위
rostopic echo -n1 /cam_front/image_jpeg/compressed/header   # stamp 가 0 이 아닌지
rosservice list | grep Morai
```

### 자주 나오는 증상

| 증상 | 원인 | 확인 |
|---|---|---|
| MORAI 가 Connect 안 됨 | rosbridge 가 안 떠 있음 | 터미널 A 에서 `./scripts/rosbridge.sh` 가 살아 있는지 |
| `rostopic list` 에 카메라 없음 | 센서셋이 아직 UDP(`commType 1`) | `config/VIP3_sensor_set_v1_ros.json` Import 했는지 |
| 카메라는 오는데 `/Ego_topic` 없음 | Network 프리셋 미적용 | `VIP3_network_v1.json` Import + Connect |
| `/ctrl_cmd` 를 보내는데 차가 안 움직임 | 제어 모드가 Keyboard | §4 의 `ctrl_mode: 3` 호출 |
| 후진이 안 됨 | 기어가 D | `option: 16, gear: 2` |
| `rostopic hz` 가 설정값보다 크게 낮음 | rosbridge 포화 | §5 완화 순서 |
| RViz/rqt 가 안 열림 | X11 | 호스트에서 `xhost +local:docker`, `DISPLAY` 확인 |

## 7. 첫 연동에서 답을 내야 할 것

순서대로, 전부 코드 작성 없이 하루 안에 끝난다. **이게 다른 모든 작업의 전제다.**

1. [ ] 카메라 4대가 rosbridge 로 실제로 올라오는가 (센서 `commType` ROS 값 확정)
2. [ ] `rostopic hz` 실측 → §5 완화가 필요한가
3. [ ] `header.stamp` 가 유효한가 · sim time 인가 wall time 인가 (`/use_sim_time` 결정)
4. [ ] `/ctrl_cmd` 를 켜고 차가 실제로 움직이는가
5. [ ] `/Service_MoraiEventCmd` 로 **후진 기어**가 들어가는가
6. [ ] MORAI pitch 양수가 렌즈 아래인가 위인가 (BEV 지면 격자 오버레이)
7. [ ] `/Ego_topic.position` 원점이 MGeo local frame 과 같은가
8. [ ] `distance_left/right_lane_boundary`, `cross_track_error` 가 주차장에서 채워지는가

결과는 이 문서에 바로 적는다. **10/05~11/01 시험 공백 뒤 복귀할 때 이 답들이 없으면 연결부터
다시 뚫어야 한다.**
