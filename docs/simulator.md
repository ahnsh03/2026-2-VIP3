# MORAI 연결과 진단

> **문서 역할:** 현재 시뮬레이터 연결 절차
> **담당:** 안승현 · 장원태
> **최종 수정:** 2026-11-11

현재 환경은 `R_KR_PR_K-city_2025`, `2023_Hyundai_Ioniq5`, MORAI 26.R1.H3이다. rosbridge WebSocket으로 ROS Noetic에 연결하고 저장소의 `morai_msgs` 26.R1을 사용한다.

## 1. 정본

| 용도 | 파일 |
|---|---|
| MORAI Sensor Import | `config/morai/sensors.json` |
| MORAI Network Import | `config/morai/network.json` |
| 토픽·프레임·주기 계약 | `config/ros/topics.yaml` |
| bridge 기본값 | `config/ros/rosbridge.env` |

팀 기본 포트는 **9090**이다. Windows에서 예약된 PC만 19090을 쓰며, 개인 변경본은 정본에서 생성해 `../data/local_morai/`에 둔다. [파일 분류와 생성 명령](../config/README.md), [Docker 환경 구성](setup.md).

## 2. 기동 순서

컨테이너가 실행 중이면 각 터미널에서 접속한다. 호스트의 현재 디렉터리는 관계없다.

```bash
docker exec -it vip3-parking-ros bash
```

```bash
# 컨테이너 터미널 A
./scripts/rosbridge.sh
```

MORAI에서 위 맵·차량을 선택하고 Network와 Sensor 정본을 Import한 뒤 각각 ROS Connect, Play를 누른다. rosbridge가 서버이므로 먼저 실행한다.

```bash
# 컨테이너 터미널 B
source devel/setup.bash
python3 scripts/verify_morai_topics.py --duration 15
roslaunch drivable_bev fisheye_svm.launch
```

```bash
# 컨테이너 터미널 C
vip3-ros-gui rqt-image /perception/svm/panel
```

## 3. 현재 입력과 확인 범위

카메라는 front/rear/left/right **4대 모두 1280×960, 179°, 10Hz**이다. 토픽은 `/cam_{front,rear,left,right}/image_jpeg/compressed`, 프레임은 `cam_{front,rear,left,right}`다. 현재 센서 프리셋에 LiDAR/GPS/IMU는 없다. 차량 상태는 네트워크의 `/Ego_topic`을 사용한다.

11월 10일 15초 수신 검사에서 카메라 4대는 9.969–10.014Hz였다. 이는 당시 PC의 측정 결과이며 모든 PC의 처리량 보장이 아니다. bridge 터미널은 기본 5초마다 수신 Hz·메시지 수·마지막 수신 후 경과 시간을 출력한다. [센서 설정·실기 기록](sensors.md).

네트워크 정본에는 ROS `/ctrl_cmd`가 포함되어 있다. 채널 설정과 차량 이동 성공은 별도 검증 대상이다. 제어 담당은 External 제어 모드, 기어 서비스, 명령 단위와 발행자 하나 원칙을 확인한다. 프리뷰 확인에는 차량 제어 명령이 필요하지 않다.

## 4. 진단

```bash
./scripts/diagnose_morai_rosbridge.sh
rostopic hz /cam_front/image_jpeg/compressed
rostopic echo -n1 /cam_front/image_jpeg/compressed/header
```

| 증상 | 먼저 확인할 내용 |
|---|---|
| Connect 실패 | bridge 실행, MORAI·Docker·bridge 포트 일치, Windows 예약 포트 |
| 일부 카메라만 수신 | 현재 Sensor 정본, ROS Connect·Play, 방향별 토픽·frame_id |
| 카메라는 오는데 Ego 없음 | Network 정본 Import와 별도 ROS Connect |
| SVM waiting/stale | 네 카메라 수신 지속, 헤더 동기 오차, Pause 상태 |
| 차량 그림이 없거나 크기가 이상함 | `vehicle_image` 경로와 [SVM 마스크 비교](sensors.md) |

SVM은 아직 미보정 미리보기다. 차량 그림 스케일 확인과 주차선 좌표 정확도 검증은 구분한다. 보정·학습 입력 연결은 [sensors.md](sensors.md)를 따른다.
