# VIP3 실행 환경

> 담당: 장원태 · 갱신: 2026-11-11

기본 이미지는 **`vip3/parking-ros:noetic`**이다. 공식 ROS Noetic ros-core에서 시작해 rosbridge, OpenCV·NumPy·SciPy, rosbag, rqt_image_view, RViz와 catkin 개발 도구를 설치한다. MORAI 연결·4어안 SVM·수집·차량 상태·GT 지도 확인용이다. SS-PSD 모델 추론·학습을 위한 PyTorch/CUDA 환경은 별도 추가 대상이며 이 이미지의 실행 범위에 포함하지 않는다.

컨테이너 이름 제안은 **`vip3-parking-ros`**이고 compose의 기본 이름도 이 값이다. `VIP3_CONTAINER_NAME`으로 바꿀 수 있다. **이미지 빌드와 실행 컨테이너 생성은 별개다.** `build`/`rebuild`는 이미지만 만들고, `up`을 직접 실행할 때 컨테이너·네트워크·빌드 볼륨이 생성된다. 인자 없이 스크립트를 호출하면 도움말만 출력한다.

## 1. 코드와 데이터

Windows에서 MORAI, WSL2/Linux에서 편집·Git·Docker 명령을 실행한다. Docker Desktop은 Linux containers/WSL 통합을 사용한다. 기본 구성은 CPU로 동작하므로 NVIDIA 컨테이너 런타임 없이 SVM을 확인할 수 있다.

```bash
git clone https://github.com/ahnsh03/2026-2-VIP3.git
cd 2026-2-VIP3
git submodule update --init --recursive
git -C src/morai_msgs rev-parse --short=7 HEAD  # 4c9be6f (26.R1)
```

개발 중인 변경은 해당 PR 브랜치를 checkout한다. 저장소 옆 `../data`에 MORAI 맵, 센서 export, 수집 결과를 둔다. 기본 compose는 이 경로를 `/data`에 연결한다. 데이터가 다른 위치에 있으면 `export VIP3_DATA="/원하는/데이터/경로"`를 지정한다. 이미지 빌드에 맵·bag·가중치·외부 프로젝트를 복사하지 않는다.

## 2. 이미지만 빌드

아래 명령은 모두 **저장소 루트**에서 실행한다.

```bash
./scripts/docker_ros_up.sh build
# 필요할 때 캐시 없이 이미지 재빌드만 수행:
# ./scripts/docker_ros_up.sh rebuild
```

빌드 과정의 별도 stage에서 VIP3 workspace 전체 catkin 빌드, MORAI 메시지 스키마, fisheye 테스트 7개, bag 프로파일 테스트 10개와 정적 검사 4개를 검증한다. 최종 이미지에는 검증 로그만 남기고 테스트용 소스·build·devel은 포함하지 않는다. 이 검증은 실행 컨테이너를 생성하지 않으며 MORAI 실시간 수신·GUI 표시 검증을 대체하지 않는다.

빌드 파일: `docker/ros-noetic/Dockerfile`. 직접 빌드하려면:

```bash
docker build -f docker/ros-noetic/Dockerfile -t vip3/parking-ros:noetic .
```

**2026-11-10 빌드 결과:** 로컬 이미지 약 4.48GB. 전체 catkin 빌드, 테스트 17개, 정적 검사 4개 통과. [검증 기록](analysis/parking-image-build-261110.json). 실행 컨테이너는 생성하지 않았으며, 새 이미지의 MORAI 연결·GUI 표시는 직접 실행할 때 확인한다.

## 3. 이름을 정한 뒤 직접 생성

**팀 기본 포트는 9090**이다. 아래는 사용자가 실행하기로 했을 때의 명령이다.

```bash
export VIP3_CONTAINER_NAME=vip3-parking-ros  # 제안 이름; 변경 가능
./scripts/docker_ros_up.sh config          # 생성 없이 최종 설정 확인
./scripts/docker_ros_up.sh up              # 여기서 처음 컨테이너 생성
./scripts/docker_ros_up.sh build-ws        # 첫 기동/소스 변경 후 catkin 빌드
```

이름을 변경했다면 이후 터미널에서도 같은 `VIP3_CONTAINER_NAME`을 지정한다. `build`와 `up`을 한 줄로 묶을 필요가 없다. build/devel은 프로젝트 전용 Docker 볼륨에 보관해 호스트 작업 트리에 다른 UID의 빌드 산출물이 쌓이는 것을 줄인다.

**현재 검증 PC의 예외:** Windows가 9090을 예약한 상태라 `up` **전에** 다음을 실행한다. 다른 팀원의 기본값은 그대로 9090이다.

```bash
export MORAI_BRIDGE_PORT=19090
python3 tools/export_local_morai_presets.py --port 19090 --output-dir ../data/local_morai
# 이후 위의 up / build-ws 실행
```

기본 포트에서는 `config/VIP3_sensor_set_261110_ros.json`과 `config/VIP3_network_261110.json`을 MORAI에 Import한다. 19090 예외를 쓰면 `../data/local_morai`에 생성된 두 사본을 Import한다. WebSocket 검증 때문에 외부·내부·MORAI 접속 포트는 동일해야 한다.

## 4. 직접 노드 실행

컨테이너가 실행 중이면 **호스트의 현재 디렉토리와 관계없이** 터미널 세 개에서 각각 접속한다. 접속 위치는 `/root/ws`이다.

```bash
docker exec -it vip3-parking-ros bash
```

```bash
# 터미널 A: 계속 유지
./scripts/rosbridge.sh
```

MORAI에서 두 프리셋을 Import한 뒤 센서·네트워크 **ROS Connect / Play**를 누른다.

```bash
# 터미널 B: 실제 수신 확인 후 SVM 실행
source devel/setup.bash
python3 scripts/verify_morai_topics.py --duration 15
roslaunch drivable_bev fisheye_svm.launch
```

```bash
# 터미널 C: 원본 4뷰 + SVM 패널
vip3-ros-gui rqt-image /perception/svm/panel
```

호스트의 저장소 루트에서는 기존 `./scripts/docker_ros_up.sh bridge`, `svm`, `view` 명령도 동일하게 사용할 수 있다.

### bridge 터미널의 센서 상태 로그

`./scripts/rosbridge.sh`는 진단 노드를 함께 실행해 **5초마다** WebSocket 클라이언트 수와 필수 토픽(현재 Ego + 카메라 4개)의 수신 Hz / 설정 Hz, 마지막 수신 후 경과 시간, 구간 메시지 수를 출력한다. 토픽과 설정 Hz는 `config/vip3_topics.yaml`에서 읽는다. 이미지는 디코딩하지 않고 수신 횟수만 센다.

- `OK`: 최근 수신 중이며 설정 Hz의 60% 이상.
- `MISSING`: 실행 후 한 번도 수신하지 않음. 연결·Play 전에는 정상적으로 나타난다.
- `STALE`: 마지막 수신 후 기본 2초 이상 경과. 저주기 센서는 설정 주기의 3배까지 허용한다.
- `SLOW`: 최근 수신은 있지만 구간 평균 Hz가 설정값의 60% 미만. 시작 직후 부분 구간에도 나타날 수 있다.

Hz는 ROS 구독자가 실제로 받은 메시지 수를 **실제 경과 시간**으로 나눈 값이다. `last`는 소스 타임스탬프 지연이나 네트워크 지연이 아니다. 시뮬레이션 시간이 멈춰도 계속 출력하며, `clients`는 센서 수가 아닌 rosbridge에 연결된 WebSocket 클라이언트 수이다. `unknown`은 연결 상태 메시지를 아직 받지 못했다는 뜻이다. 미수신/저속 로그만으로 센서 고장이나 패킷 손실을 단정하지 않는다.

컨테이너 안에서 다음처럼 주기를 조절하거나 끌 수 있다:

```bash
MORAI_TOPIC_LOG_INTERVAL=10 ./scripts/rosbridge.sh  # 10초마다
MORAI_TOPIC_MONITOR=false ./scripts/rosbridge.sh   # 센서 요약 로그 끄기
# 필요하면 MORAI_TOPIC_STALE_AFTER=3 으로 오래된 수신의 기준을 조정
```

이미 실행 중인 bridge에는 자동 적용되지 않는다. 해당 터미널에서 `Ctrl+C` 후 다시 실행하고 MORAI를 재연결한다. 새 실행에서는 bridge와 진단 노드를 하나의 roslaunch가 관리해 함께 종료한다. 기존 외부 roscore는 유지하고, 이 roslaunch가 새로 시작한 master는 함께 종료한다. 소스가 마운트되어 있으므로 이 변경을 적용하기 위해 Docker 이미지를 다시 빌드할 필요는 없다.

SVM은 아직 미보정 미리보기다. 최신 위치·해상도·실기 결과와 남은 캘리브레이션은 [sensors.md](sensors.md)를 따른다. 현재 이미지는 1280×960 입력을 처리하되 SVM 출력은 512×512다.

파일까지 저장하려면 `svm` 대신 다음을 쓴다. `save_dir`는 기존 결과와 다른 이름으로 정한다.

```bash
./scripts/docker_ros_up.sh shell
# 컨테이너 안
source devel/setup.bash
roslaunch drivable_bev fisheye_svm.launch save_dir:=/data/sensor_checks/my_run
```

GUI에 X11 권한 오류가 있을 때만 WSL에서 `xhost +si:localuser:root`를 실행하고 다시 시도한다. WSLg의 `DISPLAY`와 `/tmp/.X11-unix`를 기본 사용한다. 실제 GUI 연결은 사용자가 컨테이너를 생성한 후 확인한다.

## 5. 종료·정리

```bash
./scripts/docker_ros_up.sh stop  # VIP3의 모든 노드 종료, 컨테이너 보관
./scripts/docker_ros_up.sh down  # VIP3 컨테이너·compose 네트워크 제거
```

`down`은 소스·`../data`·빌드 볼륨을 삭제하지 않는다. 기본 compose 프로젝트 이름은 `vip3-parking`이다. ASMC 컨테이너/이미지에는 적용하지 않는다. 학습·수집 결과 보존을 위해 전체 `docker system prune`은 정리 명령에 포함하지 않는다.

## 6. 이전 GPU 환경

기존 TwinLite CUDA Dockerfile은 `Dockerfile.legacy-twinlite`에 보존했다. RTX50 오버레이와 `build-rtx50`는 해당 과거 경로를 위한 선택 기능이다. 기본 SVM 실행에는 이 파일이나 TwinLite 외부 clone이 필요 없다. SS-PSD GPU 환경을 추가할 때는 현재 논문의 의존성에 맞춘 별도 이미지로 검증한다.
