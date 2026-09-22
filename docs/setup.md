# 개발 환경

> **문서 역할:** 가이드 — 처음부터 돌리기까지
> **담당:** 장원태
> **최종 수정:** 2026-09-23

## 1. 한 줄 요약

| 구성 | 기준 |
|---|---|
| 코드 편집·Git | **WSL2 Ubuntu** (또는 native Linux) 의 리눅스 파일시스템 |
| 실행 | **Docker** — 호스트에 ROS 를 직접 설치하지 않는다 |
| ROS | **Noetic** (Python 3.8) |
| MORAI SIM | **Windows** 에서 실행, **rosbridge** 로 연결 |
| 메시지 | `morai_msgs` **26.R1** @ `4c9be6f` (`beta_drive` 아님) |

호스트가 Ubuntu 22.04 이상이면 ROS Noetic apt 패키지가 **없다.** 그래서 ROS 는 컨테이너에서만
돌린다. 저장소는 `/mnt/c/...` 가 아니라 WSL 의 `~/...` 아래에 clone 한다
(`/mnt/c` 는 파일 I/O 가 느리고 권한 문제가 생긴다).

## 2. clone

```bash
git clone https://github.com/ahnsh03/2026-2-VIP3.git
cd 2026-2-VIP3
git checkout main && git pull
git submodule update --init --recursive     # morai_msgs 26.R1
git -C src/morai_msgs rev-parse HEAD         # 4c9be6f... 이어야 한다
```

## 3. 저장소 밖 데이터

```
~/projects/2026-2-Vertically Integrated Project 3/     ← 로컬 루트
├── 2026-2-VIP3/                 ← 이 저장소
├── data/                        ← $VIP3_DATA  (맵·수집 결과·bag)
│   ├── KATRI 맵 데이터 자료/
│   ├── VIP3_sensor_set_v1.json
│   └── VIP3_network_v1.json
└── external/
    ├── baselines/TwinLiteNetPlus/   ← 추론·학습이 import 한다 (pinned 90f1b86)
    └── 2026-ASMC/                   ← 참조용 clone (읽기 전용)
```

`docker_ros_up.sh` 가 `$VIP3_DATA` 를 컨테이너 `/data` 에 마운트한다. 기본값은 저장소와
나란한 `../data` 다.

TwinLiteNet+ upstream 이 없으면:

```bash
git clone https://github.com/chequanghuy/TwinLiteNetPlus.git ../external/baselines/TwinLiteNetPlus
git -C ../external/baselines/TwinLiteNetPlus checkout 90f1b8695ae311d5123b05f8534b2e11e42499d2
```

**커밋을 정확히 이 값으로 맞춰야 한다.** 가중치 로더가 검사하고 다르면 거부한다.

## 4. Docker Desktop (Windows 최초 1회)

1. Linux containers / WSL 2 모드
2. **Settings → Resources → WSL Integration** — 쓰는 배포판 Enable

**호스트 네트워킹은 켤 필요 없다.** 기본 compose 가 `ports: 9090:9090` 브리지 모드라
rosbridge 하나만 퍼블리시한다. 이미 host 모드를 쓰고 있으면:

```bash
./scripts/docker_ros_up.sh up-hostnet
```

## 5. 빌드·기동

```bash
# 호스트 (WSL)
xhost +local:docker                  # RViz/rqt 쓸 때만
./scripts/docker_ros_up.sh build     # 최초 또는 Dockerfile 변경 시
./scripts/docker_ros_up.sh up
./scripts/docker_ros_up.sh check     # 9090 퍼블리시 + Windows 도달 확인

# 컨테이너 터미널 A — 계속 띄워 둔다
docker exec -it vip3-ros-noetic bash
cd /root/ws
./scripts/build_ws.sh
source devel/setup.bash
./scripts/rosbridge.sh
```

MORAI 연결과 그 뒤 순서는 [simulator.md](simulator.md) §2.

코드만 바뀌었으면 이미지를 다시 빌드하지 않는다 — 컨테이너에서 `./scripts/build_ws.sh` 만
다시 돌린다. `up` 은 컨테이너를 재생성하므로 **주행·녹화 중에 실행하지 않는다.**

종료:

```bash
./scripts/docker_ros_up.sh down
xhost -local:docker
```

## 6. GPU

기본 이미지는 **PyTorch 2.1.2 + CUDA 11.8** 이다. RTX 30·40 시리즈(sm_86/sm_89)까지 커버한다.

**RTX 50 시리즈(sm_120)를 쓰면 기본 이미지로는 추론이 안 된다.** torch 는 2.4.1 이후
Python 3.8 을 버렸고 sm_120 은 cu128 부터 지원되는데 ROS Noetic 이 Python 3.8 에 묶여
있어서 단일 인터프리터로는 해결이 불가능하다. venv 를 분리한 오버레이를 쓴다:

```bash
./scripts/docker_ros_up.sh build-rtx50
./scripts/docker_ros_up.sh up-rtx50
```

**팀원 5명의 GPU 모델을 먼저 확인한다** (`nvidia-smi`). RTX 50 이 아무도 없으면
`Dockerfile.rtx50` 과 오버레이를 안 써도 된다.

학습 컨테이너는 별도다:

```bash
./scripts/docker_train_up.sh build
./scripts/docker_train_up.sh up      # vip3-perception-train, /data 를 읽기 전용으로
```

## 7. 시뮬레이터 없이 되는 것

**실기 없이도 여기까지 확인할 수 있다.** 새로 합류했으면 이것부터 돌려 본다.

```bash
# 전 패키지 단위 테스트 (156개, ROS·GPU 불필요)
for p in src/perception/drivable_bev src/perception/camera_semantic_perception \
         src/data_collection src/vip3_hd_map src/vip3_vehicle_state; do
  echo "== $p"
  PYTHONPATH="$p/src:src/perception/camera_semantic_perception/src" \
    python3 -m unittest discover -s "$p/test" -p 'test_*.py' 2>&1 | tail -3
done

# 센서셋 지면 가시영역 — 차 주변 어디가 사각인지
python3 tools/analyze_sensor_set_coverage.py \
  --sensor-set config/VIP3_sensor_set_v1_ros.json --image /tmp/coverage.png

# KATRI 맵에 무엇이 있는지 (주차면 포함)
python3 src/vip3_hd_map/scripts/inspect_katri_mgeo.py \
  --map-dir "../data/KATRI 맵 데이터 자료"

# 맵 렌더
python3 tools/render_mgeo_map.py --mgeo "../data/KATRI 맵 데이터 자료" \
  --out /tmp/katri.png --scale 1.5
```

## 8. 안 되면

[simulator.md](simulator.md) §6 의 증상 표를 먼저 본다. 그 다음:

```bash
docker exec -it vip3-ros-noetic bash -lc 'cd /root/ws && ./scripts/diagnose_morai_rosbridge.sh'
```

출력을 통째로 팀 채널에 붙인다.
