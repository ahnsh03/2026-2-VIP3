# 2026-2-VIP3

**2026-2 알파프로젝트 3** (FVE9003 / Vertically Integrated Project 3) 팀 저장소.
MORAI 시뮬레이터 기반 **자율주차 시스템**을 개발한다.

| 항목 | 값 |
|------|-----|
| 교과목 | 알파프로젝트 3 — 001분반, 화 19~20교시, 1학점 **Pass/Fail** |
| 지도교수 | 원종훈 (전기전자공학부) · 협력업체 교원 (주)모라이 |
| 주제 | MORAI 시뮬레이터 기반 **자율주차** |
| 맵 | K-City 2025 (`R_KR_PR_K-city_2025`) |
| 시뮬 연결 | **rosbridge** `ws://127.0.0.1:9090` (UDP 안 씀) |
| 메시지 | `morai_msgs` **26.R1** @ `4c9be6f` |
| 협업 | `main` 직접 push + 경로 오너십 — [docs/collaboration.md](docs/collaboration.md) |

## 먼저 읽을 문서

| 문서 | 내용 |
|---|---|
| [docs/setup.md](docs/setup.md) | **환경 구성 — 여기부터** |
| [docs/simulator.md](docs/simulator.md) | MORAI 연결·토픽·진단 |
| [docs/roadmap.md](docs/roadmap.md) | **역할 분담과 작업 계획** — 자기 이름과 첫 과제는 여기 |
| [docs/README.md](docs/README.md) | 전체 문서 목록 |

## 빠른 시작

```bash
git clone https://github.com/ahnsh03/2026-2-VIP3.git && cd 2026-2-VIP3
git submodule update --init --recursive

./scripts/docker_ros_up.sh build        # 이미지만 빌드·검증
# 이름을 정한 뒤 직접 실행할 때:
export VIP3_CONTAINER_NAME=vip3-parking-ros
./scripts/docker_ros_up.sh up
./scripts/docker_ros_up.sh build-ws
./scripts/docker_ros_up.sh bridge       # 별도 터미널: svm / view
```

MORAI 설정은 [docs/simulator.md](docs/simulator.md) §2.

**시뮬레이터 없이 설정·지도부터 확인:**

```bash
python3 tools/check_sensor_preset.py
python3 src/vip3_hd_map/scripts/inspect_katri_mgeo.py --map-dir "../data/KATRI 맵 데이터 자료"
```

## 구조

```
docs/      팀 문서          docker/   실행 환경        config/  파라미터 (기계 정본)
scripts/   실행 진입점      tools/    ROS 없이 도는 도구  weights/ 인지 가중치
src/
  morai_msgs/              submodule, 26.R1
  vip3_bringup/            기동 런치 + static TF
  vip3_vehicle_state/      /Ego_topic 정규화 + 기어 소유자
  vip3_hd_map/             KATRI MGeo 로더·RViz 시각화
  data_collection/         수집·bag·학습 데이터셋
  perception/
    camera_semantic_perception/   4뷰 TwinLiteNet+ 추론
    twinlite_morai/               학습 adapter
    drivable_bev/                 BEV 투영·융합
```

경로 오너십과 데이터 흐름은 [docs/architecture.md](docs/architecture.md).

## 현재 상태

코드 기반은 [2026-ASMC](https://github.com/INHAautonav/2026-ASMC)(자율주행 대회 레포)에서
이식했고, rosbridge 단일 전송 · morai_msgs 26.R1 · 4카메라(후방 포함) 기준으로 맞췄다.
가져온 것과 안 가져온 것은 [docs/porting-from-asmc.md](docs/porting-from-asmc.md).

현재 센서 정본과 SVM 설정은 [config/README.md](config/README.md)에서 확인한다.

1. **4어안 1280×960·179°·10Hz 수신과 SVM 미리보기를 확인했다.** 렌즈·주차선 좌표 보정은 남아 있다 → [docs/sensors.md](docs/sensors.md)
2. **네트워크 정본에 `/ctrl_cmd`가 있다.** 기어·External 제어 모드와 실제 주차 제어는 별도 검증 대상이다 → [docs/simulator.md](docs/simulator.md)
3. **KATRI 맵에 주차면 기하가 없다.** 원본에는 있었는데 전달이 안 됐다 →
   [docs/katri-map.md](docs/katri-map.md) §1

역할별 남은 작업은 [docs/roadmap.md](docs/roadmap.md)를 따른다.

## 일정

개발 가능 주는 **3·4·5 / 9 / 11·12·13 — 흩어진 7주** (09/14~11/29).
중간발표-1 **10/13** · 중간고사 **11/03** · 중간발표-2 **11/17** · 기능 동결 **12/06** · 최종 발표 **12/15**.
자세한 것은 [docs/course.md](docs/course.md) §3.

> **10/05~11/01 은 시험 기간으로 중단된다.** 10/04 전에 반드시 **돌아가는 상태로 커밋**하고
> [작업 기록](docs/notes/)을 남긴다. 반쯤 고친 채로 두면 복귀에만 한 주가 든다.

## 데이터

대용량(맵 원본·bag·데이터셋)은 저장소에 넣지 않고 로컬 루트 `../data/`(`$VIP3_DATA`)에 둔다.
예외는 `weights/` 의 TwinLiteNet+ 체크포인트 두 개다 — 파일당 약 8 MB 라 저장소에 둬야
팀원이 바로 추론을 돌릴 수 있다.
