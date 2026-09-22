# 2026-2-VIP3

**2026-2 알파프로젝트 3** (FVE9003 / Vertically Integrated Project 3) 팀 저장소.
MORAI 시뮬레이터 기반 **자율주차 시스템**을 개발한다.

| 항목 | 값 |
|------|-----|
| 교과목 | 알파프로젝트 3 — 001분반, 화 19~20교시, 1학점 **Pass/Fail** |
| 지도교수 | 원종훈 (전기전자공학부) · 협력업체 교원 (주)모라이 |
| 주제 | MORAI 시뮬레이터 기반 **자율주차** |
| 맵 | KATRI (`R_KR_PG_KATRI`) |
| 시뮬 연결 | **rosbridge** `ws://127.0.0.1:9090` (UDP 안 씀) |
| 메시지 | `morai_msgs` **26.R1** @ `4c9be6f` |
| 협업 | `main` 직접 push + 경로 오너십 — [docs/collaboration.md](docs/collaboration.md) |

## 먼저 읽을 문서

| 문서 | 내용 |
|---|---|
| [docs/setup.md](docs/setup.md) | **환경 구성 — 여기부터** |
| [docs/simulator.md](docs/simulator.md) | MORAI 연결·토픽·진단 |
| [docs/roadmap.md](docs/roadmap.md) | **역할 분담과 작업 순서** |
| [docs/README.md](docs/README.md) | 전체 문서 목록 |

## 빠른 시작

```bash
git clone https://github.com/ahnsh03/2026-2-VIP3.git && cd 2026-2-VIP3
git submodule update --init --recursive

./scripts/docker_ros_up.sh build && ./scripts/docker_ros_up.sh up
docker exec -it vip3-ros-noetic bash
cd /root/ws && ./scripts/build_ws.sh && source devel/setup.bash
./scripts/rosbridge.sh
```

MORAI 설정은 [docs/simulator.md](docs/simulator.md) §2.

**시뮬레이터 없이도 여기까지 된다** — [docs/setup.md](docs/setup.md) §7:

```bash
python3 tools/analyze_sensor_set_coverage.py --sensor-set config/VIP3_sensor_set_v1_ros.json
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

**ROS·GPU 없이 호스트에서 156개 단위 테스트가 통과한다.** 아직 실기 연동은 안 했다.

### 지금 막고 있는 것 세 가지

1. **`/ctrl_cmd` 가 네트워크 프리셋에 없다.** 차를 움직일 수 없다 →
   [docs/simulator.md](docs/simulator.md) §4
2. **rosbridge 로 카메라 4대가 20 Hz 로 오는지 미검증.** 안 나오면 수집 설계가 바뀐다 →
   [docs/simulator.md](docs/simulator.md) §5
3. **KATRI 맵에 주차면 기하가 없다.** 원본에는 있었는데 전달이 안 됐다 →
   [docs/katri-map.md](docs/katri-map.md) §1

셋 다 코드가 아니라 **확인**으로 풀린다. [docs/roadmap.md](docs/roadmap.md) §4 가 그 순서다.

## 일정

개발 가능 주는 **3·4·5 / 9 / 11·12·13 — 흩어진 7주** (09/14~11/29).
중간고사 **11/03** · 계획서 발표 **11/10** · 기능 동결 **11/29** · 최종 발표 **12/08**.
자세한 것은 [docs/course.md](docs/course.md) §3.

> **10/05~11/01 은 시험 기간으로 중단된다.** 10/04 전에 반드시 **돌아가는 상태로 커밋**하고
> [작업 기록](docs/notes/)을 남긴다. 반쯤 고친 채로 두면 복귀에만 한 주가 든다.

## 데이터

대용량(맵 원본·bag·데이터셋)은 저장소에 넣지 않고 로컬 루트 `../data/`(`$VIP3_DATA`)에 둔다.
예외는 `weights/` 의 TwinLiteNet+ 체크포인트 두 개다 — 파일당 약 8 MB 라 저장소에 둬야
팀원이 바로 추론을 돌릴 수 있다.
