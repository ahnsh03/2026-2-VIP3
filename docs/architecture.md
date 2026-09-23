# 디렉터리·모듈 경계

> **문서 역할:** 정본 — 저장소 구조와 경로 오너십
> **담당:** 안승현
> **최종 수정:** 2026-09-23

## 1. 왜 경계를 먼저 잡나

이 팀은 브랜치가 아니라 **경로 분리**로 충돌을 막는다. 기능마다 브랜치를 파고 PR 을
반복하는 비용은 소규모 팀에서 이득보다 크고, 파일·모듈 경계가 흐리면 어떤 Git 방식을 써도
충돌은 반복된다. 게다가 **모여서 개발하지 않는다** — 옆에서 "이 파일 지금 건드릴게"라고
말할 수 없다.

그래서 **누가 어느 디렉터리를 맡는지**가 협업 규약([collaboration.md](collaboration.md))보다
먼저다.

## 2. 원칙

| 원칙 | 설명 |
|---|---|
| **한 사람 = 한 경로** | 각자 "주로 건드리는" 디렉터리를 겹치지 않게 나눈다 |
| **공유 코어는 얇게** | `config/`, `docker/`, `scripts/` 는 변경 빈도를 낮추고 바꿀 때 합의한다 |
| **인터페이스 고정** | 입출력을 문서화하고, 계약이 유지되면 내부 구현은 자유롭게 교체 |
| **대용량은 저장소 밖** | bag·데이터셋·맵 원본은 `$VIP3_DATA`(`../data/`). 단 인지 가중치는 파일당 8 MB 라 `weights/` 에 둔다 |
| **호스트 경로 금지** | 코드·설정에 `/home/<사용자>/...` 를 넣지 않는다. `$VIP3_DATA`, `/root/ws`, `$(find <pkg>)` 를 쓴다 |
| **정본은 하나** | 같은 값을 두 곳에 적지 않는다. [docs/README.md](README.md) 의 정본 우선순위 |

## 3. 구조

```
2026-2-VIP3/
├── docs/            팀 공유 문서·연구노트
├── docker/          실행 환경
│   ├── ros-noetic/      런타임 + 추론 (rosbridge, RViz, torch cu118)
│   └── perception-train/ 학습 전용 (ROS 없음)
├── config/          파라미터 — 기계 정본
│   ├── vip3_topics.yaml            토픽·프레임·단위 계약
│   ├── VIP3_sensor_set_v1_ros.json MORAI Sensor 설정 (ROS 전환본)
│   ├── VIP3_network_v1.json        MORAI Network 설정
│   └── perception/                 target policy 등
├── scripts/         실행 진입점 (도커·rosbridge·학습·데이터셋)
├── tools/           ROS 없이 도는 오프라인 도구
├── weights/         인지 가중치 (TwinLiteNet+ Medium, 파일당 약 8 MB)
└── src/
    ├── morai_msgs/                     submodule, 26.R1 @ 4c9be6f
    ├── vip3_msgs/                      팀 자체 메시지 (최소한으로)
    ├── vip3_bringup/                   기동 런치 + static TF
    ├── vip3_vehicle_state/             /Ego_topic 정규화 + 기어 소유자
    ├── vip3_hd_map/                    KATRI MGeo 로더·RViz 시각화
    ├── data_collection/                수집·bag·학습 데이터셋
    └── perception/
        ├── camera_semantic_perception/  4뷰 TwinLiteNet+ 추론
        ├── twinlite_morai/              학습 adapter
        └── drivable_bev/                BEV 투영·융합
```

## 4. 데이터 흐름

```
MORAI (Windows)
  │  rosbridge ws://127.0.0.1:9090
  ▼
/cam_{front,left,right,rear}/image_jpeg/compressed
/velodyne_points  /gps  /imu  /Ego_topic  /Object_topic  /tf
  │
  ├─▶ vip3_vehicle_state ──▶ /vehicle/state, /vehicle/odom, /vehicle/gear
  │                          └──▶ /Service_MoraiEventCmd (기어·제어모드)
  │
  ├─▶ camera_semantic_perception ──▶ /perception/camera/{view}/{drivable,lane}/probability
  │                                    │
  │                                    ▼
  │                                  drivable_bev ──▶ /perception/bev/debug/{view}/*
  │                                    │              /perception/bev/camera/*  (융합)
  │                                    ▼
  │                                  (주차칸 검출 · 경로 생성 · 제어)  ← 아직 없음
  │                                    │
  │                                    ▼
  │                                  /ctrl_cmd  ──▶ MORAI
  │
  ├─▶ data_collection ──▶ $VIP3_DATA/{capture_runs,datasets,dataset_versions,bags}
  │                          └──▶ twinlite_morai 학습
  │
  └─▶ vip3_hd_map ──▶ /vip3_hd_map/{global,local}/* (RViz)
```

**빈 칸이 주차 로직이다.** 주차칸 검출 → 경로 생성 → 저속 후진 제어. 기반 논문이 정해지면
`src/parking/` 아래에 붙인다.

## 5. 경로 오너십

| 담당 | 경로 |
|---|---|
| **안승현** | `src/perception/drivable_bev/`, `src/vip3_hd_map/`, `src/perception/camera_semantic_perception/`, `tools/` |
| **장원태** | `docker/`, `scripts/`, `src/vip3_bringup/`, `src/vip3_vehicle_state/` |
| **강도균** | `config/VIP3_sensor_set_*.json`, `src/vip3_hd_map/config/vip3_parking_spaces.json` |
| **김동현** | `src/data_collection/`, `scripts/{sync_capture_data,curate_perception_frames,bake_perception_masks,build_perception_dataset,audit_semantic_capture}.py`, `src/vip3_eval/`(예정) |
| **하수영** | `src/perception/twinlite_morai/`, `scripts/{train,evaluate,render,smoke}_twinlite*.py`, `weights/` |
| 공동 (합의 후 변경) | `config/vip3_topics.yaml`, `config/VIP3_network_v1.json`, `src/vip3_msgs/`, `src/morai_msgs`(submodule) |
| 전원 | `docs/` (연구노트는 각자 파일) |

**자기 경로 밖을 고칠 때만** 사전에 합의한다. 그 외에는 바로 `main` 에 push 한다.

역할 배분의 근거와 조정 제안은 [roadmap.md](roadmap.md) §1~§3.

### 센서셋을 바꿀 때 같이 고칠 파일 (강도균)

센서 위치·해상도·FOV 가 바뀌면 네 곳이 같이 바뀐다. 하나라도 빠지면 BEV 가 조용히 틀린다.

1. `config/VIP3_sensor_set_v1_ros.json` — 기계 정본
2. `config/vip3_topics.yaml` — 팀 계약
3. `src/perception/drivable_bev/config/cameras_vip3_v1.yaml` — **`source_sha256` 포함**
   (`sha256sum config/VIP3_sensor_set_v1_ros.json`). 안 맞으면 BEV 노드가 기동을 거부한다
4. `src/vip3_bringup/launch/static_tf.launch` — `python3 tools/dump_static_tf.py` 출력으로 교체

바꾼 뒤 확인:

```bash
# 지면 가시영역 (빠른 확인)
python3 tools/analyze_sensor_set_coverage.py --sensor-set config/VIP3_sensor_set_v1_ros.json
# SVM 기준 정밀 평가 — 사각 폭·중복률·주차선 해상도까지 (drivable_bev 와 같은 코드)
python3 tools/design_svm_sensor_set.py --sensor-set config/VIP3_sensor_set_v1_ros.json
# 후방 카메라 기하 회귀
PYTHONPATH="src/perception/drivable_bev/src:src/perception/camera_semantic_perception/src" \
  python3 -m unittest discover -s src/perception/drivable_bev/test -p 'test_rear*.py'
```

## 6. 정합성 검사기 3개

시뮬레이터를 켜기 전에, **조용히 틀리는 종류의 오류**를 잡는다. 셋 다 ROS·GPU 없이 돈다.

```bash
python3 tools/check_topic_contract.py      # 토픽 이름이 config/vip3_topics.yaml 과 맞는가
python3 tools/check_node_imports.py        # 노드가 없는 이름을 import 하는가
python3 tools/check_launch_params.py --all # 런치가 노드 파라미터를 설정할 수 있는가
```

왜 셋 다 필요한가 — 단위 테스트가 못 보는 영역이 서로 다르다.

| 검사기 | 못 잡던 사고 |
|---|---|
| `check_topic_contract` | 이름이 한 글자 다른 토픽. 노드는 멀쩡히 뜨고 구독만 안 된다 |
| `check_node_imports` | 노드 스크립트는 `rospy` 가 있어야 import 되므로 호스트 테스트가 못 건드린다. 실제로 `bag_replay_node.py` 가 삭제된 이름을 import 한 채 **기동 즉시 죽는** 상태였고, 테스트 190개는 전부 통과 중이었다 |
| `check_launch_params` | `<rosparam file>` 이 없는 파일을 가리키면 `roslaunch` 가 즉사한다. `--all` 은 "런치 인자로 문서화됐는데 실제로는 못 켜지는" 기능도 보여준다 |

## 7. 아직 없는 것

- `src/parking/` — 주차칸 검출·경로 생성·제어. 기반 논문 확정 후
- `src/vip3_eval/` — 주차 성공 판정. [roadmap.md](roadmap.md) §2
- `vip3_msgs` 확장 — 지금은 `VehicleState` + `SetGear` 뿐이다. stock 메시지로 되는 것은
  커스텀 타입을 만들지 않는다
