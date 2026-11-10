# 디렉터리·모듈 경계

> **문서 역할:** 정본 — 저장소 구조와 경로 오너십
> **담당:** 안승현
> **최종 수정:** 2026-11-10

**현재 역할:** 강도균은 GT 기반 경로 생성·추종, 김동현은 ROS bridge·수집, 하수영은 SVM·SS-PSD 적용을 담당한다. 안승현은 센서셋 확정·공유, 장원태는 발표자료 제작·발표를 맡으며 두 사람은 공동 관리·MORAI 총괄·질의응답을 담당한다. 개발 순서와 인터페이스 초안은 [roadmap.md](roadmap.md)를 따른다. 아래 저장소 구조는 현재 구현이고 SS-PSD 주차 경로는 앞으로 연결할 대상이다.

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
| **대용량은 저장소 밖** | bag·데이터셋·맵 원본은 `$VIP3_DATA`(`../data/`). 기존 TwinLite 가중치 크기를 SS-PSD에 일반화하지 않는다. 새 가중치는 실제 크기를 확인하고 출처·hash·경로를 기록한다 |
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
│   ├── VIP3_sensor_set_261110_ros.json MORAI Sensor 설정 (ROS 전환본)
│   ├── VIP3_network_261110.json        MORAI Network 설정
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

**SS-PSD 기반 목표 흐름**은 원본 카메라 → SVM 컬러 BEV → SS-PSD → 입구·방향 → 목표 pose → 경로 생성·저속 전방주차다. 위 기존 semantic 파이프라인을 모두 통과해야 하는 것은 아니다. 계획/추종은 `src/parking/` 아래 신규 구현을 제안한다. GT 목표와 인지 목표를 같은 인터페이스로 연결한다.

## 5. 경로 오너십

| 담당·상태 | 주요 경로·작업 경계 |
|---|---|
| **강도균 — 주업무 확정** | `src/parking/`(계획·추종·GT 목표 어댑터 예정). 원본 GT는 읽기 전용이며 일반화한 목표 pose를 입력받도록 구성. 인지 관측→지도 목표 변환은 입력 어댑터로 최소 구현하는 안을 제안 |
| **김동현 — 주업무 확정** | `src/data_collection/`, rosbridge 기동·진단·수집 관련 `scripts/`. 네트워크 설정·기어/제어 전달 검증 |
| **하수영 — 주업무 확정** | `src/perception/drivable_bev/`, `src/perception/slot_detector/`(예정). SVM·인지 전처리/후처리·가중치 연결. 차량 기준 입구·방향 출력까지 담당하는 안을 제안 |
| **안승현 — 주업무 확정** | 센서 JSON 확정·공유, 전체 관리·MORAI 총괄·질의응답. 센서 기준·모듈 계약을 검토하며 별도 통합 구현 인력으로 계산하지 않음 |
| **장원태 — 주업무 확정** | `docs/presentations.md`, 발표자료 제작·발표, 안승현과 공동 관리·질의응답. 별도 평가·학습 구현 인력으로 계산하지 않음 |
| 공동 | `config/vip3_topics.yaml`, `config/VIP3_network_261110.json`, 센서 JSON·카메라 파라미터·static TF, `src/vip3_msgs/` |
| 기존 공용 코드 | `docker/`, `src/vip3_bringup/`, `src/vip3_vehicle_state/`, `src/vip3_hd_map/`, 공통 `tools/`는 기존 담당과 인계하며 작업. MORAI 메시지 submodule의 버전을 임의로 바꾸지 않음 |
| 전원 | `docs/` (연구노트는 각자 파일) |

**자기 경로 밖을 고칠 때만** 사전에 합의한다. 그 외에는 바로 `main` 에 push 한다.

역할 검토·첫 과제·인터페이스는 [roadmap.md](roadmap.md) §1~§3을 따른다. 5명의 주업무는 사용자 결정이며 세부 신규 경로·통합 경계는 구현을 위한 제안이다. `/ctrl_cmd` 송신은 강도균의 제어기 한 곳에서 소유하고 김동현은 통신을 검증한다. 평가용 결과는 각 구현자가 남기고 장원태가 발표자료로 정리한다.

### 센서셋을 바꿀 때 같이 고칠 파일 (안승현 확정, 하수영·김동현 연동 확인)

센서 위치·해상도·FOV가 바뀌면 아래 종류의 파일을 함께 갱신한다. v1 파일명은 현 구현 예시이며 임시 센서셋을 유지할 의무는 없다. 새 버전 경로·해시·토픽으로 함께 전환한다.

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

**261110 현재 센서 전환:** 현재 정본은 `config/VIP3_sensor_set_261110_ros.json`과 `config/VIP3_network_261110.json`이다. 어안 미리보기는 `fisheye_svm.launch`를 사용하고 `check_sensor_preset.py`로 JSON·토픽 계약·SVM 파라미터를 대조한다. 기존 핀홀 설정을 새 어안에 적용하지 않는다. static TF도 새 4카메라 배치로 변경했다. 상세 절차는 [sensors.md](sensors.md)의 현재 적용 절을 따른다.

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

## 7. 추가 구현·통합 대상

- `src/vip3_eval/` — 김동현의 `parking_eval.py`와 테스트가 main에 추가됨. ROS 패키지화·실기 평가 연결 여부는 별도 확인
- `src/parking/` — GT 목표 공급·주차 경로 생성·추종·정지 (강도균)
- `src/perception/slot_detector/` — SS-PSD 인지 어댑터 (하수영)
- `src/perception/slot_geometry/` — 별도 패키지 미구현. 하수영의 차량 기준 관측 출력과 강도균의 목표 입력 어댑터로 나누는 안을 제안하며 별도 전담자를 두지 않음
- `vip3_msgs` 확장 — 지금은 `VehicleState` + `SetGear` 뿐이다. stock 메시지로 되는 것은
  커스텀 타입을 만들지 않는다
