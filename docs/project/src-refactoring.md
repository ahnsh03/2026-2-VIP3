# src 구조와 개발 경계

2026-11-12 적용. **6개 catkin 패키지**로 정리했다. SVM과 주차칸 인지는 `vip3_perception`에 함께 두되, 실행 노드와 Python 모듈을 분리한다. 같은 담당자가 BEV 좌표·설정을 공유하면서 SVM만 CPU로 실행할 수 있는 구조다.

```text
src/
├── morai_msgs/                 # MORAI 26.R1 외부 submodule, pin 유지
├── vip3_msgs/                  # 팀 공용 ROS 메시지·서비스
├── vip3_bringup/               # bridge·진단·공통 실행·TF
├── vip3_data/                  # bag 기록·검사·재생, snapshot
├── vip3_perception/            # 어안 SVM, GCN 주차칸 인지
└── vip3_parking/               # 상태·기어·GT·평가, 이후 경로·추종
```

## 인지

```text
vip3_perception/
├── launch/svm.launch
├── scripts/svm_node.py         # ROS 입출력·프레임 처리 스케줄
├── src/vip3_perception/
│   ├── coordinates.py         # BEV 픽셀 중심 ↔ 차량 x/y(m)
│   └── svm/
│       ├── frames.py          # 시각 정렬·선택 JPEG 디코딩
│       ├── lens.py            # 어안 렌즈와 광학 좌표
│       ├── projection.py      # 지면 투영·합성
│       ├── ground.py          # 영상 겹침 기반 로컬 지면 추정
│       ├── pose.py            # 자세 보간·촬영 시차 보상
│       ├── calibration.py     # 대응점 기반 보정
│       ├── vehicle_geometry.py
│       └── visualization.py   # 차량 그림·진단 격자
├── config/
│   ├── svm.yaml               # 합성·보정 정본
│   ├── vehicles/              # 차량 치수·base_link 높이
│   ├── calibration/           # 보정 검증 근거
│   └── validation/            # 성능 검증 근거
├── assets/ioniq5_top_view.png
└── test/
```

GCN 검출은 `src/vip3_perception/detection/gcn_adapter.py`, `scripts/gcn_slot_detector_node.py`, `launch/gcn_slot_detector.launch`에 둔다. upstream 코드는 `external/parking/gcn-parking-slot`에 유지하고 복사·수정하지 않는다. SVM의 `/perception/svm/clean_bev/compressed` 출력이 그림·진단 표시 전 투영이며 검출 입력이다. 검출 결과는 `/perception/gcn/slots` JSON과 시각 확인용 overlay로 발행한다. PyTorch는 GCN adapter에서만 import해 SVM 단독 실행을 유지한다. checkpoint는 Git에 넣지 않고 launch 인자로 전달한다.

`coordinates.py`는 영상의 픽셀 중심과 차량 기준 미터 좌표를 연결한다. GCN JSON에는 base_link 프레임, 크기·범위·중심, SVM 보정 프로필, source stamp를 전달한다. 현재 슬롯 전용 메시지 대신 `std_msgs/String`을 사용한다.

## 수집과 주차

`vip3_data`의 실행 파일은 `record`, `replay`, `snapshot`으로 통일했다. `config/bag_profiles/`에는 `parking_raw`, 평가 채널을 추가하는 `parking_gt`와 공통 기반 설정이 있다. `snapshot_profiles/`는 연결 확인용 `dry_run`, 4카메라용 `parking_core`만 유지한다. bag의 원본 JPEG·source stamp를 보존하며 GCN 라벨/BEV dataset 변환기는 이후 개발한다.

`vip3_parking`은 `state.py`, `freshness.py`, `gt.py`, `frames.py`, `evaluation.py`와 상태·기어 ROS 노드를 유지한다. 이후 planner/controller를 같은 패키지에 추가한다. GT 31개 원본은 `data/katri/`에 보존한다. GT polygon의 입구 방향과 씬 원점 정합은 별도로 확인해야 한다.

## 제거·이동과 실행

| 이전 | 현재 |
|---|---|
| `drivable_bev` 어안 기능 | `vip3_perception.svm` |
| `fisheye_svm.launch` / `fisheye_svm_node.py` | `svm.launch` / `svm_node.py` |
| `data_collection` | `vip3_data` |
| `bag_recorder`, `bag_replay`, `sync_collector` launch | `record`, `replay`, `snapshot` launch |
| 루트 tools의 긴 SVM 파일명 | `tools/svm/{calibrate,replay,benchmark}.py` |
| TwinLite·핀홀 BEV·semantic/Capture 변환·옛 가중치·전용 Docker | 제거, Git 이력에서 조회 |
| 차량 상태·기어·전체 HD map·평가 디렉터리 | 이전 통합에서 `vip3_parking`으로 정리, 전체 도로망 제거 |

센서 프리셋·보정 수치·GT 원본, `/perception/svm/*`, `/vehicle/*`, ROS 노드·서비스 이름은 유지한다. 이전 이름의 호환 패키지는 두지 않는다. 갱신 후 생성된 build/devel/install을 정리하고 새로 빌드하는 명령은 [commands.md](../commands.md#setup)를 따른다.

## 담당

- 김동현: `vip3_bringup`, `vip3_data`, 주차 상태·기어 배선.
- 하수영: `vip3_perception`의 SVM·GCN·BEV 좌표 계약.
- 강도균: `vip3_parking`의 GT·경로·추종·평가.
- 안승현: 센서·차량·좌표 기준, 통합 검증. 장원태: 발표와 결과 정리.

공용 메시지와 센서·좌표 변경은 소비 파트에 전달한다. 상세 오너십은 [팀 문서](team.md), 알고리즘 범위는 [베이스라인](baseline.md)을 따른다.

## 이번 구조 변경 검증

새 build/devel/install 경로의 catkin 설치와 단위 테스트 97개를 통과했다. 저장된 4카메라 영상과 변화하는 25개 자세(워밍업 포함)를 이전 코드와 비교해 remap·가중치·mask·출력 픽셀 차이가 모두 0이었다. 보정 YAML·센서 JSON·차량 그림·GT 원본 바이트를 보존했다.

별도 ROS master에서 새 설치 경로의 launch로 snapshot 3묶음과 bag 기록→검사→재생→SVM 표시를 확인했다. JPEG·source stamp·bag 해시, 재생 종료 시 시계 복원, 영상 중단/잘못된 JPEG/TF 중단·시각 리셋 처리가 통과했다. 모의 송신과 기존 캡처를 사용한 회귀 검사이며 새로운 MORAI 주행·캘리브레이션 정확도 평가는 아니다.

작업별 결과와 인수인계는 [팀 작업 노트](../notes/README.md)에 남긴다. 이 구조 문서는 최신 코드 경계를 유지한다.
