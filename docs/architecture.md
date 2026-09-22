# 디렉터리·모듈 경계

> **상태:** 초안 — 코드 유입 전 팀 합의로 확정한다.
> **역할:** 이 팀은 브랜치가 아니라 **경로 분리**로 충돌을 막는다. 따라서 이 문서가
> 협업 규약([collaboration.md](collaboration.md))보다 먼저다.

## 1. 왜 경계를 먼저 잡나

기능마다 브랜치를 파고 PR을 반복하는 비용은 소규모 팀에서 이득보다 크다. 대신
**한 사람이 주로 건드리는 디렉터리를 겹치지 않게 나누고** `main`에 자주 push한다.
경계가 흐리면 어떤 Git 방식을 써도 충돌은 반복되므로, 아래 표가 협업의 실제 정본이다.

## 2. 계획 구조

```
2026-2-VIP3/                  # catkin workspace root
├── docs/                     # 팀 공유 문서
├── docker/                   # ROS Noetic 개발 이미지·compose
├── config/                   # 파라미터 yaml, MORAI 프로필
├── data/                     # 소용량 맵·센서 프리셋만 (대용량은 저장소 밖)
├── scripts/                  # bridge·build·run 진입점
└── src/
    ├── vip3_msgs/            # 팀 내부 공용 message 계약
    ├── morai_bridge/         # MORAI UDP/ROS ↔ 차량 상태·제어
    ├── perception/           # 주차면·장애물 인지 (camera / LiDAR)
    ├── localization/         # GPS·IMU → 차량 pose
    ├── parking_planner/      # 주차 슬롯 선택과 전·후진 경로 생성
    ├── parking_controller/   # 경로 추종·기어 전환 제어
    ├── behavior/             # 탐색→정렬→진입→정차 상태기계
    └── integration_launch/   # 통합 launch만 (로직 금지)
```

## 3. 경로 오너십

| 영역 | 경로 | 담당 |
|------|------|------|
| 공용 message | `src/vip3_msgs/` | 미정 — 변경 시 전원 합의 |
| MORAI 브리지 | `src/morai_bridge/` | 미정 |
| 인지 | `src/perception/` | 미정 |
| 측위 | `src/localization/` | 미정 |
| 주차 경로 | `src/parking_planner/` | 미정 |
| 제어 | `src/parking_controller/` | 미정 |
| 상태기계 | `src/behavior/` | 미정 |
| 공용 infra | `docker/`, `scripts/`, `config/`, `docs/` | 미정 — 변경 전 공유 |

팀 구성이 정해지면 `미정`을 실제 담당자로 채우고, 그 후에는 **자기 경로 밖을 고칠 때만**
사전 합의한다.

## 4. 얇게 유지할 것

| 대상 | 이유 |
|------|------|
| `src/vip3_msgs/` | 모든 파트가 의존한다. 필드 추가는 consumer 빌드까지 확인 |
| `src/integration_launch/` | launch만 둔다. 로직이 들어오면 모두의 충돌 지점이 된다 |
| `config/` | 파일을 파트별로 쪼개 한 yaml을 여러 명이 고치지 않게 한다 |
| `scripts/` | 공용 진입점. 개인용 실험 스크립트는 자기 패키지 안에 둔다 |

## 5. 인터페이스 고정

각 패키지는 **입력/출력 토픽과 메시지 타입**을 README에 적고, 그 계약이 유지되는 한
내부 구현은 자유롭게 교체한다. 계약이 바뀌면 producer·consumer 담당이 함께 확인한다.
계약 표가 여러 개 생기면 `docs/contracts/topics.md`로 모은다.

## 6. 저장소에 넣지 않는 것

bag, dataset, model checkpoint, MORAI 맵 원본 같은 대용량은 저장소 밖 로컬 루트
`../data/`에 둔다. 코드에 `/home/<사용자>/...` 같은 호스트 절대경로를 넣지 않는다.
