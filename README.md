# 2026-2-VIP3

**2026-2 알파프로젝트 3 (Vertically Integrated Project 3)** — MORAI 시뮬레이터 기반
**자율주차 시스템** 팀 통합 저장소.

| 항목 | 값 |
|------|-----|
| GitHub | https://github.com/ahnsh03/2026-2-VIP3 |
| 교과목 | 2026-2 알파프로젝트 3 |
| 목표 | MORAI SIM 기반 자율주차 (주차면 인지 → 경로 생성 → 전·후진 제어) |
| 맵 | KATRI (`R_KR_PG_KATRI`, K-CITY 구간 주행 가능 영역 포함) |
| 협업 | `main` 직접 push/pull + 경로 오너십 — [docs/collaboration.md](docs/collaboration.md) |

## 팀원이 먼저 읽을 문서

| 문서 | 내용 |
|------|------|
| [docs/architecture.md](docs/architecture.md) | 디렉터리·모듈 경계와 담당 경로 (**작업 전 필독**) |
| [docs/collaboration.md](docs/collaboration.md) | `main` 푸시 규약, 커밋 메시지, 충돌을 피하는 법 |
| [docs/README.md](docs/README.md) | 문서 색인 |

## 빠른 시작

```bash
git clone https://github.com/ahnsh03/2026-2-VIP3.git
cd 2026-2-VIP3
git checkout main && git pull
```

호스트가 Ubuntu 24.04/26.04면 ROS Noetic apt 패키지가 없다. 실행 환경은 **Docker**를
기준으로 하며, 이미지 정의는 이 저장소의 `docker/`에 둔다 (구성 예정 — §아래 상태).

## 저장소 구조

전체 계획과 각 경로의 담당은 [docs/architecture.md](docs/architecture.md)가 정본이다.

```
2026-2-VIP3/              # catkin workspace root (예정)
├── docs/                 # 팀 공유 문서
├── docker/               # ROS Noetic 개발 이미지 (예정)
├── config/               # 파라미터·프로필 (예정)
├── data/                 # 맵·센서 프리셋 등 소용량만 (대용량은 저장소 밖)
├── scripts/              # bridge·build·run 진입점 (예정)
└── src/                  # ROS 패키지 (예정 — architecture.md 참고)
```

## 현재 상태

저장소 초기화 단계다. 코드가 들어오기 전에 **디렉터리 경계와 담당부터 확정**한다
(이 팀의 협업 방식은 브랜치가 아니라 경로 분리로 충돌을 막는다).

- [x] 저장소 생성·문서 골격
- [ ] `docs/architecture.md` 모듈 경계 팀 확정
- [ ] `docker/` ROS Noetic 개발 이미지
- [ ] MORAI 브리지 연결과 첫 `ctrl_cmd` 주행
- [ ] 주차면 인지 → 주차 경로 → 제어 파이프라인

## 참고 저장소

이전 대회·프로젝트 코드는 이 저장소에 복사하지 않고 로컬 루트의 `external/`에서 참조한다.

| 참고 | 경로 |
|------|------|
| 2026-ASMC (AI·SW 모빌리티 팀 통합, MORAI/ROS1 전반) | `../external/2026-ASMC/` |
