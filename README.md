# 2026-2-VIP3

**2026-2 알파프로젝트 3** (FVE9003 / Vertically Integrated Project 3) 팀 저장소.
MORAI 시뮬레이터 기반 **자율주차 시스템**을 개발한다.

| 항목 | 값 |
|------|-----|
| 교과목 | 알파프로젝트 3 (FVE9003) — 전공선택, 1학점(설계 1) |
| 학기 | 2026-2 정규학기 |
| 지도교수·분반 | 원종훈 (알파프로젝트 1·2와 동일 분반 기준 — **확정 시 갱신**) |
| 주제 | MORAI 시뮬레이터 기반 **자율주차 시스템** |
| 시뮬 | MORAI SIM (버전 미정) · 맵 KATRI (`R_KR_PG_KATRI`) |
| 협업 | `main` 직접 push/pull + 경로 오너십 — [docs/collaboration.md](docs/collaboration.md) |
| GitHub | https://github.com/ahnsh03/2026-2-VIP3 |

> 알파프로젝트 1·2는 **자율주행** 주제였다. 이번 학기는 **자율주차**로 주제가 다르므로
> 이전 학기 코드·파라미터를 그대로 가져오지 않는다. 참고는 구조와 환경 수준에서만 한다.

## 먼저 읽을 문서

| 문서 | 내용 |
|------|------|
| [docs/course.md](docs/course.md) | 알파프로젝트 교과목이 무엇이고 무엇으로 평가되는지 |
| [docs/setup.md](docs/setup.md) | 개발 환경 기준 (WSL2 · Docker · MORAI) |
| [docs/architecture.md](docs/architecture.md) | 디렉터리·모듈 경계와 담당 경로 |
| [docs/collaboration.md](docs/collaboration.md) | `main` 푸시 규약, 커밋 메시지 |
| [docs/notes/](docs/notes/) | **연구노트** — 교과목 필수 평가도구 |

## 빠른 시작

```bash
git clone https://github.com/ahnsh03/2026-2-VIP3.git
cd 2026-2-VIP3
git checkout main && git pull
```

실행 환경(Docker 이미지, 빌드 시스템)은 아직 구성 전이다. [docs/setup.md](docs/setup.md) 참고.

## 현재 상태

**초기 단계 — 기술 스택·기반 논문·백본 모두 미정.** 주제를 구체화하면서 아래 순서로 채운다.

- [x] 저장소 생성, 교과목·환경·협업 문서 골격
- [ ] 팀 구성과 경로 오너십 확정 → [docs/architecture.md](docs/architecture.md)
- [ ] 관련 연구 조사, 접근 방식·기반 논문 선정 (교과목 4주차 발표와 연계)
- [ ] 모듈 경계 확정 → `src/` 구조 생성
- [ ] 실행 환경 구성 → `docker/`
- [ ] MORAI 연결과 첫 주행

## 저장소 구조

```
2026-2-VIP3/
├── docs/          # 교과목·환경·협업·연구노트
├── docker/        # 실행 환경 (구성 예정)
├── config/        # 파라미터 (구성 예정)
├── scripts/       # 실행 진입점 (구성 예정)
└── src/           # 코드 (모듈 경계 확정 후)
```

대용량 데이터(맵 원본, bag, 가중치)는 저장소에 넣지 않고 로컬 루트 `../data/`에 둔다.

## 참고 저장소

| 참고 | 경로 | 무엇을 |
|------|------|--------|
| 2026-ASMC | `../external/2026-ASMC/` | MORAI UDP/ROS 브리지, Docker Noetic 구성, 문서 체계 |
| 26-summer-VIP2 | `~/projects/2026-summer-Vertically Integrated Project 2/` | 알파프로젝트 2 (자율주행) — 교과목 운영·환경만 참고 |
