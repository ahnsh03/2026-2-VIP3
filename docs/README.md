# 2026-2-VIP3 문서

## 순서대로 읽기

**처음 들어왔다면** → [setup.md](setup.md) → [simulator.md](simulator.md) → [roadmap.md](roadmap.md)

| 문서 | 역할 | 담당 |
|---|---|---|
| [course.md](course.md) | 교과목 일정·평가·중간고사 | 안승현 |
| [setup.md](setup.md) | 개발 환경 — WSL2 · Docker · 빌드 | 장원태 |
| [simulator.md](simulator.md) | **MORAI 연결·토픽·진단** | 안승현·장원태 |
| [sensors.md](sensors.md) | **261110 센서셋·실기 수신·SVM 미리보기** — 이전 센서 기록 포함 | 안승현 (확정), 하수영·김동현 (연동) |
| [msgs-26r1.md](msgs-26r1.md) | **morai_msgs 26.R1 필드·단위** | 안승현 |
| [katri-map.md](katri-map.md) | **KATRI 맵에 무엇이 있고 없나** | 안승현 |
| [roadmap.md](roadmap.md) | **11/10 역할 분담·통합 순서** — GT/인지 목표 계약·첫 과제·남은 일정 | 팀 공동 |
| [parking-five-week-plan.md](parking-five-week-plan.md) | **5주 계획 제안** — 팀 전체 75인시·개발자 가용 45인시, 논문·GT/인지 비교 | 팀 공동 검토 |
| [parking-perception-svm-review.md](parking-perception-svm-review.md) | **DMPR-PS·SS-PSD 검토** — 추가 의견 검증, SVM 재설계 논문·근접 가시성 | 팀 공동 검토 |
| [ss-psd-sensor-bev-baseline.md](ss-psd-sensor-bev-baseline.md) | **SS-PSD 채택·센서/BEV 설계** — 공식 클론, AVM-SLAM 기준, 근접 카메라 후보, 수집·라벨·학습 | 팀 공동 |
| [morai-fisheye-sensor-trial.md](morai-fisheye-sensor-trial.md) | **MORAI 4어안 첫 시험안** — UI 설정값·조건부 XYZ·근접/겹침 검증·핀홀 코드의 한계 | 안승현·하수영·김동현 |
| [presentations.md](presentations.md) | **발표 슬라이드 설계** — 개발자 실행 증거·결과를 반영 | 장원태 (제작·발표), 안승현 (공동 총괄) |
| [related-work.md](related-work.md) | **자율주차 참고 목록** — 논문·코드·가중치·데이터셋 | 하수영 |
| [porting-from-asmc.md](porting-from-asmc.md) | ASMC에서 가져온 것/안 가져온 것 | 안승현·장원태 |
| [architecture.md](architecture.md) | 디렉터리·경로 오너십 | 안승현 |
| [collaboration.md](collaboration.md) | `main` 푸시 규약 | 전원 |
| [notes/](notes/) | 주간 작업 기록 — 점검·보고서 재료 | 각자 |

패키지별 실행법은 `src/<패키지>/README.md` 에 있다. 여기에는 배경과 팀 규약만 둔다.

## 정본 우선순위

같은 값이 여러 곳에 생겼을 때 **위쪽을 믿는다.**

1. **기계 정본** — `config/VIP3_sensor_set_v1_ros.json`, `config/VIP3_network_v1.json`,
   `src/morai_msgs/msg/*.msg`
2. **팀 계약** — `config/vip3_topics.yaml` (토픽·프레임·단위)
3. **문서** — `docs/*.md`
4. 코드 주석

문서는 기계 정본의 사본이다. 어긋나면 **문서를 고친다.**

## 문서 형식

모든 문서 맨 위에 세 줄을 둔다.

```markdown
# <제목>

> **문서 역할:** <정본|가이드|제안|기록> — <한 줄 범위>
> **담당:** <이름>
> **최종 수정:** YYYY-MM-DD
```

계약·기록 문서에는 `> **기계 정본:** <경로>` 를 한 줄 더 붙인다.

| 등급 | 뜻 |
|---|---|
| **정본** | 지금 이게 사실이다. 코드와 어긋나면 둘 중 하나가 버그다 |
| **가이드** | 이렇게 하면 된다. 절차 |
| **제안** | 아직 합의 전. 논의용 |
| **기록** | 그때 그랬다. 현재 설정으로 읽지 말 것 |

## 유지 규칙

1. **한 곳에서만 정의하고 나머지는 링크한다.** 카메라 위치가 문서 세 곳에 복사되면 반드시
   틀어진다
2. **구현을 바꾸는 커밋에 문서 갱신을 포함한다.** `main` 직푸시라 리뷰가 없으므로 더 중요하다
3. 빈 문서를 미리 만들지 않는다

## 아직 없는 문서

| 문서 | 언제 |
|---|---|
| `contracts.md` | 모듈 입출력 계약 — 모듈 경계 확정 후 |
| `experiments.md` | 학습·수집 결과 append-only 기록 |
| `troubleshooting.md` | 막히는 것이 쌓이면. 당분간은 [simulator.md](simulator.md) §6 |

## 문서 밖

- 맵 원본·bag·데이터셋 → 저장소 밖 `$VIP3_DATA` (`../data/`)
- 교과목 PDF → 로컬 루트 `../docs/`
- 개인 개발 환경 → `~/projects/DEV-ENVIRONMENT.md`
