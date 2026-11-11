# 2026-2-VIP3

알파프로젝트 3 — MORAI 기반 자율주차 팀 저장소.
**4어안 SVM → SS-PSD 주차칸 인지 → 일반 주차칸 하나에 전방주차**를 구현한다.

| 환경 | 기준 |
|---|---|
| 시뮬레이터 / 맵 | MORAI 26.R1.H3 / `R_KR_PR_K-city_2025` |
| 차량 | `2023_Hyundai_ioniq5` |
| 연결 | ROS Noetic + rosbridge, 팀 기본 `9090` |
| 컨테이너 | `vip3-parking-ros` / `vip3/parking-ros:noetic` |
| 개발 규모 | 5명·5주·개인 주 3시간; 구현 담당 3명 |
| 학습 장비 | RTX 4060 8GB 또는 노트북 RTX 3060 6GB |

## 시작하기

1. **[실행 명령](docs/commands.md)** — 컨테이너 생성부터 bridge·SVM·수집·재생까지.
2. **[개발 계획](docs/project/plan.md)** · **[담당과 오너십](docs/project/team.md)** — 자기 파트의 다음 작업.
3. **[베이스라인](docs/project/baseline.md)** — 논문 적용 범위와 비교 실험.
4. **[문서 목록](docs/README.md)** · **[설정 정본](config/README.md)**.

컨테이너를 이미 만들었다면 호스트의 어느 디렉터리에서든 다음으로 입장한다.

```bash
docker exec -it vip3-parking-ros bash
# 컨테이너 안: /root/ws에서 바로 실행
./scripts/rosbridge.sh
```

MORAI Import/Connect/Play 이후 별도 터미널에서 SVM을 실행한다. 자세한 순서와 19090 예외는 실행 문서를 따른다.

## 현재 구현

4어안 수신, SVM 미리보기·차량 그림, bridge 상태 로그, bag 수집·재생, 차량 상태·기어 노드, MGeo 시각화, 주차 평가 함수가 있다. **렌즈·지면 정합, clean BEV, SS-PSD ROS 노드, 주차 경로·추종 통합은 남은 작업**이다.

제공 [주차 GT 31개](src/vip3_hd_map/data/katri/README.md)는 제어 선행 개발과 평가에 사용한다. 최종 인지 입력을 GT로 대신하지 않는다. 이전 TwinLite·핀홀 BEV 코드는 선택 도구로 남아 있으며 현재 베이스라인과 구분한다.

## 저장소 구조

```text
docs/       계획·역할·베이스라인·시스템·실행법
config/     MORAI import와 ROS 공통 설정
docker/     주차 ROS 환경, 별도 legacy/학습 환경
scripts/    실행·수집·학습 진입점
tools/      설정 검사와 오프라인 도구
src/        ROS 패키지·인지 코어·평가 함수
weights/    기존 TwinLite 선택 체크포인트
```

대용량 데이터·bag은 저장소 밖 `../data`(컨테이너 `/data`), 개인 연구·실험 기록은 `../docs`, 외부 소스는 `../external`에 둔다. 팀원이 실행할 때 필요한 설정·차량 이미지·작은 제공 GT는 저장소에 포함한다.
