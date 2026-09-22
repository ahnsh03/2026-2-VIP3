# 개발 환경 기준

> **상태:** 골격 — 실행 환경(Docker 이미지·빌드)은 기술 스택 확정 후 채운다.
> 여기에는 **팀이 공유해야 하는 절차만** 적는다. 개인 PC 경로·계정은 각자 관리한다.

## 1. 환경 한 줄 요약

| 구성 | 기준 |
|---|---|
| 코드 편집·Git | **WSL2 Ubuntu** (또는 native Linux)의 리눅스 파일시스템 |
| 실행 환경 | **Docker** — 호스트에 ROS를 직접 설치하지 않는다 |
| MORAI SIM | **Windows**에서 실행, 컨테이너와 네트워크로 연결 |
| IDE | VS Code + WSL Remote |

호스트가 Ubuntu 22.04/24.04/26.04면 ROS Noetic apt 패키지가 **없다**. 그래서 ROS는
컨테이너에서만 돌린다. 저장소는 `/mnt/c/...`가 아니라 WSL의 `~/...` 아래에 clone한다
(`/mnt/c`는 파일 I/O가 느리고 권한 문제가 생긴다).

## 2. clone

```bash
git clone https://github.com/ahnsh03/2026-2-VIP3.git
cd 2026-2-VIP3
git checkout main && git pull
```

## 3. Docker Desktop (Windows 팀원 최초 1회)

1. Linux containers / WSL 2 모드로 실행
2. **Settings → Resources → WSL Integration** — 사용하는 배포판 Enable
3. **Settings → Resources → Network → Enable host networking** → Apply and restart

3번은 시뮬레이터와 UDP로 통신할 때 필요하다. 네트워크 방식(UDP / rosbridge)이 정해지면
이 문서에 포트와 토픽 표를 추가한다.

GUI(RViz 등)를 쓸 때만 호스트에서 `xhost +local:docker`를 실행한다.

## 4. 실행 환경 — 구성 예정

| 항목 | 상태 |
|---|---|
| ROS 배포판 | 미정 |
| Docker 이미지 (`docker/`) | 미정 |
| 빌드 시스템 | 미정 |
| MORAI 버전·연결 방식 | 미정 |
| 센서 프리셋 | 미정 (로컬 루트 `../data/Sample Sensorset/` 참고) |

확정되면 이 절을 빌드·실행 순서로 교체한다.

## 5. 데이터

맵 원본, bag, 학습 가중치 같은 대용량은 **저장소에 넣지 않는다.** 로컬 루트 `../data/`에
두고, 코드에는 `/home/<사용자>/...` 같은 호스트 절대경로를 넣지 않는다.

현재 로컬 루트에 있는 것:

| 경로 | 내용 |
|---|---|
| `../data/KATRI 맵 데이터 자료/` | MGeo link/node·신호등·횡단보도 등 set |
| `../data/KATRI 맵(K-CITY 구간 한정) 주행 가능 영역 자료/` | 주행 가능 영역 + `mgeo_toolkit` |
| `../data/Sample Sensorset/` | MORAI 센서 프리셋 JSON 모음 |

## 6. 참고

- 개인 환경 상세(도구 버전, 셸 변수, 컨테이너 목록): `~/projects/DEV-ENVIRONMENT.md`
- Docker Noetic 구성 예시: `../external/2026-ASMC/docker/ros-noetic/`
