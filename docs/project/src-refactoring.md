# src 리팩토링 계획

2026-11-11 기준. 현재 실행 가능한 **4어안 SVM → SS-PSD → 전방주차** 흐름에 맞춰 패키지 책임을 나눈다. 이번 변경은 성능 개선과 계획 문서이며, 아래 패키지 이동·이름 변경·레거시 삭제는 아직 실행하지 않았다.

## 확인한 문제

`src`에는 catkin 패키지 8개와 package.xml이 없는 Python 코드 `twinlite_morai`, `vip3_eval`이 섞여 있다. ROS 패키지 이름은 `vip3_*`와 일반 명칭이 혼재하고, 일부만 `perception/` 아래에 있다.

| 현재 위치 | 실제 역할·문제 | 처리 방향 |
|---|---|---|
| `perception/drivable_bev` | 현재 어안 SVM과 이전 핀홀 RGB/semantic BEV·차선 피팅이 함께 있음. Python 파일 39개, 약 6,674줄(테스트 포함, 성능 작업 전) | SVM에 필요한 코드·설정·검사만 `vip3_svm`으로 추출 |
| `drivable_bev/__init__.py` | SVM 하위 모듈을 import해도 핀홀·차선·semantic 코어를 함께 import. `model_grid_projection` 때문에 `camera_semantic_perception`에 의존 | 새 패키지의 `__init__.py`는 가볍게 유지. 사용하지 않는 코어를 자동 import하지 않음 |
| `perception/camera_semantic_perception`, `perception/twinlite_morai` | 이전 TwinLite segmentation·학습 경로. SS-PSD 슬롯 출력과 인터페이스가 다르고 기본 ROS 이미지에 PyTorch도 없음 | 주차 런타임에서 제외. 필요한 유틸리티를 먼저 추출한 뒤 코드·관련 launch·scripts·설정을 함께 삭제하고 Git 이력으로 보존 |
| `data_collection` | bag 기록·재생 외에 구형 Capture, semantic 팔레트·mask dataset·TwinLite 변환까지 포함 | 현재 `parking_raw` 경로를 우선 유지. 구형 파이프라인은 의존성 확인 후 제거. 이름 변경은 마지막 |
| `vip3_eval` | 평가 함수·테스트는 있지만 ROS/Python 패키지 선언이 없음 | 우선 순수 Python 평가 라이브러리로 정리. ROS 노드가 필요해질 때 catkin 진입점 추가 |
| `vip3_hd_map` | MGeo 표시, 주차 GT, 좌표 변환 | 이름·원본 GT 유지. GT 목표 생성과 인지 입력을 분리하고, 원본 JSON을 수정하지 않음 |
| `vip3_vehicle_state`, `vip3_msgs`, `vip3_bringup` | 상태·기어, 팀 메시지, 연결/실행 조합 | 기존 이름과 토픽 유지. bringup에는 알고리즘을 넣지 않음 |
| `morai_msgs` | MORAI 26.R1 submodule | 패키지·메시지 이름과 pin `4c9be6f` 유지. 외부 코드에 팀 명명 규칙을 적용하지 않음 |

프로파일의 오래된 주석도 정리 대상이다. 예를 들어 `bag_profile.py`의 “rosbridge에는 source timestamp가 없다”는 설명은 현재 SVM의 헤더 시각 사용과 맞지 않는다. 주석만 믿고 기록 시각을 바꾸지 말고, 실제 메시지·bag의 source/receive 시각을 각각 확인해 계약을 갱신한다.

## 목표 구조

ROS 패키지는 초보 팀원이 찾기 쉽도록 `src/` 바로 아래에 둔다. **예정 모듈의 빈 패키지는 미리 만들지 않는다.**

```text
src/
├── morai_msgs/             # 외부 submodule
├── vip3_msgs/              # 팀 내부 메시지·서비스
├── vip3_bringup/           # bridge·launch·환경 진단
├── vip3_vehicle_state/     # ego 상태와 기어 서비스
├── vip3_hd_map/            # MGeo·제공 GT·좌표
├── vip3_svm/               # 4어안 입력·시간 정렬·지면 투영·합성
├── vip3_data/              # bag 수집·감사·재생 (data_collection의 최종 이름)
├── vip3_eval/              # 평가 라이브러리; 현 함수부터 정리
├── vip3_slot_detection/    # 예정: SS-PSD adapter·출력 좌표 변환
└── vip3_parking/           # 예정: GT/인지 목표 선택·경로·추종·상태기
```

`vip3_data`로의 이름 변경은 MVP 이후로 미뤄도 된다. 현재 `data_collection` 이름보다 서로 다른 목적의 코드가 한꺼번에 실행·의존되는 것이 먼저 해결할 문제다. `vip3_parking`은 초기에는 하나의 패키지 안에서 planner/controller 모듈을 분리하고, 작은 프로젝트에 ROS 패키지를 과도하게 늘리지 않는다.

새 `vip3_svm`의 Python 모듈은 `input`, `pose`, `projection`, `calibration`, `vehicle_geometry`, `visualization`처럼 역할이 드러나게 나눈다. ROS callback·Publisher는 `scripts/svm_node.py`, 순수 계산은 `src/vip3_svm/`, 검증은 `test/`, launch는 `svm.launch`에 둔다. 상위 이름이 SVM이므로 모든 파일에 다시 `fisheye_svm_`을 붙일 필요는 없다.

## 이동 순서와 완료 기준

팀의 주 개발 시간은 3명×3시간×5주=45인시다. 이름 변경만으로 한 주를 쓰지 않도록 **필수 정리 3–5인시를 우선 상한**으로 잡고, 깨진 의존성이 많으면 선택 정리를 연기한다. 아래 시간은 코드 확인 후 잡은 작업 예상치이며 실측 일정은 아니다.

| 순서 | 변경 | 예상·주 담당 | 완료 기준 |
|---|---|---|---|
| 0 / 이번 작업 | 기존 경로에서 SVM 연산·입력 대기·표시 비용 최적화, 단계별 지연 측정 추가 | 구현 완료 후 인수인계: 하수영·안승현 | 같은 보정/입력의 출력 일치, 이동 자세·중단 처리 검사, 재현 가능한 전후 성능 보고 |
| 1 / 다음 작은 PR | 어안 SVM만 `vip3_svm`으로 추출. 핀홀 GroundPlane 의존도 분리 | 2–3인시 / 하수영, 설정 확인 안승현 | 보정 YAML·차량 에셋·테스트·launch 함께 이동, 새로운 build/devel 경로에서 import/실행, 기존 녹화 재생 동일 |
| 2 / 의존성 분리 직후 | 미사용 TwinLite/핀홀·semantic Capture 경로와 전용 scripts/config 정리 | 1–2인시 / 해당 코드 담당, 김동현 수집 확인 | `rg`·정적 import·launch 검사에서 참조 0, 기본 이미지에 학습 라이브러리 불필요, parking_raw 수집·재생 통과 |
| 3 / 실제 기능 개발과 함께 | SS-PSD를 `vip3_slot_detection`, 경로·추종을 `vip3_parking`에 구현 | 기존 기능 일정 / 하수영·강도균 | 기존 topics/frames 계약 연결, GT 목표와 인지 목표 출처 구분, `/ctrl_cmd` 단일 발행자 |
| 4 / 선택, MVP 이후 | `data_collection`→`vip3_data`, eval packaging, 남은 CLI 명칭 통일 | 1–2인시 / 김동현·강도균 | 외부 소비자·Python import·catkin 설정·Docker·문서 동시 갱신. 실행되는 구버전 복사본을 두지 않음 |

1번 PR에서 기존 `roslaunch drivable_bev fisheye_svm.launch`가 새 launch를 include하는 얇은 호환 경로를 둘 수 있다. 팀원이 새 명령으로 전환하고 공통 bag 재생을 확인한 다음 제거한다. 호환 경로에 알고리즘 사본을 만들지 않는다. 내부 Python import는 같은 PR에서 갱신한다.

## 반드시 같이 옮길 항목

- ROS 패키지 이름, `package.xml`, `CMakeLists.txt`, `setup.py`, Python import, node 설치 목록, launch의 `find/pkg/type`.
- SVM config·`config/vehicles`·캘리브레이션 증거. 센서 JSON의 날짜·MORAI 버전 명명 규칙과 원본 장착값은 유지한다.
- `tools/check_sensor_preset.py`, 보정·replay·benchmark 도구, Docker build validation, CI의 경로.
- `config/README.md`, 패키지 README, [commands.md](../commands.md), [오너십](team.md). 기존 설명은 한 곳으로 통합한다.
- 이전 `devel`의 Python wrapper가 누락된 파일을 가려주지 않는지 새 build/devel 경로에서 확인한다.

`parking_space_set.json`은 데이터 원본으로 보존한다. 제거할 레거시는 Git 이력에 남기고 현재 `src/legacy`에 실행 가능한 전체 복사본을 쌓지 않는다. upstream 고지·라이선스는 실제로 남겨 쓰는 코드와 함께 유지한다.

## 실행 경계

현재 단계에서는 ROS 토픽·frame 이름을 바꾸지 않는다. 센서 연결 담당과 SVM/제어 담당을 동시에 막는 이름 변경을 피하기 위해 패키지 이동과 메시지 변경을 다른 PR로 나눈다.

- 김동현: 연결·bag·입력 시각. 하수영: SVM·SS-PSD·픽셀→미터. 강도균: 목표→경로·제어·종료 조건.
- 안승현: MORAI·센서·차량 좌표와 변경의 통합 확인. 장원태: 최종 구조·검증 결과를 발표에 반영.
- 매 PR에 이전→새 경로, 실행 명령 변화, 검증 입력, 완료 여부를 적는다. 기능이 없는 모듈은 구현 완료로 표시하지 않는다.

성능 개선의 측정 범위와 실행법은 [SVM 성능](../system/svm-performance.md)에 둔다. 리팩토링 과정에서도 보정값·512² 출력·10m 범위·차량 마스크·프레임 시각을 회귀 기준으로 유지한다.
