# 2026-11-16 작업 노트 — 김동현 / 4어안 Capture Mode 수집

- 작성자 / 작업 시간: 김동현 / 2026-11-16
- 파트: 연결·수집
- 상태: 완료(정지 상태 캡처 한정). 차량 주행 중 캡처와 후속 변환은 미확인
- 코드: 브랜치 `donghyun/capture`, 커밋 `8769335` (`feat(data): 4어안 Capture Mode 수집과 검증 추가`). 이 노트와 문서 갱신은 이어지는 커밋

## 한 일

- 목표: 팀장이 올린 센서셋(261110)으로 MORAI 캡처모드를 써서 데이터를 수집하는 코드 연결
- 실제로 바꾼 것: 4어안 Capture Mode 수집 코드 추가. 캡처 노드(`capture_node.py`), launch(`capture.launch`), 저장 파일 검사 스크립트(`check_capture.py`), 테스트(`test_capture.py`, 14개)
- 노드는 `/SaveSensorData`를 `capture_hz` 주기로 발행한다. 제어 명령은 발행하지 않는다. 기본 `capture_hz`는 0.5Hz다.

## 재현 조건과 실행

- 실행 위치·선행 조건: 컨테이너 `vip3-parking-ros`에서 bridge(rosbridge) 실행, MORAI Play 상태, 카메라 4대 모두 Active List의 Sensor Capture Mode 켬. 검사는 호스트(WSL)에서 실행
- 입력: run ID `cap_test_02`~`cap_test_05`
- 설정: 센서셋 `config/morai/261110_VIP3_Sensor_26.R1.H3.json` (카메라 1 front, 2 rear, 3 left, 4 right, 각 1280×960). 640×480 센서셋(261115)은 `image_width`/`image_height`로 기대 해상도를 바꾼다. 이 해상도는 실행해 보지 않았다(미확인, 단위 테스트만).
- 실행 명령: [commands.md 4절](../commands.md#collection)

```bash
# 컨테이너
roslaunch vip3_data capture.launch capture_hz:=0.5 max_captures:=20 run_id:=cap_test_03
# 호스트 (종료 몇 초 뒤, 또는 --skip-last 1)
python3 src/vip3_data/scripts/check_capture.py --run-id cap_test_03 --dry-run
```

## 결과와 근거

실측한 저장 방식:

- ROS(`/SaveSensorData`) 캡처의 저장 위치는 `<MORAI>/MoraiLauncher_Win/SensorData/CAMERA_<ID>/<이름>_<종류>.png`이고, `custom_file_name`이 파일명에 반영된다. WSL 경로는 `/mnt/c/MoraiLauncher_Win/MoraiLauncher_Win/SensorData`다.
- 스페이스바 캡처는 `.../MoraiLauncher_Win_Data/SaveFile/SensorData`에 UTC 시각 이름으로 저장된다. ROS 캡처와 위치·이름 규칙이 다르다.
- 한 번에 카메라 4대 × Intensity/Semantic/Instance = 12파일이 저장된다. Depth는 나오지 않는다.
- 카메라마다 Sensor Capture Mode 버튼을 켜야 3종이 나온다.

주기 시험(정지 상태, 요청한 프레임 수 대비 12파일이 모두 저장된 프레임 수):

| 주기 | run | 결과 | 누락 프레임 |
|---|---|---|---|
| 0.5Hz | `cap_test_02` | 10/10 | 없음 |
| 0.5Hz | `cap_test_03` | 20/20 (240파일) | 없음 |
| 0.7Hz | `cap_test_04` | 9/10 | 8번 |
| 1.0Hz | `cap_test_05` | 7/10 | 2·5·8번 |
| 1.0Hz | `cap_test_01` (삭제) | 7/10 | 3프레임 |

- 누락된 프레임은 12파일이 전부 없었다. 저장은 요청보다 0.2~0.8초 늦게 끝난다. 저장 중 들어온 요청을 MORAI가 버리는 것으로 추정하지만 MORAI 내부 동작은 확인하지 않았다.
- 한계는 0.5~0.7Hz 사이이며 그 사이 값은 재지 않았다.
- 기본값은 0.5Hz다. 0.5Hz보다 빠르게 설정하면 노드가 경고를 낸다.
- 증거: 각 run의 `$VIP3_DATA/capture_runs/<run_id>/`(`meta.json`, `capture_manifest.jsonl`, `summary.json`)와 MORAI `SensorData/CAMERA_1~4/`의 PNG. 큰 자료는 보유자(김동현)의 로컬 PC에 있다.
- 남겨 둔 시험 실행: `cap_test_02`~`cap_test_05`는 주기 한계의 근거라서 남긴다. `cap_test_04`, `cap_test_05`는 프레임이 빠진 불완전한 실행이라 학습 데이터로는 쓰지 않는다. `cap_test_01`은 `cap_test_05`와 근거가 겹쳐 지웠다(`cap_dry_01`은 PNG 없는 dry-run이라 같이 지움).

## 막힌 점과 다음 작업

- **주의:** manifest의 `success=true`는 publish가 리턴했다는 뜻일 뿐 저장 확인이 아니다. MORAI가 저장 완료 응답을 주지 않으므로 `check_capture.py` 검사가 필수다. 종료 직후에는 마지막 프레임이 덜 쓰였을 수 있어(`cap_test_05`에서 확인) 몇 초 뒤 검사하거나 `--skip-last 1`을 쓴다.
- 미확인: 차량이 움직일 때의 캡처(모든 시험은 정지 상태), SVM BEV 변환과 라벨링 연결, 0.5~0.7Hz 사이의 한계값, 640×480 센서셋 실측.
- 다음 할 일 / 담당: 미정. 위 미확인 항목은 측정하기 전까지 결론으로 쓰지 않는다.
- 다른 파트에 전달할 변경: 없음. 기존 노드·토픽은 바꾸지 않았다.

## 발표에 사용할 내용

- 확인된 성과 한 문장: 센서셋 261110의 4어안 카메라로 0.5Hz 정지 상태 캡처를 30프레임(360파일) 연속 누락 없이 저장·검증했다.
- 사용할 표: 위 주기 시험 표. 조건은 정지 상태이고, 1.0Hz에서는 프레임이 통째로 빠진다는 한계를 함께 적는다.

## 확인 메모

- 안승현 — 확인 날짜·범위·추가 확인 사항: 미확인
- 장원태 — 자료 확인·발표 활용·추가 요청: 미확인
