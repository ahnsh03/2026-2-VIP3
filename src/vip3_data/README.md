# vip3_data

담당 **김동현**. 원본 카메라·ego·TF를 기록하고 같은 입력을 재생한다.
[실행 명령](../../docs/commands.md#collection) · [데이터와 평가](../../docs/system/data.md)

| 진입점 | 용도 |
|---|---|
| `record.launch` | allowlist bag 기록. `scenario`·`controller` 필수, raw run 덮어쓰기 거부 |
| `replay.launch` | `/use_sim_time`, 준비 후 `/vip3_bag_replay/start`로 시작, 평가 GT 명시 허용 |
| `snapshot.launch` | 최신 메시지를 주기적으로 저장하는 간단한 연결·영상 확인 |
| `capture.launch` | MORAI Capture Mode 트리거. `/SaveSensorData`를 `capture_hz` 주기로 발행 |
| `scripts/check_capture.py` | 저장된 PNG 검사(프레임당 12파일, 1280×960). `--dry-run`은 검사 전용 |
| `scripts/audit_bag.py` | 수신율·header 시각의 단조성·채널별 시간 통계 검사 |
| `scripts/reject_bag.py` | 불량 run을 사유와 함께 rejected로 분류 |

`parking_raw`는 4 × 1280×960 JPEG와 `/Ego_topic`을 필수로 기록한다. TF·명령·gear는 시작할 때 발행 중이면 추가된다. 현재 LiDAR/GPS/IMU는 없다. `/Ego_topic`은 시뮬 GT 자세이며, object/collision 등 평가 전용 채널을 구분하는 `simulator_gt` 플래그와 의미가 다르다.

`parking_gt`는 object/collision을 추가할 수 있다. 평가 채널을 인지 모델 입력으로 사용하지 않는다. 원본은 `$VIP3_DATA/rosbags/raw/<run_id>/`, bag은 하위 `bags/`에 저장한다. bag 기록 시각은 수신 시각이며 메시지 내부 source stamp·JPEG는 그대로 보존한다. `metadata.yaml`에는 수신 통계·bag SHA·Git 상태·프리셋 경로를 기록한다. 센서/네트워크 파일과 SVM/차량 YAML의 원본은 자동 복사하지 않으므로 별도 보관한다.

`config/bag_profiles/`는 기록·재생 계약, `config/snapshot_profiles/`는 `dry_run`(ego만)·`parking_core`(4카메라+ego+평가 채널) 설정이다. snapshot은 정밀 시간 동기화나 GCN 학습 라벨을 보장하지 않는다. `/data/datasets/<run_id>/`에 저장하며 기존 run ID 재사용을 막지 않으므로 새 ID를 사용한다. 원본 bag → 시간 정렬 → 보정 BEV → 직접 point/slot 라벨 변환은 추가 구현 대상이며, 학습 split은 run 단위로 나눈다.

ROS 입출력은 `scripts/`, 프로파일·메타데이터·저장 로직은 `src/vip3_data/`, 검사는 `test/`에 둔다. 구형 Capture·semantic-mask 변환 경로는 제거했다.

## Capture Mode 수집

센서셋 `config/morai/261110_VIP3_Sensor_26.R1.H3.json`의 어안 카메라 4대(CAMERA_1 front, 2 rear, 3 left, 4 right, 1280×960)를 쓴다. 카메라마다 Active List의 **Sensor Capture Mode** 버튼을 켜야 3종이 나온다.

```bash
roslaunch vip3_data capture.launch capture_hz:=0.5 run_id:=park_001
python3 src/vip3_data/scripts/check_capture.py --run-id park_001 --dry-run
```

`/SaveSensorData`(`is_custom_file_name: true`, `file_dir: ''`)를 발행하면 MORAI가 Windows의 `C:\MoraiLauncher_Win\MoraiLauncher_Win\SensorData\CAMERA_<ID>\<이름>_{Intensity,Semantic,Instance}.png`에 저장한다(WSL: `/mnt/c/MoraiLauncher_Win/MoraiLauncher_Win/SensorData`, 4×3=12파일, Depth 없음). 경로는 `$VIP3_MORAI_SENSOR_ROOT`나 `--sensor-root`로 바꾼다. 스페이스바 수동 캡처는 `...\MoraiLauncher_Win_Data\SaveFile\SensorData`에 UTC 시각 이름으로 저장되므로 이 경로와 무관하다.

`$VIP3_DATA/capture_runs/<run_id>/`에는 `meta.json`·`capture_manifest.jsonl`·`summary.json`만 남는다. manifest의 `success=true`는 **publish가 리턴했다는 뜻일 뿐 저장 확인이 아니다.** MORAI는 저장 완료 응답을 주지 않으므로 `check_capture.py`가 필수 게이트이며, 누락·빈 파일·잘못된 크기가 있으면 종료 코드 1을 반환한다. 실측: 1.0Hz에서는 10프레임 중 3프레임이 12파일 전부 저장되지 않았고(MORAI가 저장 중 요청을 버리는 것으로 추정), 0.5Hz는 10/10이었다. 기본값은 0.5Hz다. 주기별 실측(각 10~20프레임): 0.5Hz 20/20(240파일), 0.7Hz 9/10(8번 누락), 1.0Hz 7/10(2·5·8번 누락)이므로 안전 한계는 0.5Hz 부근이다(0.5~0.7 사이는 미측정). 저장이 요청보다 0.2~0.8초 늦으므로 종료 직후 검사하면 마지막 프레임이 누락처럼 보일 수 있다(재검사하면 정상). 수집 중에 검사하면 마지막 프레임이 쓰는 중일 수 있어 `--skip-last 1`을 쓴다. 해상도는 기본 1280×960이며 640×480 센서셋(261115)이면 `capture.launch image_width:=640 image_height:=480`로 run에 기록하고, `check_capture.py`는 이를 따른다(`--width/--height`로 덮어쓰기). 테스트: `PYTHONPATH=src/vip3_data/src python3 -m unittest discover -s src/vip3_data/test -p test_capture.py`.
