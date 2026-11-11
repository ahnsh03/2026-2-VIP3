# data_collection

담당 **김동현**. 카메라·ego·TF를 기록하고 같은 입력을 재생해 SVM·인지·제어 개발에 제공한다.
[실행 명령](../../docs/commands.md#collection) · [데이터와 평가](../../docs/system/data.md)

| 구현 | 용도 / 주의 |
|---|---|
| `bag_recorder.launch` | 현재 권장 수집. `scenario`·`controller` 필수, raw run 덮어쓰기 거부 |
| `bag_replay.launch` | `/use_sim_time`, 일시정지 후 start 서비스, GT 포함 bag 명시 허용 |
| `sync_collector.launch` | latest-timer snapshot. `parking_core`는 4어안+ego+objects/collision |
| `capture_collector.launch` | 기존 Capture 트리거. 현재 어안용 변환기는 별도 개발 필요 |
| `scripts/audit_rosbag_run.py` | 수신율·시각·카메라 데이터 점검 |

## 현재 수집 계약

`parking_raw`는 4 × 1280×960 JPEG와 `/Ego_topic`을 필수로 기록한다. TF·명령·gear는 시작할 때 발행 중이면 추가된다. 현재 LiDAR/GPS/IMU는 없다. `/Ego_topic` 자체는 시뮬 GT 자세이며, 코드상 `simulator_gt` 플래그와 동일한 의미가 아니다.

`parking_gt`는 object/collision을 추가할 수 있다. 평가 채널을 인지 모델 입력으로 사용하지 않는다. 원본 저장 위치는 `$VIP3_DATA/rosbags/raw/<run_id>/`, 실제 bag은 하위 `bags/`다.

source stamp, 수신 시각, 센서·보정 버전을 보존한다. raw bag → 시간 정렬 → 보정 BEV → 직접 point/slot 라벨의 마지막 변환·라벨 경로는 추가 구현 대상이다.

## 기존 Capture 경로의 범위

`capture_sync`는 구형 v1의 카메라 ID/해상도와 GPS/IMU 파일을 기대한다. 현재 카메라 ID는 **1 front, 2 rear, 3 left, 4 right**이므로 옛 Capture 도구를 그대로 적용하지 않는다. `/SaveSensorData` publish 완료는 MORAI 디스크 저장 완료를 보장하지 않아 동기화 도구의 `--dry-run` 파일 검사가 필요하다.

기존 semantic-mask 생성은 무손실 Semantic PNG와 팔레트를 사용한다. SS-PSD 방향 접점 라벨과는 다르다. 학습 split은 run 단위로 분리한다.

## 코드 위치

- `scripts/`: ROS 노드와 bag 감사 CLI.
- `src/data_collection/`: bag profile, 시간·파일·데이터 변환 코어.
- `config/bag_profiles/`, `config/profiles/`: 수집 계약. 현재 우선 프로파일은 `parking_raw`다.
- `test/`: 입력·프로파일·데이터 무결성 검사.
