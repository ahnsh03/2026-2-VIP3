# vip3_data

담당 **김동현**. 원본 카메라·ego·TF를 기록하고 같은 입력을 재생한다.
[실행 명령](../../docs/commands.md#collection) · [데이터와 평가](../../docs/system/data.md)

| 진입점 | 용도 |
|---|---|
| `record.launch` | allowlist bag 기록. `scenario`·`controller` 필수, raw run 덮어쓰기 거부 |
| `replay.launch` | `/use_sim_time`, 준비 후 `/vip3_bag_replay/start`로 시작, 평가 GT 명시 허용 |
| `snapshot.launch` | 최신 메시지를 주기적으로 저장하는 간단한 연결·영상 확인 |
| `scripts/audit_bag.py` | 수신율·header 시각의 단조성·채널별 시간 통계 검사 |
| `scripts/reject_bag.py` | 불량 run을 사유와 함께 rejected로 분류 |

`parking_raw`는 4 × 1280×960 JPEG와 `/Ego_topic`을 필수로 기록한다. TF·명령·gear는 시작할 때 발행 중이면 추가된다. 현재 LiDAR/GPS/IMU는 없다. `/Ego_topic`은 시뮬 GT 자세이며, object/collision 등 평가 전용 채널을 구분하는 `simulator_gt` 플래그와 의미가 다르다.

`parking_gt`는 object/collision을 추가할 수 있다. 평가 채널을 인지 모델 입력으로 사용하지 않는다. 원본은 `$VIP3_DATA/rosbags/raw/<run_id>/`, bag은 하위 `bags/`에 저장한다. bag 기록 시각은 수신 시각이며 메시지 내부 source stamp·JPEG는 그대로 보존한다.

`config/bag_profiles/`는 기록·재생 계약, `config/snapshot_profiles/`는 `dry_run`(ego만)·`parking_core`(4카메라+ego+평가 채널) 설정이다. snapshot은 정밀 시간 동기화나 SS-PSD 학습 라벨을 보장하지 않는다. 원본 bag → 시간 정렬 → 보정 BEV → 직접 point/slot 라벨 변환은 추가 구현 대상이며, 학습 split은 run 단위로 나눈다.

ROS 입출력은 `scripts/`, 프로파일·메타데이터·저장 로직은 `src/vip3_data/`, 검사는 `test/`에 둔다. 구형 Capture·semantic-mask 변환 경로는 제거했다.
