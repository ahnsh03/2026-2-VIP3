# drivable_bev

담당 **하수영**, 차량·센서 기준은 **안승현**과 공동 확인.
[실행 명령](../../../docs/commands.md#svm) · [센서·보정 상태](../../../docs/system/sensors.md)

## 현재 4어안 SVM

`fisheye_svm.launch`는 `/cam_{view}/image_jpeg/compressed` 네 입력으로 10×10m·512×512·최대 10Hz 미리보기를 만든다. `261111_kcity_1280x960_v1`의 렌즈·각도 보정과 페인트 높이 +10mm를 기본 적용한다. 두 정차 자세에서 확인한 preview 프로필이다.

| 출력 | 형식 / 목적 |
|---|---|
| `/perception/svm/preview/compressed` | JPEG SVM + 경고·차량 그림 |
| `/perception/svm/panel`, `panel/compressed` | 원본 네 뷰 + 격자 SVM; Image/JPEG |
| `/perception/svm/valid_mask` | mono8, 투영에서 유효한 영역; 입력 중단 시 0 |
| `/perception/svm/status` | String/JSON, 입력 유효성·자세 보정·지면 가정·보정 버전 |

launch 인자는 `config`, `vehicle_config`, `vehicle_image`, `vehicle_overlay`, `vehicle_mask`, `save_dir`, 선택 `publish_hz`다. 생략하면 YAML 출력률을 사용한다. 마스크·그림 해제는 진단용이며 valid mask의 차체 영역은 여전히 무효다. 파일 저장은 첫 유효 영상 묶음 한 번만 수행한다.

`svm_input.py`가 선택 프레임의 JPEG 디코딩·재사용, `svm_pose.py`가 영상 시각의 자세 보간·MORAI quaternion 어댑터, `svm_calibration.py`가 지면 대응점 최적화, `fisheye_svm.py`가 영상 선택·투영, `vehicle_geometry.py`가 차체·지면 기준, `vehicle_overlay.py`가 차량 그림 배치를 맡는다. 표시용 그림·격자·문구와 인지 입력을 분리한다. **clean BEV ROS 출력과 SS-PSD 연결은 아직 없다.**

## 남아 있는 핀홀 코어

`rgb_bev.launch`, `camera_bev.launch`, `camera_bev_fusion.launch`, `camera_bev_visualizer.launch`, `four_view_bev.launch`는 이전 핀홀 RGB/semantic BEV 경로다. 별도 `camera_config`가 필수이며 투영 검증에는 `sensor_set_path`도 필요하다. 현재 179° 어안을 이 호모그래피에 직접 넣지 않는다.

semantic 입력은 `/perception/camera/{view}/{head}`이며 lane 또는 road_marking head를 투영·융합한다. 핀홀 `bev_grid.yaml`과 현재 `fisheye_svm.yaml`의 출력 범위를 혼동하지 않는다. 기존 geometry·fusion 테스트와 fixture는 수학 회귀 검사이고 현재 센서 보정 완료의 증거가 아니다.

자세 보정은 `/tf`를 사용하며 수평 지면·고정 차축 높이를 가정한다. [보정 절차와 한계](../../../docs/system/svm-calibration.md)를 먼저 확인한다.

선택 JPEG만 디코딩하고 입력 이벤트에 따라 처리한다. 구독자 없는 표시·인코딩은 생략한다. status의 `timing`과 약 5초 주기 로그로 로컬 대기·계산 시간을 확인한다. [측정 범위와 성능 재현](../../../docs/system/svm-performance.md).
