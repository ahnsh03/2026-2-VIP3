# 현재 센서와 SVM

## 실행 정본

- MORAI Import: [센서](../../config/morai/261110_VIP3_Sensor_26.R1.H3.json), [네트워크](../../config/morai/261110_VIP3_Network_26.R1.H3.json)
- 토픽·프레임 계약: [topics.yaml](../../config/ros/topics.yaml)
- 합성 설정: [svm.yaml](../../src/vip3_perception/config/svm.yaml)
- 차량 치수·지면: [2023_hyundai_ioniq5.yaml](../../src/vip3_perception/config/vehicles/2023_hyundai_ioniq5.yaml)
- 차량 그림: [ioniq5_top_view.png](../../src/vip3_perception/assets/ioniq5_top_view.png)

프리셋 이름은 **`YYMMDD_VIP3_Sensor/Network_모라이버전.json`**을 따른다. `261110`은 현재 export 날짜다. 새 버전으로 바꾸면 참조 경로도 갱신하고 이전 실행 프리셋은 Git 이력에만 남긴다. 팀원은 공유 Sensor/Network JSON을 사용하며 접속 주소는 `127.0.0.1:9090`으로 통일한다.

## 카메라 배치

공통: **FishEyeCamera, 1280×960, Horizontal FOV 179°, 10Hz, JPEG 품질 90**. 현재 센서셋에는 LiDAR·GPS·IMU가 없다. 아래 위치는 지면이 아니라 `base_link` 기준이며, 회전은 MORAI UI의 Roll/Pitch/Yaw(deg)다.

| ID / view | x, y, z (m) | Roll, Pitch, Yaw | frame |
|---|---|---|---|
| 1 front | 3.85, 0, 0.30 | 0, 35, 0 | `cam_front` |
| 2 rear | −0.80, 0, 0.30 | 0, 35, 180 | `cam_rear` |
| 3 left | 2.10, 1.05, 0.70 | 0, 45, 90 | `cam_left` |
| 4 right | 2.10, −1.05, 0.70 | 0, 45, 270 | `cam_right` |

2026-11-12 실제 네 영상의 JPEG 디코딩·해상도와 약 10Hz 수신을 확인했다. 이는 통신 검사이며 모든 방향의 지면 관측 검증은 아니다. [측정 기록](../notes/261112-integration.md).

각 입력은 `/cam_<view>/image_jpeg/compressed`다. 카메라 수만으로 차량 바로 옆 지면의 관측을 보장할 수 없으므로 네 모서리, 차체 옆 선, 겹침 구간을 확인한다.

## 차량 치수와 검증 범위

| 항목 | 적용 값 |
|---|---:|
| 전장 / 차체 폭 | 4.635 / 1.892m |
| 휠베이스 / 전방·후방 오버행 | 3.000 / 0.845·0.790m |
| 차체 x 범위 / y 범위 | [−0.790, 3.845] / ±0.946m |
| 차량 중심 x | 1.5275m |
| base_link의 지면 위 높이 | 0.35m; 현재 지면 z=−0.35m |
| 최대 휠 각 / 제원 최소 회전 반경 | 40° / 5.87m |

2026-11-11 설치된 `hyundai_ioniq5.vehicle`(v.R1.260701.H3)와 K-City 2025 collision mesh를 읽고, 정차 TF 222표본의 위치와 지면을 대조했다. 그 장소의 높이는 약 **0.3513m**여서 0.35m를 반올림 기준으로 사용한다. **휠 반경 0.36m와는 다른 값**이며, 다른 위치의 경사·주행 중 차체 높이까지 검증한 값은 아니다.

제원표 높이 2.434m와 설치 에셋 bbox 높이 약 1.644m는 불일치한다. 센서 높이에 전고를 사용하지 않는다. 에셋 전체 bbox 폭 약 2.161m에는 거울 등 돌출부가 포함되므로 차체 마스크 폭과 구분한다. 최소 회전 반경의 측정 기준은 명확하지 않아 후륜축 자전거 모델 반경과 그대로 동일시하지 않는다.

원자료 출처는 차량 YAML에 남겼다. 상세 로컬 측정 보고서는 `docs/experiments/261110-261111-svm/analysis/vehicle-geometry-261111.json`, 공유 이력은 정리 이전 커밋 `a5d1665`의 `docs/analysis/vehicle-geometry-261111.json`에서 확인할 수 있다.

## SVM 상태와 다음 보정

현재 출력은 **10×10m, 512×512, 목표 10Hz**다. 차량 중심 x=1.5275m를 영상 중심에 둔다. 픽셀 중심 `(row, col)`의 차량 좌표는 다음과 같다.

```text
x = 1.5275 + 5 − (row + 0.5) × 10 / 512
y =          5 − (col + 0.5) × 10 / 512
```

렌즈는 OpenCV fisheye 모델이며 현재 `261112_kcity_ego_ground_v2`의 공유 K/D를 사용한다. 이전의 카메라별 각도 correction은 제거하고 원본 mount를 유지했다. base_link 높이는 0.35m다. 가정 지면에는 페인트 오프셋 +0.01m를 적용하며, 영상 근거가 충분하면 주변 지면을 별도 추정한다. 다섯 저장 장면에서 비교한 제한적 preview로, 일부 장면은 v1보다 오차가 늘었다. 차체 마스크와 차량 그림은 시각화용이며 크기를 바꿔 정합을 맞추지 않는다. 그림·마스크를 꺼도 valid mask의 관측 불가 영역은 유지한다.

차량 그림은 물리 치수와 후륜축 기준으로 배치하고 픽셀 반올림 오차를 줄였다. 이전 전방 범퍼 배치 보정은 그림 정렬에 대한 수정이며, 시뮬 주차선과 SVM 지면 좌표의 정합 완료를 뜻하지 않는다.

자세·영상 시차 보정과 상태 토픽은 적용됐다. MORAI TF quaternion 변환, 지면 가정, 가장자리 재보정 수치와 남은 범위는 [SVM 보정과 검증](svm-calibration.md)에 기록했다. 다른 위치의 절대 좌표 정합 검증과 학습용 clean BEV 출력은 아직 남아 있다.

1. [마스크·그림 해제 명령](../commands.md#svm)으로 가까운 선과 카메라 겹침을 확인한다.
2. 지면 대응점을 별도 train/validation으로 수집해 [보정 명령](../commands.md#svm-calibration)을 실행한다.
3. 다른 차량 위치에서도 m 단위 오차를 확인한 뒤 SS-PSD용 clean BEV로 연결한다.
