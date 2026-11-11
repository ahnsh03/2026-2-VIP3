# SVM 보정과 검증

현재 **영상 시각에 맞춘 차량 자세·카메라 간 촬영 시차 보정**은 구현됐다. 렌즈 K/D와 설치 오프셋을 지면 대응점으로 보정하는 도구도 제공한다. **실측 렌즈·절대 주차 좌표 보정 완료를 뜻하지 않는다.** 초기 K/D를 교체한 후보는 아직 없다.

## 적용한 방식

| 참고 | 채택한 부분 | 적용 범위 |
|---|---|---|
| [2026-ASMC](https://github.com/INHAautonav/2026-ASMC), `76e25d3`의 `calibration.py`, `map_lane_calibration.py` | 지면 평면·좌표 변환, m 단위 잔차와 강건한 추정 | 기존 핀홀 homography를 179° 어안에 직접 쓰지 않고 어안 광선으로 역투영 |
| [Click-Calib (2025)](https://arxiv.org/abs/2501.01557), [저자 코드](https://github.com/lwangvaleo/click_calib) | 지면 대응점으로 카메라 정합을 검증·최적화하는 접근 | 이 구현은 측정된 지면 좌표를 사용하고 카메라 위치/높이를 고정한 제한적 변형이다. 논문 재현 전체는 아님 |
| [SurroundCameraCalib](https://github.com/OpenCalib/SurroundCameraCalib), `c0295a8` | 겹침 영역·지면만 대상으로 보정 | 초기 K/D·외부 파라미터가 필요하다. 벽·기둥·차체를 대응점으로 사용하지 않음 |
| [OpenCV fisheye](https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html) | θ 다항식 모델 | K와 D 4항을 지원. 단조성 검사와 역투영, horizon/잘못된 광선 거부 |

수학 코드는 독립 구현했다. 실행에 external 체크아웃이나 해당 논문의 학습 모델은 필요 없다.

## 런타임 자세와 시간

- `/tf`의 `map → base_link` 자세를 영상 헤더 시각에 보간한다. 네 카메라 모두 허용 시각차 50ms, 수신 age 0.5초 안에서 자세를 찾아야 한다.
- 가장 늦은 영상의 base_link를 출력 기준으로 삼는다. 각 카메라 촬영 시점의 상대 회전·이동을 투영에 반영한다. 시각이 0이면 보정하지 않는다.
- 지면은 `normal_map`으로 정한 평면, 차축과 지면의 법선 거리는 0.35m로 가정한다. **기본 `[0,0,1]`은 수평 지면 가정이다. 차체 기울기만으로 도로 경사를 알아낼 수 없다.** 경사면은 별도 측정한 법선으로 설정해야 한다. 주행 중 높이 변화·비평면 노면·벽/기둥은 이 모델로 복원하지 못한다.
- 자세가 없거나 오래되면 초기 정적 투영으로 돌아가고 화면·`/perception/svm/status`에 이유를 표시한다. 네 영상 중 하나라도 끊기면 preview와 panel에 대기를 표시하고 valid mask를 0으로 발행한다.
- 차량 그림과 차체 마스크 크기는 보정값에 따라 늘리지 않는다. valid mask는 지면 투영 가능 영역이며 장애물/주차 가능 영역을 뜻하지 않는다.

### MORAI 26.R1.H3 quaternion 확인

이번 라이브 TF의 후방·좌우 카메라 회전은 UI의 nominal mount와 각각 약 70°·62.8° 차이가 났다. quaternion **x 성분만 부호 전환**하면 네 카메라 모두 0.000008° 이내로 일치했다. [측정값](../../src/perception/drivable_bev/config/calibration/261111_validation.json)에 원본 TF와 비교 결과를 남겼다.

현재 SVM 설정은 `quaternion_convention: morai_26r1_x_flipped` 어댑터를 적용한다. 같은 MORAI 직렬화 경로를 사용하는 ego TF에도 적용하고, 설치 맵의 주차선 투영으로 대조했다. `/tf` 자체를 수정하거나 재발행하지 않는다. **일반 ROS·다른 simulator/bridge에서는 `ros_xyzw`를 사용하고 다시 확인한다.** MORAI 전체 버전에 대한 공식 사양으로 일반화하지 않는다.

## 렌즈·카메라 정합 추정

`tools/calibrate_fisheye_svm.py`는 네 카메라의 공유 focal scale, k1/k2와 각 카메라의 작은 Roll/Pitch/Yaw 오프셋을 함께 추정한다. principal point, k3/k4, 위치·높이는 고정한다. nominal `rotation_deg`와 `translation_m`은 유지하고 설치 보정만 `correction`에 저장한다.

관측 JSON의 `observations` 배열에는 다음 형식의 점을 넣는다. 숫자는 **형식 예시**이며 실측값으로 바꿔야 한다.

```json
{
  "observations": [
    {
      "point_id": "capture01_corner_A",
      "view": "front",
      "pixel": [640.0, 600.0],
      "ground_xy": [4.5, 0.0],
      "plane": {"z_at_origin_m": -0.35, "dz_dx": 0.0, "dz_dy": 0.0},
      "split": "train"
    }
  ]
}
```

`pixel`은 원본 영상의 `[u,v]`, `ground_xy`와 `plane`은 **그 촬영 시각의 base_link** 기준이다. 실제 페인트 경계/교점을 측정한다. GT 주차칸 폴리곤의 선 중심과 페인트 바깥 모서리를 혼동하지 않는다. GT와 설치 맵의 좌표·버전 일치도 먼저 확인한다. 가능하면 여러 정차 위치에서 수집한다.

카메라마다 2차원으로 분포한 train ≥8점, validation ≥4점이 필요하다. 같은 물리 점의 `point_id`를 양쪽 split에 재사용할 수 없다. soft-L1 손실과 작은 변화 범위를 사용하며 다음 조건을 모두 만족해야 새 YAML을 저장한다.

- optimizer 수렴, 파라미터 제한값 미도달, 렌즈 단조성 유지
- 검증 XY RMSE ≤0.15m 및 초기값 대비 20% 이상 개선
- 네 카메라 각각 검증 RMSE가 나빠지지 않음

이 기준은 **후보 파일 생성 조건**이다. 실제 배포 전에는 다른 차량 위치·각도에서 별도 검증하고 주차 목표 오차에 맞춰 더 엄격한 기준을 정한다. 원본 설정은 도구가 덮어쓰지 않는다. 실패하면 report만 남기고 종료 코드 2를 반환한다. [실행 명령](../commands.md#svm-calibration).

## 2026-11-11 검증 결과와 남은 일

라이브 카메라 4대 약 9.9–10.1Hz, 영상 시각차 34ms에서 자세 보정을 적용했다. 저장된 투영 상태로 재생한 결과가 원래 `svm_projection.png`와 픽셀 단위로 같았다. 수학 회귀 테스트와 catkin 빌드를 통과했으며 GitHub Actions에서도 기하·설정 검사를 실행한다.

실측 렌즈 후보는 후방·좌·우 지면의 설치 맵 페인트 경계를 사용했다. 검증 경계 156표본의 픽셀 RMSE는 5.15→4.93px였지만 k2가 제한값에 닿았고 개선도 작아 **기각했다**. 전방은 해당 위치에서 차체가 가까운 페인트를 가렸다. 따라서 초기 K의 f≈409.7128px, D=0을 유지한다. 이 픽셀 잔차는 주차칸 검출/좌표 정확도 지표가 아니다.

다음 보정 데이터는 벽과 기둥 없이 네 카메라에 지면 교점이 보이는 위치에서 수집한다. 지면 경사와 렌즈 오차를 함께 섞어 맞추지 않도록 지면 기준을 측정하고, 학습용/검증용 위치를 나눠 실제 m 단위 오차를 확인해야 한다. 그 전까지 SS-PSD 학습·평가용 clean BEV로 승인된 상태는 아니다.
