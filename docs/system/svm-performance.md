# SVM 지연과 연산 성능

아래 비교 표는 **2026-11-11 최적화 당시**의 기록이다. 현행 프로필은 `261112_kcity_ego_ground_v2`이며 새 지면 추정 비용은 별도로 측정했다.

2026-11-11 최적화. 캘리브레이션 프로필 `261111_kcity_1280x960_v1`, 4×1280×960 입력, 512²·10m 출력을 유지한다. 기본 출력 상한은 센서와 같은 **10Hz**다. GPU·CUDA·추가 학습은 사용하지 않는다.

## 바뀐 처리

- 지면과 카메라 변환을 X/Y의 affine 식으로 합쳐 계산한다. 매 카메라의 큰 XYZ 배열·행렬곱·중간 배열을 줄였고, 모든 출력 픽셀을 double precision으로 계산한다. 격자를 성기게 만들거나 자세를 반올림해 재사용하지 않는다.
- ROS callback은 JPEG를 압축 상태로 버퍼에 넣는다. 동기화 조건으로 선택된 네 장만 디코딩하며, 같은 프레임을 다시 선택하면 마지막 디코딩 결과를 재사용한다. 카메라마다 최대 JPEG 10장과 디코딩 결과 1장으로 제한한다.
- 고정 주기 Timer 대신 새 입력이 들어오면 단일 처리 worker를 깨운다. 출력 상한과 단조 시계 기준의 주기 마감을 유지해 실행 지터가 계속 쌓이지 않게 한다. 처리 중 도착한 입력은 버퍼에서 최신 묶음으로 다시 선택한다.
- 구독자가 없는 preview/panel의 장식·resize·JPEG 인코딩을 생략한다. `save_dir`가 있으면 구독자 없이도 필요한 결과를 한 번 저장한다. mask 해제 진단도 저장할 때만 masked/unmasked 두 투영을 만든다.
- JPEG 오류·해상도 오류, 네 입력 중단, 시각 역행, 처리 중 입력 만료를 검사한다. 잘못된 입력에서 이전 영상을 정상 결과로 다시 내보내지 않고 valid mask를 0으로 발행한다. TF만 없으면 기존 정적 fallback과 그 이유를 유지한다.

## 측정 결과

동일 PC의 `vip3-parking-ros` CPU, Python 3.8.10 / OpenCV 4.2.0 / NumPy 1.17.4, 컨테이너 가용 logical CPU 20개에서 측정했다. 비교 기준 코드는 `460a0f5`다. **MORAI 실시간 전체 지연이나 팀원 노트북의 성능을 측정한 값은 아니다.**

| 검사 | 기존 | 최적화 후 |
|---|---:|---:|
| 투영표+합성 중앙값, 50프레임·변하는 자세 | 96.2ms | 53.7ms |
| 같은 연산 95백분위 | 103.0ms | 56.8ms |
| ROS 출력률, 10Hz JPEG 입력 | 5.00Hz | 10.00Hz |
| ROS 입력 publish→raw panel 수신 중앙값 | 194.0ms | 80.9ms |
| 같은 ROS 경로 95백분위 | 204.3ms | 94.7ms |

코어 벤치마크는 양쪽 OpenCV thread 4개, 같은 보정·원본 영상에 변하는 자세/촬영 시차를 적용하고 실행 순서를 번갈아 측정했다. JPEG와 표시 비용은 제외한다. remap 좌표의 최대 차이는 **4.77×10⁻⁷px**, 비교한 출력 영상·valid mask·융합 weight는 동일했다. 자세 보정이나 렌즈 보정을 끄지 않았다.

ROS 검사는 별도 master에서 저장 영상을 JPEG 10Hz로 보내고 합성 TF를 공급했다. preview/compressed, panel/compressed, raw panel, mask, status를 모두 구독했다. 각 조건 10초 중 초기 2초를 제외해 기존 40프레임, 최적화 80프레임을 측정했다. 네 영상의 source stamp는 같은 값이므로 실제 MORAI의 카메라별 시차 분포를 대신하지 않는다. MORAI 렌더링·WebSocket·Windows/WSLg·rqt 표시 지연은 포함하지 않는다. 최적화 버전의 5Hz 설정도 검사했지만 입력과 처리 시점에 따라 오래된 묶음을 고를 수 있어 10Hz 기본값으로 비교했다.

검사 요약은 [성능 기록](../../src/vip3_perception/config/validation/261111_performance.json)에 있다. 상세 로그·원본·실험 스크립트는 관리자 로컬 `docs/experiments/261111-svm-performance/`, `data/sensor_checks/261111_svm_performance/`에 보관한다.

## 실행·지연 해석

기존 실행 명령으로 새 설정이 적용된다. 느린 PC에서 처리 시간이 지속적으로 100ms를 넘으면 출력률을 낮춰 비교할 수 있다.

```bash
roslaunch vip3_perception svm.launch
# 설정 파일을 수정하지 않는 대안
roslaunch vip3_perception svm.launch publish_hz:=5
rostopic echo /perception/svm/status
```

약 5초마다 decode / maps / warp / total / receive-to-publish 시간이 로그에 나온다. `/perception/svm/status`의 `timing`에는 다음 값이 있다.

| 필드 | 의미 |
|---|---|
| `input_age_ms` | 선택한 네 프레임 중 가장 오래된 프레임의 로컬 수신→선택 대기 |
| `decode_ms`, `pose_ms`, `maps_ms`, `warp_ms` | 선택 JPEG 디코딩, 자세 계산, 투영표 갱신, 합성 |
| `ground_refinement_ms` | 영상 지면 추정과 채택 시 추가 투영표 갱신. 기본 1Hz 검사 |
| `maps_rebuilt` | 동일 변환 재사용 여부. 자세 양자화는 하지 않음 |
| `display_ms`, `publish_ms` | 그림·패널 생성, 필요한 JPEG 인코딩과 publish 호출 |
| `processing_ms` | 선택부터 영상/마스크 publish 호출 완료까지 |
| `received_to_publish_ms` | 가장 오래된 입력의 로컬 수신부터 publish 호출 완료까지 |

`timing.clock`은 `local_monotonic_receive`다. 서로 다른 시계인 MORAI 헤더 시각과 PC 시각을 빼서 네트워크 지연이라고 표시하지 않는다. status 전송·GUI 화면 표시·`save_dir` 최초 PNG 파일 저장 시간은 위 처리 시간에 포함하지 않는다. 최초 저장은 일회성 디스크 지연을 일으킬 수 있다. 입력 무효 상태에서는 `timing: null`이다.

화면이 여전히 늦으면 입력 카메라 Hz, `input_age_ms`, `processing_ms`, raw panel 구독과 GUI 순서로 구간을 확인한다. 11/12 저녁 실제 MORAI 관측은 아래 후속 기록을 참고한다.

## 코어 성능 재현

컨테이너 `/root/ws`에서 실행한다. 기본값은 합성 영상이므로 개인 캡처 없이도 실행할 수 있다. 출력 파일은 기존 측정과 겹치지 않게 지정한다.

```bash
python3 tools/svm/benchmark.py --frames 50 \
  --output /data/svm_checks/benchmark_new.json
```

기존 코드와 같은 입력으로 수치와 처리 시간을 비교하려면:

```bash
git show 460a0f5:src/perception/drivable_bev/src/drivable_bev/fisheye_svm.py \
  > /tmp/vip3_svm_reference.py
python3 tools/svm/benchmark.py \
  --reference-module /tmp/vip3_svm_reference.py \
  --capture-dir /data/sensor_checks/261111_edge_calibration_result/after \
  --output /data/svm_checks/benchmark_comparison.json
```

위 capture 경로는 관리자 로컬 자료다. 팀원은 본인이 저장한 front/rear/left/right PNG 폴더를 지정하거나 `--capture-dir`를 생략한다. remap 0.001px, weight 10⁻⁵, mask 변경 0, 출력 채널 차이 1 이하의 허용 기준을 넘으면 종료 코드 2다. 이번 측정의 출력 채널 차이는 0이었다. 벤치마크는 CPU 계산 검사이며 캘리브레이션의 물리 정확도를 새로 입증하는 검사는 아니다.

2026-11-12 패키지 정리 후에도 단일 작업자·선택 JPEG 디코딩·투영표 재사용을 유지했다. 같은 캡처와 변화하는 자세를 이전 `a6f02f4` 코어와 비교해 투영표·가중치·mask·출력 픽셀이 모두 동일했다. 이번 변경은 추가 속도 개선을 주장하는 변경이 아니다. [현재 코드 구조](../project/src-refactoring.md)

## 2026-11-12 지면 추정 추가 후

새 프로필은 1초마다 지면을 검사한다. 실제 ROS 노드에 정차 JPEG·TF를 재생한 70프레임에서 `processing_ms` 중앙값 **26.2ms**, 95백분위 **55.9ms**였다. 정차 중에는 투영표를 대부분 재사용하므로 위의 변하는 자세 성능과 직접 비교하지 않는다. 새 프로필의 MORAI 실시간 전체 지연·팀원 노트북·주행 성능은 미측정이다. 지면 추정/추가 map 갱신은 `ground_refinement_ms`와 총 처리 시간에 포함된다. [측정 근거](../../src/vip3_perception/config/calibration/261112_validation.json)

## 2026-11-12 실제 MORAI 연결 후

동일 PC에서 1280×960 카메라 4개를 약 10Hz로 받고 rqt로 표시했다. 두 번째 25초 구간의 panel은 **8.84Hz**, 유효 220프레임의 처리 시간 중앙값 **89.7ms**, P95 **123.1ms**였다. 로컬 수신→publish는 중앙값 189.9ms/P95 329.9ms이며 렌더링·수신 전 네트워크·GUI 표시 시간은 포함하지 않는다. 상태 222개 중 2개는 처리 중 입력 만료였다.

정차 중에도 실제 TF가 미세하게 변해 유효 프레임 모두 투영표를 갱신했고, maps 중앙값은 59.7ms였다. 위 저장 재생의 26.2ms와 조건이 다르다. 10Hz 입력을 항상 10Hz로 처리하거나 모든 순간에 저지연을 유지한다고 주장하지 않는다. [시계열 요약·관측 조건](../notes/261112-svm-live.md)
