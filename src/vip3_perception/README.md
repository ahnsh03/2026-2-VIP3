# vip3_perception

담당 **하수영**. 어안 4카메라 SVM과 이후 SS-PSD 주차칸 인지를 담는다. 현재 실행 가능한 노드는 **SVM만** 있다. 검출은 별도 노드로 추가하며 PyTorch가 SVM의 필수 의존성이 되지 않게 한다.

[실행 명령](../../docs/commands.md#svm) · [구조·확장 위치](../../docs/project/src-refactoring.md) · [보정 근거](../../docs/system/svm-calibration.md)

- 입력: `/cam_{front,rear,left,right}/image_jpeg/compressed`, 자세용 `/tf`의 `map → base_link`.
- 출력: `/perception/svm/preview/compressed`, `panel`, `panel/compressed`, `valid_mask`, `status`.
- 현재 출력은 진단용이다. clean BEV 토픽·SS-PSD 어댑터·학습은 미구현이다.
- 원본 JPEG는 콜백에서 버퍼링하고, 단일 작업자가 선택한 묶음만 디코딩한다. 기본 출력 상한 10Hz, 입력 누락/과거 프레임은 거부한다.
- `roslaunch vip3_perception svm_unsynced.launch`는 640×480 입력 설정(`svm_640x480.yaml`)을 기본으로 사용하며, timestamp 시차와 입력 age 검사를 건너뛰고 각 카메라의 마지막 프레임을 합성한다. 입력을 못 받은 카메라는 `NO FRAME` 검은 화면으로 대체한다. MORAI에는 `config/morai/261115_VIP3_Sensor_26.R1.H3_640x480.json`을 Import한다. 기본 SVM과 출력 토픽이 같아 동시에 실행하지 않는다.
- 렌즈·지면·자세·시차 계산은 ROS 없는 `src/vip3_perception/svm/`, 픽셀↔차량 좌표는 `coordinates.py`, ROS 연결은 `scripts/svm_node.py`에 둔다.
- `config/svm.yaml`과 `config/vehicles/`가 정본이다. 그림은 `assets/`, 오프라인 도구는 저장소 `tools/svm/`에 있다.

캘리브레이션은 일부 정차 자세에서 검증한 preview다. 여러 위치·선회·경사에서 추가 확인해야 하며 차량 그림과 마스크 해제로 보이지 않는 지면을 복원할 수는 없다.
