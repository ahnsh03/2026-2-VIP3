# vip3_perception

담당 **하수영**. 어안 4카메라 SVM과 GCN 주차칸 인지를 담는다. GCN은 별도 노드·모듈에서만 로드하며 PyTorch가 SVM의 필수 의존성이 되지 않게 한다.

[실행 명령](../../docs/commands.md#svm) · [구조·확장 위치](../../docs/project/src-refactoring.md) · [보정 근거](../../docs/system/svm-calibration.md)

- 입력: `/cam_{front,rear,left,right}/image_jpeg/compressed`, 자세용 `/tf`의 `map → base_link`.
- 출력: `/perception/svm/clean_bev/compressed` (차량 그림·진단 표시 전 투영), `/perception/svm/preview/compressed`, `panel`, `panel/compressed`, `valid_mask`, `status`.
- GCN 연결: `launch/gcn_slot_detector.launch`가 clean BEV를 받아 `/perception/gcn/slots` (`std_msgs/String`, JSON)와 `/perception/gcn/overlay/compressed`를 발행한다. 슬롯 JSON에는 원본 stamp와 base_link 기준 두 끝점(m)이 들어간다.
- `svm_unsynced.launch`도 같은 clean BEV 토픽을 발행하므로 GCN 입력으로 연결할 수 있다. 이 모드는 카메라 시각 차를 무시하고 각 카메라의 마지막 프레임을 합성하므로 결과는 배선·시각화 확인용이며 정합된 추론 입력으로 간주하지 않는다. 기본 SVM과 unsynced SVM은 같은 출력 토픽을 사용하므로 둘 중 하나만 실행한다.
- 외부 upstream clone은 프로젝트 옆 `external/parking/gcn-parking-slot`에서 읽고, 가중치는 `checkpoint` launch 인자로 지정한다. 모델 기본 설정은 upstream `config/ps_gat.yaml`이며 CUDA 자동 선택을 지원한다.
- SVM 보정 상태는 preview 수준이다. 슬롯 좌표를 자율 주차 제어에 쓰기 전 여러 위치·방향에서 검증해야 한다.
- 원본 JPEG는 콜백에서 버퍼링하고, 단일 작업자가 선택한 묶음만 디코딩한다. 기본 출력 상한 10Hz, 입력 누락/과거 프레임은 거부한다.
- `roslaunch vip3_perception svm_unsynced.launch`는 640×480 입력 설정(`svm_640x480.yaml`)을 기본으로 사용하며, timestamp 시차와 입력 age 검사를 건너뛰고 각 카메라의 마지막 프레임을 합성한다. 입력을 못 받은 카메라는 `NO FRAME` 검은 화면으로 대체한다. MORAI에는 `config/morai/261115_VIP3_Sensor_26.R1.H3_640x480.json`을 Import한다. 기본 SVM과 출력 토픽이 같아 동시에 실행하지 않는다.
- 렌즈·지면·자세·시차 계산은 ROS 없는 `src/vip3_perception/svm/`, 픽셀↔차량 좌표는 `coordinates.py`, ROS 연결은 `scripts/svm_node.py`에 둔다.
- `config/svm.yaml`과 `config/vehicles/`가 정본이다. 그림은 `assets/`, 오프라인 도구는 저장소 `tools/svm/`에 있다.

캘리브레이션은 일부 정차 자세에서 검증한 preview다. 여러 위치·선회·경사에서 추가 확인해야 하며 차량 그림과 마스크 해제로 보이지 않는 지면을 복원할 수는 없다.
