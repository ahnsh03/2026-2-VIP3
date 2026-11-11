# camera_semantic_perception

**기존 TwinLiteNet+ ROS adapter**. 현재 SS-PSD 주차 슬롯 기준선과 구분한다.
[선택 실행 명령](../../../docs/commands.md#legacy) · [현재 베이스라인](../../../docs/project/baseline.md)

JPEG 입력을 latest-only로 받아 segmentation 결과를 발행한다. 추론 모델은 backend로 분리되어 있지만, SS-PSD의 점·슬롯 출력을 그대로 이 segmentation 인터페이스에 넣을 수는 없다.

| 입력 / 출력 | 내용 |
|---|---|
| `/cam_{view}/image_jpeg/compressed` | front/left/right/rear JPEG; 현재 프리셋은 모두 1280×960 |
| `/perception/camera/{view}/drivable/probability` | mono8 확률 |
| `/perception/camera/{view}/lane/probability` | binary checkpoint의 lane 출력 |
| `/perception/camera/{view}/road_marking/{class_id,confidence}` | 4-class checkpoint 출력 |
| `/perception/camera/{view}/debug/panel/compressed` | 뷰별 시각화 |
| `/perception/camera/debug/mosaic/compressed` | 네 뷰 시각화 |
| `/perception/camera/diagnostics` | 추론 진단 |

입력 stamp·frame_id를 보존한다. checkpoint metadata로 head를 구분한다. 기본 v5와 선택 v6는 `weights/twinlite/`에 있으나 과거 데이터 성능을 현재 주차칸 인식 성능으로 해석하지 않는다. 어안 입력에서의 정확도·처리량은 별도 측정 대상이다.

`single_view`, `four_view`가 추론 래퍼, 대응 `*_visualizer`가 표시 래퍼, `four_view_with_visualizer`가 묶음이다. 공통 진입점은 `semantic_inference.launch`, `semantic_visualizer.launch`다. 기본 주차 ROS 이미지에는 PyTorch가 없어 이 노드를 바로 실행할 수 없다.
