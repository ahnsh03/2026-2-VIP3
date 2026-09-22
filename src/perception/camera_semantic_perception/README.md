# camera_semantic_perception — 4카메라 TwinLiteNet+ 추론

> **문서 역할:** 패키지 가이드
> **담당:** 하수영 (모델) · 안승현 (배선)
> **토픽 정본:** [config/vip3_topics.yaml](../../../config/vip3_topics.yaml)

MORAI rosbridge 가 주는 4대 JPEG 카메라를 **latest-only** 로 받아 TwinLiteNet+ 2-head 추론
결과를 표준 ROS 토픽으로 발행한다. 모델 로딩은 backend 에 격리돼 있어 **다른 segmentation
모델로 바꿔도 ROS 배선은 그대로**다 — 하수영이 기반 논문 모델로 갈아탈 때 이 구조를 쓴다.

## 실행

```bash
cd /root/ws && source devel/setup.bash

# 전방 한 대만 (첫 확인)
roslaunch camera_semantic_perception single_view.launch view:=front

# ★ 후방 단독 — 아무도 해 본 적 없는 실험
roslaunch camera_semantic_perception single_view.launch view:=rear
roslaunch camera_semantic_perception single_view_visualizer.launch view:=rear

# 4뷰 + 시각화
roslaunch camera_semantic_perception four_view_with_visualizer.launch

# v6 4-class 체크포인트로
roslaunch camera_semantic_perception four_view.launch \
  checkpoint:=/root/ws/weights/twinlite/v6_road_marking/best_mean.pt
```

보기:

```bash
vip3-ros-gui rqt-image /perception/camera/debug/mosaic/compressed     # 4열 모자이크
vip3-ros-gui rqt-image /perception/camera/rear/debug/panel/compressed # 한 뷰
```

임계값은 재시작 없이 바꾼다:

```bash
rosparam set /camera_semantic_visualizer/drivable_threshold 0.55
rosparam set /camera_semantic_visualizer/lane_threshold 0.45
```

## 입출력

| view | 입력 토픽 | 원본 | FOV | letterbox pad (l,t,r,b) |
|---|---|---|---:|---|
| front | `/cam_front/image_jpeg/compressed` | 1280×720 | 90° | (0, 12, 0, 12) |
| left | `/cam_left/image_jpeg/compressed` | 640×480 | 130° | (64, 0, 64, 0) |
| right | `/cam_right/image_jpeg/compressed` | 640×480 | 130° | (64, 0, 64, 0) |
| **rear** | `/cam_rear/image_jpeg/compressed` | 1280×720 | 90° | (0, 12, 0, 12) |

출력 (`{view}` = `front|left|right|rear`), 전부 `sensor_msgs/Image` `mono8`:

```
/perception/camera/{view}/drivable/probability      0~255 = 확률 0~1
/perception/camera/{view}/lane/probability          binary 체크포인트
/perception/camera/{view}/road_marking/class_id     4-class 체크포인트 (0 bg/1 white/2 yellow/3 stopline)
/perception/camera/{view}/road_marking/confidence   4-class 체크포인트
/perception/camera/diagnostics                      diagnostic_msgs/DiagnosticArray
/perception/camera/{view}/debug/panel/compressed    시각화
/perception/camera/debug/mosaic/compressed          시각화 1920×1080
```

**입력 메시지의 `header.stamp` 와 `frame_id` 를 그대로 보존한다.** 이 보존이 시각화 노드와
BEV 의 정확 시각 동기 전제다. 바꾸지 않는다.

체크포인트의 메타데이터로 binary / 4-class 를 자동 구분한다. 설정할 것 없다.

## 성능 — 4뷰는 여유가 없다

ASMC 실측(같은 4060 Ti 급, 1280×720, Medium, 478,876 파라미터):
front 단독 약 20 Hz (GPU 16~19 ms), 3뷰 약 18.5~19 Hz (GPU 23~25 ms).
선형 외삽하면 4뷰 GPU 약 27~29 ms.

병목은 GPU 가 아니라 **단일 worker 스레드의 CPU 작업**이다:

```
JPEG 디코드   2×1280x720 + 2×640x480     4~8 ms
letterbox     4× resize/cvtColor/transpose 3~5 ms
GPU 추론                                 27~29 ms
restore       2 head × 4 view 원본 복원   8~14 ms
publish       4.92 MB 직렬화             3~6 ms
합계          45~62 ms  ->  16~22 Hz
```

**4뷰 20 Hz 는 가능하지만 마진이 없다. 15~18 Hz 로 계획하고 20 Hz 는 스트레치로 둔다.
주차는 5 km/h 미만이라 10 Hz 로도 충분하다 — 여기에 개발 주차를 태우지 않는다.**

VRAM 은 문제가 아니다 (학습 피크가 2.51 GB, 추론 batch 4 는 1.5 GB 미만).

### 튜닝 순서 (위에서부터)

1. **소스 해상도를 낮춘다.** front/rear 를 640×360 으로. 모델은 어차피 384×640 으로
   letterbox 하므로 **정확도 손해가 0** 이고 디코드·복원·publish·rosbridge 부하가 모두 준다.
   → **BEV 내부 파라미터가 같이 바뀐다.** 강도균·안승현과 먼저 합의한다
2. `restore_original: false` — publish 4.92 → 1.97 MB/batch. BEV projector 가 384×640
   model grid 를 소비하도록 같이 고쳐야 한다
3. `channels_last: true` (A/B)
4. 좌우 카메라를 10 Hz 로 (`sensorPeriod 0.1`). 주차는 횡방향 변화가 느리다
5. `batch_wait_ms` 10~12
6. **여기까지 다 하고 나서야** TensorRT/ONNX 를 생각한다. 1번이 공짜로 주는 것을 얻으려고
   한 주를 쓰는 셈이다

### 과부하 판단

```bash
rostopic echo /perception/camera/diagnostics
```

- `replaced_by_view` 가 늘어나면 처리보다 입력이 빠른 것이다 (= 과부하 경보)
- `output_frames / output_batches` 가 **4.0 에 가까워야** 한다. 2~3 이면 batch 가 쪼개지고
  있으니 `batch_wait_ms` 를 올린다
- `end_to_end_p95_ms` 가 실제 지연이다

## 설계

```
콜백 → LatestFrameBuffer(뷰별 최신 1장만 유지, 밀린 건 버림) → worker 스레드 1개
     → 디코드 → letterbox → GPU(batch 1~4) → 복원 → publish
```

ROS 콜백은 버퍼에 넣기만 한다. `pop_batch` 가 한 뷰라도 들어올 때까지 기다린 뒤
`batch_wait_ms` 만큼 더 기다려 나머지 뷰를 모은다. 그래서 batch 크기가 1~4 로 변한다.

`views` 는 rosparam 리스트다. 뷰를 늘려도 publisher·버퍼·모자이크가 전부 그 리스트로부터
만들어지므로 **이름만 추가하면 된다** — 후방 fisheye 를 나중에 붙일 때 이 성질을 쓴다.

## ASMC 대비 바뀐 것

1. `SUPPORTED_VIEWS` 에 `rear` 추가
2. **`/morai_sensor_bridge/diagnostics` 구독 삭제** — UDP bridge 진단이었고 VIP3 에는 bridge 가
   없다. 이게 이 패키지의 유일한 UDP 결합이었다
3. 카메라 토픽을 `/cam_*/image_jpeg/compressed` 로 (전방에 접두어를 붙여 4방향 대칭)
4. `batch_wait_ms` 4.0 → 8.0 (rosbridge 는 UDP 보다 지터가 크다)
5. 체크포인트 기본값을 `weights/twinlite/v5_binary_lane/best_mean.pt` 로
6. 모자이크 열 너비 640 → 480 (4열이 1920×1080 에 정확히 맞는다)

## 검증

```bash
cd /root/ws
PYTHONPATH=src/perception/camera_semantic_perception/src \
  python3 -m unittest discover -s src/perception/camera_semantic_perception/test -p 'test_*.py'
```

22개 통과 (2026-09-23). **ROS·GPU 없이 돈다** — letterbox 기하, 배치, 모자이크 조립을
검증하므로 시뮬레이터를 열기 전에 4뷰 포팅이 맞는지 확인할 수 있다.

## 확인해야 할 것

- [ ] MORAI 가 카메라 토픽에 `/compressed` 를 자동으로 붙이는가
- [ ] `header.stamp` 가 0 이 아닌가, sim time 인가 wall time 인가.
      0 이거나 중복이면 시각화 노드의 정확 시각 동기가 아무것도 못 내보낸다
      (추론은 정상인데 화면만 안 나오는 상태)
- [ ] **rosbridge 가 4 JPEG 스트림을 버티는가** ([docs/simulator.md](../../../docs/simulator.md) §5)
- [ ] **후방 뷰에서 TwinLiteNet+ v5 가 쓸 만한 것을 내는가.** 학습 데이터가 K-City 주행
      영상이라 후방·주차장은 분포 밖이다. `single_view.launch view:=rear` 로 30분이면
      답이 나오고, **하수영이 기반 논문 모델로 갈아탈 근거가 바로 이 화면이다**
