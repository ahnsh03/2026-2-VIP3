# <5>주차 작업 기록 — <김동현>

**기간:** 2026-09-28 ~ 10-04 (개강 08/31 기준 주차 → [../course.md](../course.md) §3)

## 이번 주에 한 일

- 수집 파이프라인 구조 파악: Capture Mode → `sync_capture_data.py` → `curate_perception_frames.py` → `bake_perception_masks.py` → `build_perception_dataset.py` (README 절차 기준)
- DMPR-PS 라벨 형식 확인 (`prepare_dataset.py` 기준): 600×600 jpg와 JSON(`marks`: 표시점 위치, 방향용 두 번째 점, 종류값), 512×512로 변환, 5° 단위 회전 증강
- `drivable_bev`의 `rgb_bev_node.py` 확인: 카메라 4대 원본을 지면 평면으로 투영해 서라운드 BEV를 합성하는 기존 노드가 있음. 격자는 x[-10,10] y[-8,8] m, 해상도 0.05 m/px (주차칸 2.5×5.0 m → 약 50×100 px)
- **Semantic 확인(과제 #10)**: MORAI 카메라 센서 속성(`Sensor Settings`)에서 `Ground Truth`를 `Semantic`으로 전환하고 `View`로 MORAI가 실제 렌더링한 화면을 확인. 주차칸 선이 흰색으로 렌더링됨 (팔레트상 `white_lane`에 해당). 같은 화면에서 하늘·노면·차량 색이 각각 `sky`·`asphalt`·`ego_vehicle`과 일치해 공식 팔레트 사용을 뒷받침함

## 알게 된 것 / 공부한 것

- 공식 Semantic 팔레트(`OFFICIAL_PALETTE_RGB`)는 26개 클래스이며 주차칸 전용 클래스는 없다. 선 관련 클래스는 `white_lane`, `yellow_lane`, `blue_lane`, `stopline`, `crosswalk`이다.

## 막힌 것

- 증상: `capture_collector`로 `/SaveSensorData`를 발행했으나 Windows에서 PNG를 찾지 못했다.
  - 이 때문에 과제 #10을 지정된 `audit_semantic_capture.py`(픽셀 단위 자동 확인) 대신, 센서 속성의 `Ground Truth` 전환 후 `View`로 보는 방식(육안 확인)으로 대체했다.

## 다음 주 계획 / 질문

- MORAI에서 카메라로 Capture하는 방법을 모르겠습니다.
- 저장된 파일 픽셀값은 어떻게 확인하는지도 궁금합니다.
- 지정 슬롯의 선 클래스와 픽셀값 확인 (이번에는 임의 장면 1곳만 확인함)
- 실측 pose로 `vip3_eval` CSV 확보

## 팀 공유 사항

- Semantic 렌더링에서 주차선은 흰색(`white_lane`)으로 보이며 칸 전용 클래스는 없다 (육안 확인, 픽셀값 재확인 예정).