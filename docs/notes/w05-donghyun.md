# <5>주차 작업 기록 — <김동현>

**기간:** 2026-09-28 ~ 10-04 (개강 08/31 기준 주차 → [../course.md](../course.md) §3)

## 이번 주에 한 일

- 수집 파이프라인 구조 파악: Capture Mode → `sync_capture_data.py` → `curate_perception_frames.py` → `bake_perception_masks.py` → `build_perception_dataset.py` (README 절차 기준)
- DMPR-PS 라벨 형식 확인 (`prepare_dataset.py` 기준): 600×600 jpg와 JSON(`marks`: 표시점 위치, 방향용 두 번째 점, 종류값), 512×512로 변환, 5° 단위 회전 증강
- `drivable_bev`의 `rgb_bev_node.py` 확인: 카메라 4대 원본을 지면 평면으로 투영해 서라운드 BEV를 합성하는 기존 노드가 있음. 격자는 x[-10,10] y[-8,8] m, 해상도 0.05 m/px (주차칸 2.5×5.0 m → 약 50×100 px)
- **Semantic 확인(과제 #10)**: MORAI Capture Mode로 Semantic PNG를 저장하고, cv2로 픽셀을 직접 분석해 클래스 구성을 확정함

## 알게 된 것 / 공부한 것

- 공식 Semantic 팔레트(`OFFICIAL_PALETTE_RGB`)는 26개 클래스이며 주차칸 전용 클래스는 없다. 선 관련 클래스는 `white_lane`, `yellow_lane`, `blue_lane`, `stopline`, `crosswalk`이다.
- Semantic PNG 한 장을 픽셀 단위로 분석한 결과(픽셀 수 많은 순):

| 색 (RGB) | 픽셀 수 | 클래스 |
|---|---|---|
| (0,255,255) | 408,927 | `sky` |
| (23,2,6) | 206,093 | `etc` |
| (127,127,127) | 159,015 | `asphalt` |
| (0,0,0) | 118,172 | `ego_vehicle` |
| **(255,255,255)** | **18,131** | **`white_lane`** |
| (153,255,51) | 4,571 | `building` |
| (113,178,37) | 3,968 | `standing_object` |
| (255,102,30) | 904 | `sidewalk` |
| (99,48,250) | 659 | `traffic_sign` |
| (255,74,240) | 537 | `traffic_light` |
| **(255,255,0)** | **267** | **`yellow_lane`** |
| (204,127,51) | 180 | `road_sign` |
| (255,0,0) | 100 | `stopline` |
| (76,255,76) | 75 | `crosswalk` |
| (0,178,255) | 1 | `blue_lane` |

- 주차선/차선은 `white_lane`이 압도적으로 많고(18,131px), `yellow_lane`도 소량 확인됨(267px). `blue_lane`은 1px로 노이즈 수준. 공식 팔레트 26개 중 10개 클래스가 한 프레임에서 관측됨.

## 다음 주 계획

- 저장된 Semantic PNG로 `audit_semantic_capture.py` 공식 스크립트 돌려 교차 검증
- 지정 슬롯의 선 클래스와 픽셀값 확인 (이번에는 임의 장면 1곳만 확인함)
- 실측 pose로 `vip3_eval` CSV 확보

## 팀 공유 사항

- Semantic 렌더링에서 주차선은 흰색(`white_lane`, 18,131px)이 주를 이루고 노란선(`yellow_lane`, 267px)도 일부 있음.