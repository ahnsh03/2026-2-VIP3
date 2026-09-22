# vip3_hd_map — KATRI MGeo 로더·시각화

> **문서 역할:** 패키지 가이드
> **담당:** 안승현
> **맵 정본:** `$VIP3_DATA/KATRI 맵 데이터 자료/` (저장소 밖) — [docs/katri-map.md](../../docs/katri-map.md)

KATRI MGeo JSON 을 읽어 RViz `visualization_msgs/MarkerArray` 로 띄운다.
**커스텀 메시지를 쓰지 않는다** — RViz 만 있으면 누구나 본다.

ASMC `asmc_hd_map` 은 19,241줄이었고 대부분이 K-City 2025 대회용 경로 큐레이션과
주행가능영역 A/B 연구였다. 여기는 그중 좌표계·크롭·오버레이 코어만 가져오고 나머지는
새로 썼다.

## 제일 먼저 — ROS 없이 맵부터 본다

```bash
python3 scripts/inspect_katri_mgeo.py --map-dir "$VIP3_DATA/KATRI 맵 데이터 자료"
```

좌표계, 레이어별 개수·범위, lane_type 분포, **`global_info` 매니페스트 대조**,
주차 레이어 유무, 주차장 밀집도 휴리스틱을 출력한다. ROS 빌드가 필요 없다.

주차면 유무만 확인하려면 (있으면 exit 0):

```bash
python3 scripts/inspect_katri_mgeo.py --map-dir "$VIP3_DATA/KATRI 맵 데이터 자료" --check
```

**이 명령이 이 패키지에서 가장 쓸모 있는 것이다.** MORAI 가
`parking_space_set.json` 을 보내 줬는지 1초에 확인된다.

## RViz

```bash
roslaunch vip3_hd_map katri_map_viz.launch
roslaunch vip3_hd_map katri_map_viz.launch local_radius_m:=15.0   # 주차 근접
roslaunch vip3_hd_map katri_map_viz.launch publish_global:=true rviz:=false
roslaunch vip3_hd_map katri_map_viz.launch pose_source:=gps_imu   # /Ego_topic 이 (0,0) 일 때
```

| 토픽 | frame | latch | 내용 |
|---|---|---|---|
| `/vip3_hd_map/global/links` | `map` | ○ | 링크 중심선 667 |
| `/vip3_hd_map/global/lane_boundaries` | `map` | ○ | lane_type 별 ns 분리 |
| `/vip3_hd_map/global/stop_lines` | `map` | ○ | lane_type 530, 164개 |
| `/vip3_hd_map/global/surface_markings` | `map` | ○ | 노면 표시 238 |
| `/vip3_hd_map/global/crosswalks` | `map` | ○ | 횡단보도 58 |
| `/vip3_hd_map/global/parking_spaces` | `map` | ○ | **현재 0개** (아래) |
| `/vip3_hd_map/local/*` | `base_link` | ✗ | ego 주변 `~local_radius_m` 크롭 |
| `/vip3_hd_map/local/ego_context` | `base_link` | ✗ | 차체 footprint |
| `/vip3_hd_map/debug/pose` | `map` | ✗ | 노드가 인식한 ego pose |

`Marker.DELETEALL` 을 매 발행마다 넣는다. 안 넣으면 ego 가 움직일 때 지난 프레임 마커가
RViz 에 눌어붙는다.

## 주차면 — 지금은 비어 있다

배포된 KATRI MGeo 에 `parking_space_set.json` 이 없다. 그런데 **원본 export 에는 있었다** —
`global_info.json` 의 `mgeo_file_hash` 가 그 증거다 ([docs/katri-map.md](../../docs/katri-map.md) §1).

로더는 세 단계로 찾는다:

1. `parking_space_set.json` (맵 디렉터리) — 받으면 **코드 수정 없이** 켜진다
2. `config/vip3_parking_spaces.json` — 손으로 적은 fallback, 같은 스키마
3. 없으면 빈 리스트 + 경고

스키마는 MORAI MGeo `ParkingSpace` 와 동일하다 →
[`config/vip3_parking_spaces.SCHEMA.md`](config/vip3_parking_spaces.SCHEMA.md)

## 좌표계

```
map_xy = utm52n_xy − (302459.942, 4122635.537)      EPSG:32652
```

- **MGeo `points` 는 이미 로컬 미터 좌표다.** 변환 없이 `map` frame 에 그대로 찍는다.
- `global_info.workspace_origin` `[168.0, 1461.0, −72.0]` 은 **MORAI 에디터의 뷰포트 상태다.
  좌표 변환 파라미터가 아니다.** 쓰지 말 것.
- 노드가 첫 ego pose 와 가장 가까운 링크의 거리를 재서, `~pose_distance_warning_m`(기본 20 m)
  보다 멀면 경고한다. **`/Ego_topic` 원점이 MGeo 와 다른 경우를 자동으로 잡으려는 장치다.**

## MGeo 필드 타입 함정

실측으로 확인한 것들이다. 로더가 방어적으로 정규화하지만 직접 읽을 때 주의한다.

| 필드 | 실제 타입 |
|---|---|
| `lane_boundary.lane_type` | **리스트** `[505]` |
| `lane_boundary.lane_shape` / `lane_color` | 리스트 |
| `surface_marking.type` / `sub_type` | **문자열** `'1'` / `'5371'` |
| `link.link_type` | 문자열 `'6'`/`'1'` 에 `None` 7개와 `'Driving'` 1개가 섞여 있다 |
| `link.its_link_id` / `lane_change_dir` / `hov` | 전부 `None` |

`lane_boundary` 는 한 폴리라인이 구간별로 다른 `lane_type` 을 가질 수 있는 구조지만
(`lane_type`/`lane_shape`/`lane_color`/`lane_type_offset` 병렬 리스트), KATRI 는 1,579개 중
**18개만** 다중 구간이라 1차 구현에서는 첫 값만 쓴다.

## lane_type 라벨

확인된 것만 이름을 붙이고 나머지는 `UNVERIFIED_<코드>` 다. 특히 **535 는 미확정**이다 —
27개가 `(82.2, 1190.2)` 주변에 흩어져 있는데 centroid 기준 반경 표준편차/평균이 0.536 이라
회전교차로 링이 아니고, 주차 규격도 아니다. NGII B2_SURFACELINEMARK 코드표 원문으로
확인하기 전까지 그대로 둔다. **주차 기능에는 영향이 없다.**

## 검증

```bash
cd /root/ws
PYTHONPATH=src/vip3_hd_map/src \
  python3 -m unittest discover -s src/vip3_hd_map/test -p 'test_*.py'
```

## 확인해야 할 것

- [ ] **`parking_space_set.json` 수령** (최우선 — 팀에서 1인이 메일 한 통)
- [ ] MORAI SIM 3D 씬에 주차장이 실제로 렌더링되는가. MGeo 는 도로망 레이어일 뿐이라
      벡터가 없어도 씬에는 있을 수 있다
- [ ] `/Ego_topic.position` 원점이 MGeo local frame 과 같은가 (노드가 자동 경고)
- [ ] KATRI ego 실제 치수. 현재 `~ego_length_m 4.635 / ~ego_width_m 1.892 /
      ~ego_rear_overhang_m 0.790` 은 ASMC 차량 값이라 **잠정**이다.
      후진 주차에서 실제로 부딪히는 곳은 후범퍼다
