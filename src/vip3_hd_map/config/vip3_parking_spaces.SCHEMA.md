# vip3_parking_spaces.json 스키마

`parking_space_set.json` 을 MORAI 에서 받기 전까지 쓰는 **손으로 적은 주차면 GT** 다.
스키마는 MORAI MGeo `ParkingSpace` 와 **완전히 같다**. 원본이 도착하면 이 파일을 지우고
`parking_space_set.json` 을 맵 디렉터리에 넣으면 코드 수정 없이 대체된다.

```json
[
  {
    "idx": "P0001",
    "points": [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]],
    "center_point": [cx, cy, cz],
    "parking_type": "parallel | perpendicular | diagonal",
    "parking_target_type": "normal | disabled | electric",
    "parking_direction": "forward | backward",
    "distance": 0.0,
    "width": 2.5,
    "length": 5.0,
    "angle": 90.0,
    "linked_left_list_idx": [],
    "linked_right_list_idx": []
  }
]
```

| 필드 | 필수 | 뜻 |
|---|---|---|
| `idx` | ○ | 고유 id. 손으로 적을 때는 `P0001` 부터 |
| `points` | ○ | 주차면 네 코너, **map frame 로컬 미터 좌표** (MGeo 와 같은 좌표계). z 는 생략 가능 |
| `center_point` | | 없으면 `points` 의 무게중심으로 채운다 |
| `width` / `length` | | 기본 2.5 / 5.0 m (일반형 주차장) |
| `angle` | | 진입 각도 [deg]. 직각주차 90 |
| 나머지 | | 없으면 기본값으로 채운다 (`mgeo_layers.normalise_parking_space`) |

## 좌표를 찍는 방법 (빠른 순)

1. **차를 주차면에 정확히 세우고 ego pose 로 역산** — 가장 빠르고 오차가 작다.
   `rostopic echo -n1 /Ego_topic` 의 `position` + `heading` 에 차체 치수를 더해 네 코너를 만든다.
2. BEV 영상 위에서 클릭 후 역투영 (`/perception/bev/debug/rgb/compressed`).
3. RViz `Publish Point` 로 코너 네 개를 찍는다.

어느 방법이든 **좌표계는 MGeo local frame** 이다. UTM 이 아니다
(`map_xy = utm52n_xy − (302459.942, 4122635.537)`, docs/katri-map.md §3).

## 확인

```bash
python3 src/vip3_hd_map/scripts/inspect_katri_mgeo.py \
  --map-dir "$VIP3_DATA/KATRI 맵 데이터 자료"
# §6 주차 레이어 절에서 source 가 manual 로, 개수가 적은 수만큼 나오면 된다
```
