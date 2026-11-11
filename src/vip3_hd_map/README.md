# vip3_hd_map

MGeo 로딩·좌표 코어와 RViz MarkerArray 시각화. GT 기반 제어는 **강도균**, 자료·좌표 설정 관리는 **안승현**과 확인한다.
[실행 명령](../../docs/commands.md#packages) · [GT·좌표 기준](../../docs/system/data.md)

제공 주차 GT 31개는 [data/katri/](data/katri/README.md)에 보관한다. raw GT를 수정하지 않고 `idx`로 선택한다. 목표 pose·입구 방향·좌표 정합은 별도 개발 항목이다.

## 구현

- `inspect_katri_mgeo.py`: ROS 없이 레이어·좌표·매니페스트·주차면 유무 확인. GT-only 폴더도 검사 가능.
- `katri_map_viz.launch`: 전체 MGeo와 차량 pose를 RViz에 표시. `map_dir`로 자료를 지정한다.
- `pose_source=ego_status`가 현재 사용 경로다. `gps_imu`는 별도 센서와 원점·lever arm 검증이 필요하며 현재 센서셋에는 없다.
- global 토픽은 `map`, local 토픽은 `base_link` 기준이다. 근처 링크까지 거리·원점 경고는 참고 검사이며 좌표 정합 완료를 보장하지 않는다.

| 출력 | 내용 |
|---|---|
| `/vip3_hd_map/global/*` | links, lane_boundaries, stop_lines, surface_markings, crosswalks, parking_spaces |
| `/vip3_hd_map/local/*` | 차량 주변 crop, ego_context |
| `/vip3_hd_map/debug/pose` | 사용한 ego pose |

GT-only 폴더에는 전체 RViz 실행에 필요한 global_info·도로 레이어가 없다. 과거 KATRI와 K-City 2025 원점은 다르므로 제공 GT를 옛 전체 맵 폴더에 복사해 같은 좌표로 취급하지 않는다. 기존 `workspace_origin`은 에디터 viewport 값이므로 좌표 변환에 사용하지 않는다.

## 로더와 스키마

`parking_space_set.json`을 우선 읽고, 없으면 설정된 수동 `vip3_parking_spaces.json`, 둘 다 없으면 빈 주차 레이어를 쓴다. [수동 형식 설명](config/vip3_parking_spaces.SCHEMA.md)은 데이터 편집용이며 목표 자동 추정 규칙이 아니다.

`lane_type`·`lane_color`가 리스트, surface type/sub_type이 문자열일 수 있어 코어에서 정규화한다. 확정하지 않은 lane_type은 코드값을 유지한다. 제공 GT의 width/length/angle 기본값 대신 points로 형상을 계산한다.
