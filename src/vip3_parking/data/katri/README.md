# MORAI 제공 주차면 GT

[parking_space_set.json](parking_space_set.json)은 MORAI 연구원이 제공한 원본의 팀 공유 사본이다. **31개 레코드, 30,847바이트**, 전체 도로망은 포함하지 않는다. catkin install의 패키지 `data/katri/`에도 설치한다.

- 수령 원본: 로컬 `data/KATRI 맵(K-CITY 구간 한정) 주행 가능 영역 자료/mgeo_toolkit/data/KATRI/parking_space_set.json`
- SHA-256: `d31405670ebf7ec84668412a6ce90f9cd102d0cda602072b940ee633df4dc3b1`
- 원본 CRLF 줄바꿈까지 보존한다. `.gitattributes`에서 이 파일의 자동 줄바꿈 변환을 끈다.
- 옛 전체 도로망 매니페스트에 적힌 주차 레이어 해시와 다르다. 이후 수령한 별도 파일로 관리한다.

JSON은 최상위 배열이며 `idx`로 선택한다. `points`가 제공 polygon이고, width/length/angle은 전 레코드에서 2.5/5/90, parking_direction은 null이다. 실제 형상은 points로 계산하고 입구 방향·현재 맵과의 정합을 별도로 확인한다.

제어 개발·평가에 사용하고 원본은 편집하지 않는다. 목표 pose·좌표 변환·선택 ID는 별도 설정으로 남긴다. `vip3_parking.gt.load_spaces()`로 원본 좌표를 그대로 읽는다. 이 파일에 global_info가 없으므로 과거 도로망의 원점을 자동 적용하지 않는다. 현재 차량 pose와의 정합을 별도로 확인한다. 전체 도로망·지도 표시 패키지는 제거했다.

읽기·검사 명령은 [commands.md](../../../../docs/commands.md#packages), 후보 ID와 평가 기준은 [데이터 문서](../../../../docs/system/data.md)를 따른다.
