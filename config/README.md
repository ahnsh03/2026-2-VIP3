# 공유 설정과 로컬 자료

팀원이 clone한 뒤 실행·검증하는 데 필요한 설정은 저장소에 둔다. MORAI Import 파일은 아래 두 개이며, 파일명은 `YYMMDD_VIP3_Sensor_모라이버전.json` / `YYMMDD_VIP3_Network_모라이버전.json` 규칙을 따른다. 현재 `261110`은 원본 export 날짜이고 `26.R1.H3`는 MORAI 버전이다. 파일명만 정리하는 이번 변경에서 날짜를 새로 만들지 않는다.

```text
config/
├── morai/
│   ├── 261110_VIP3_Sensor_26.R1.H3.json        # 현재 4어안, 1280×960, 179°, 10 Hz, ROS 9090
│   └── 261110_VIP3_Network_26.R1.H3.json        # 현재 ROS 네트워크, /ctrl_cmd 포함
├── ros/
│   ├── topics.yaml         # 토픽·프레임·센서 위치·주기 계약
│   └── rosbridge.env       # 접속 기본값과 진단 주기
└── perception/             # 기존 TwinLite 학습 정책·선택 양식 (SS-PSD 정본 아님)
```

| 자료 | 정본 위치 / 관리 방식 |
|---|---|
| MORAI에서 Import할 센서·네트워크 | `config/morai/*.json`, 팀 공유·Git 관리 |
| SVM 파라미터 | `src/perception/drivable_bev/config/fisheye_svm.yaml` |
| Ioniq5 제원·지면 높이 | `src/perception/drivable_bev/config/vehicles/2023_hyundai_ioniq5.yaml` |
| SVM 차량 탑뷰 이미지 | 같은 `vehicles/ioniq5_top_view.png`; 팀 실행 에셋 |
| 팀 공유 주차칸 GT | `src/vip3_hd_map/data/katri/`; [지도 가이드](../docs/katri-map.md) |
| 팀 리뷰용 근거·대표 결과 | `docs/analysis/`, `docs/images/` |
| 수집 영상·bag·학습 데이터·실험 로그 | 저장소 밖 `$VIP3_DATA` (호스트 `../data`, 컨테이너 `/data`) |
| 원본 MORAI export·탑뷰 제공 원본 | `../data/reference/morai_exports/261110/`, `../data/reference/vehicles/` |
| 개인 PC용 주소·포트 변경본 | `../data/local_morai/`; 아래 명령으로 재생성 |
| 원본 MGeo 자료 전체 | 로컬 `data`의 기존 KATRI 자료 폴더; 제공 디렉터리 구조 유지 |

ROS 패키지 전용 설정은 해당 패키지에 둔다. `$(find 패키지)`와 catkin install에서 같은 파일을 찾도록 루트에 중복 사본을 만들지 않는다.

```bash
# 저장소 루트에서, 19090 예외가 필요한 PC만
python3 tools/export_local_morai_presets.py --port 19090 --output-dir ../data/local_morai
# 컨테이너 /root/ws에서는 --output-dir /data/local_morai
```

생성되는 `261110_VIP3_Sensor_26.R1.H3_local19090.json`, `261110_VIP3_Network_26.R1.H3_local19090.json`을 Import한다. **공유 정본은 9090**이다. 동일한 값을 이미 불러온 MORAI는 이름 변경 때문에 재연결할 필요가 없다. 다음 Import부터 새 경로를 사용한다.

새 버전을 적용하면 `config/ros/topics.yaml`의 `presets` 경로와 SVM의 `source_sensor_preset` 등을 함께 갱신하고 `tools/check_sensor_preset.py`를 실행한다. 이전 실행 프리셋은 현행 폴더에 쌓지 않고 Git 이력으로 보존한다.

구형 v1/v2 센서·네트워크 JSON은 제거했다. 정리 전 마지막 상태 `217b5a3`에서 조사 목적으로만 복원한다.

```bash
git show 217b5a3:config/VIP3_sensor_set_v1_ros.json > /tmp/old-sensors.json
```

과거 핀홀 launch는 명시적인 `camera_config` (투영 노드는 `sensor_set_path`도)를 요구한다. `test/fixtures/pinhole_cameras.yaml`은 수학 회귀 검사용이며 실행 정본이 아니다. 현재 실행은 `roslaunch drivable_bev fisheye_svm.launch`를 사용한다.
