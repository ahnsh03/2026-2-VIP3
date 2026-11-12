# 설정 정본

실행 명령은 [docs/commands.md](../docs/commands.md), 센서·차량 기준은 [센서 문서](../docs/system/sensors.md)에 모은다.

```text
config/
├── morai/
│   ├── 261110_VIP3_Sensor_26.R1.H3.json  # 현재 실행 정본
│   ├── 261112_VIP3_Sensor_26.R1.H3.json  # LiDAR 추가 검토용, 미적용
│   └── 261110_VIP3_Network_26.R1.H3.json
└── ros/
    ├── topics.yaml
    └── rosbridge.env
```

| 자료 | 정본 |
|---|---|
| MORAI Import 센서·네트워크 | `morai/261110_VIP3_Sensor_26.R1.H3.json` + `morai/261110_VIP3_Network_26.R1.H3.json`, ROS 9090 |
| 토픽·프레임·센서 계약 | `ros/topics.yaml` |
| bridge 접속·진단 주기 | `ros/rosbridge.env` |
| SVM 보정·합성 | `src/vip3_perception/config/svm.yaml` |
| 차량 제원 | `src/vip3_perception/config/vehicles/` |
| 차량 탑뷰 이미지 | `src/vip3_perception/assets/` |
| 제공 주차 GT | `src/vip3_parking/data/katri/` |
| 수집·재생 프로파일 | `src/vip3_data/config/` |

패키지 전용 설정은 해당 패키지에 두고 루트에 사본을 만들지 않는다.

새 export 이름은 **`YYMMDD_VIP3_Sensor/Network_모라이버전.json`**으로 한다. 날짜는 export 날짜를 사용하고, 이름만 바꿀 때 새 날짜를 만들지 않는다. 새 정본을 적용하면 `topics.yaml`의 presets, SVM의 `source_sensor_preset`, 카메라 TF, 수집 조건을 함께 갱신하고 설정 검사를 실행한다. 이전 정본은 Git 이력에만 남긴다.

팀은 `config/morai/`의 공유 프리셋과 **9090**을 사용한다. 원본 export와 제공 이미지 원본은 `../data/reference/`에 보존한다. 상세 분석·캡처 보고서는 repo 밖 `../docs/`에 두고 팀 문서에는 재현에 필요한 결론만 남긴다. 파일 분류 기준은 [데이터 문서](../docs/system/data.md)를 따른다.

## 검토용 export

[261112 센서](morai/261112_VIP3_Sensor_26.R1.H3.json)는 기존 카메라4개와 동일한 설정에 LiDAR1개를 추가한 사용자 export다. 현재 실행 정본으로 선택하지 않았다. `/lidar3D`·`Lidar3D-5`의 실제 수신/TF/수집 계약은 미검증이며, 기본 `topics.yaml`과 SVM은 261110을 참조한다. 팀원은 위 실행 정본을 Import하고, 새 센서는 담당자 검토 후 채택한다. [인수인계](../docs/notes/261112-svm-handoff.md#추가된-센서-export)
