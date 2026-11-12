# 설정 정본

실행 명령은 [docs/commands.md](../docs/commands.md), 센서·차량 기준은 [센서 문서](../docs/system/sensors.md)에 모은다.

```text
config/
├── morai/
│   ├── 261110_VIP3_Sensor_26.R1.H3.json
│   └── 261110_VIP3_Network_26.R1.H3.json
└── ros/
    ├── topics.yaml
    └── rosbridge.env
```

| 자료 | 정본 |
|---|---|
| MORAI Import 센서·네트워크 | `morai/`, ROS 9090 |
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
