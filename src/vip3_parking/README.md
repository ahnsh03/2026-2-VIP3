# vip3_parking

주차에 필요한 **차량 상태·기어, 제공 GT·좌표 변환, 결과 평가**를 한 패키지에 모은다. 경로 생성·추종·주차 상태기는 아직 구현하지 않았으며 이 패키지에 추가한다.

[실행 명령](../../docs/commands.md#packages) · [좌표·단위 계약](../../docs/system/design.md) · [제공 GT](data/katri/README.md)

| 위치 | 역할·담당 |
|---|---|
| `scripts/state_node.py`, `src/vip3_parking/state.py` | Ego→상태·odom. 김동현 배선, 강도균 소비 |
| `scripts/gear_node.py` | 기어·External 제어 모드 서비스. 김동현 배선, 강도균 요청 정책 |
| `src/vip3_parking/gt.py`, `data/katri/` | 원본 polygon을 ID로 조회. 강도균 사용, 안승현 좌표 확인 |
| `src/vip3_parking/frames.py` | 명시한 두 CRS·원점 간 변환. 정합을 자동 추정하지 않음 |
| `src/vip3_parking/evaluation.py` | 칸 내 주차·오차·충돌 평가. 강도균 구현, 장원태 결과 활용 |
| `src/vip3_parking/freshness.py` | 단조 시계 기준 수신 경과 계산 |

## 실행과 인터페이스

```bash
roslaunch vip3_parking vehicle.launch start_gear_node:=false
# 기어 서비스를 포함하는 대체 실행. 기본값은 External 모드 전환 요청
roslaunch vip3_parking vehicle.launch
# 서비스는 준비하되 시작 시 External 전환을 하지 않는 대안
roslaunch vip3_parking vehicle.launch set_external_on_start:=false
```

| 노드 이름 | 입력 | 출력 / 서비스 |
|---|---|---|
| `vip3_ego_state` | `/Ego_topic` | `/vehicle/state`, `/vehicle/odom`; 선택 map→base_link TF |
| `vip3_gear` | `/Service_MoraiEventCmd` | `/vehicle/gear`, `/vip3_gear/set_gear` |

노드 이름·토픽·메시지·서비스는 통합 전과 같다. `publish_map_tf=false`로 MORAI TF 중복을 피하고, `stamp_mode=source`는 0 timestamp를 거부한다. `receive`는 수신 시각 사용을 명시한 개발 옵션이다. heading·각속도는 deg→rad, 상태 속도는 m/s다. 기본 `velocity_is_body_frame=true`, `signed_speed=true`이며 실제 이동으로 축과 부호를 확인해야 한다.

기어 코드는 P=1/R=2/N=3/D=4다. 상태 노드는 `/ctrl_cmd`를 발행하지 않는다. `vehicle.launch` 실행은 경로 추종 시작을 뜻하지 않는다.

## GT와 평가

호스트에서 `python3 tools/inspect_parking_gt.py --id A5256W000008`로 확인한다. Python에서는 `from vip3_parking.gt import load_spaces`, `from vip3_parking.evaluation import evaluate_parking`을 사용한다. catkin 환경 없이 쓸 때는 `PYTHONPATH=src/vip3_parking/src`를 지정한다.

`parking_space_set.json`은 원본 31개 레코드·CRLF·SHA-256을 유지한다. `width/length/angle` 기본값 대신 points로 형상을 계산하며 입구 방향·현재 맵 정합은 별도 확정한다. 평가 함수의 비직사각형/진입 방향 한계는 [데이터 문서](../../docs/system/data.md)에 있다.

## 정리 범위

기존 `vip3_vehicle_state`, `vip3_hd_map`, `vip3_eval`의 필요한 기능을 합쳤다. 전체 MGeo 도로망·차선·표지·RViz 표시, GPS/IMU 변환, 수동 주차칸 fallback은 제거했다. 삭제 코드는 Git 이력에 남는다. 보정 SVM과 SS-PSD 입력을 이 패키지에 섞지 않는다.
