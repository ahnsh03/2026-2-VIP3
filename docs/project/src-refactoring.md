# src 구조와 정리 계획

2026-11-12 기준. **차량 상태·기어, 제공 GT·좌표 변환, 주차 평가는 `vip3_parking` 하나에 둔다.** 경로·추종도 이 패키지 안에서 구현한다. 작은 기능마다 ROS 패키지를 추가하지 않는다.

## 이번에 적용한 통합

| 이전 | 처리 | 현재 |
|---|---|---|
| `vip3_vehicle_state` | 상태·기어 노드와 수신 경과 계산 통합. 미사용 GPS/IMU 변환 제거 | `vip3_parking/{scripts,src/vip3_parking}` |
| `vip3_hd_map` | 전체 MGeo 도로망·차선·표지·RViz 표시와 수동 슬롯 fallback 제거 | GT 원본은 `vip3_parking/data/katri`, 명시적 CRS/원점 변환은 `frames.py` |
| `vip3_eval` | 별도 최상위 디렉토리 제거, 순수 평가 함수·테스트 통합 | `vip3_parking.evaluation` |

`vip3_eval`에는 원래 package.xml이 없었다. 따라서 **catkin 패키지는 8개→7개**, 별도 Python 평가 디렉토리도 제거된 상태다. 삭제 코드는 Git 이력에 남으며 현재 src에 legacy 사본을 만들지 않는다. 제공 GT의 31개 레코드·원본 바이트·SHA-256은 그대로 보존한다.

현재 주차 패키지:

```text
src/vip3_parking/
├── launch/vehicle.launch       # 상태·기어 실행; 주차 추종은 아직 아님
├── scripts/
│   ├── state_node.py
│   └── gear_node.py
├── src/vip3_parking/
│   ├── state.py                # Ego 메시지 단위·좌표 처리
│   ├── freshness.py            # 수신 경과
│   ├── gt.py                   # 제공 polygon을 idx로 조회
│   ├── frames.py               # 명시된 원점 간 변환
│   └── evaluation.py           # 외곽·오차·충돌 결과 평가
├── data/katri/                 # parking_space_set.json + 출처
└── test/
```

GT polygon에는 입구 방향·현재 씬 정합이 확정되어 있지 않다. `gt.py`는 원본 좌표를 읽기만 하고, `frames.py`도 두 원점과 CRS를 직접 지정해야 한다. 이 파일에 과거 전체 도로망의 원점을 자동 적용하지 않는다. 경로 생성·추종·인지 목표 어댑터는 이후 실제 코드가 생길 때 같은 패키지에 추가한다.

## 남은 SVM 분리와 레거시 정리

아래는 **계획이며 아직 적용하지 않은 이름**이다. 현재 SVM 실행은 `roslaunch drivable_bev fisheye_svm.launch`다.

```text
src/
├── morai_msgs/             # 외부 submodule, 26.R1 pin 유지
├── vip3_msgs/              # 상태·기어 등 생산자/소비자 공용 계약
├── vip3_bringup/           # bridge·launch·진단
├── vip3_svm/               # 예정: 기존 drivable_bev에서 어안 기능만 추출
├── vip3_data/              # 선택: data_collection의 최종 이름
├── vip3_parking/           # 통합 완료: 상태·기어·GT·평가, 이후 경로·추종
└── vip3_slot_detection/    # 예정: SS-PSD adapter가 구현될 때 생성
```

| 순서 | 작업 | 완료 기준 |
|---|---|---|
| 완료 | SVM CPU/입력 처리 최적화 | [측정과 회귀 결과](../system/svm-performance.md) |
| 완료 | 주차 주변 기능 통합, HD map 제거 | 새 catkin build/install, 상태/기어 연결, GT 보존·평가·좌표 검사 |
| 다음 | 어안 SVM을 `vip3_svm`, 진입점을 `svm.launch`/`svm_node.py`로 분리 | 렌즈·자세·마스크·출력 동일, Python import/launch/Docker/CI/문서 함께 수정 |
| SVM 분리와 함께 | 미사용 TwinLite/핀홀 BEV·semantic 학습 경로 제거 | 필요한 공통 기하를 먼저 추출하고 관련 scripts/config/테스트의 참조 정리 |
| 기능 구현 시 | SS-PSD adapter와 `vip3_parking`의 planner/controller 모듈 추가 | GT/인지 목표 출처 구분, `/ctrl_cmd` 단일 발행자, 고정 조건 재생 |
| 선택 | 수집 패키지 이름을 `vip3_data`로 통일 | 기존 parking_raw 수집·감사·재생 기능 유지 |

현재 `drivable_bev/__init__.py`는 핀홀·차선 코어와 `camera_semantic_perception`까지 import한다. 따라서 TwinLite 패키지 폴더만 먼저 지우면 SVM도 깨진다. 분리할 때 가벼운 `__init__.py`, 독립적인 지면·렌즈 코어로 정리한다. SVM 전용 설정·차량 에셋·검증 자료·tools 경로도 같은 변경에서 옮긴다.

## 실행 명령과 담당

현재 상태 관측은 `roslaunch vip3_parking vehicle.launch start_gear_node:=false`다. `/vehicle/*`, `/vip3_gear/set_gear`, 노드 이름과 메시지 타입은 유지했다. 과거 package/launch 이름의 호환 패키지는 남기지 않았으며 팀원은 rebuild 후 [commands.md](../commands.md)의 명령을 사용한다.

- 강도균: `vip3_parking`의 GT·평가·이후 경로·추종. 김동현: 같은 패키지의 상태·기어 배선, bridge·수집.
- 하수영: SVM 분리·SS-PSD. 안승현: 센서·차량·GT 좌표 기준. 장원태: 최종 구조와 결과 발표.
- `morai_msgs`는 외부 의존성이며 이름을 바꾸지 않는다. `vip3_msgs`는 다른 파트가 주차 구현에 의존하지 않고 메시지 계약만 공유할 수 있도록 유지한다.
- 패키지 이동 후 새 build/devel/install에서 확인해 옛 Python wrapper가 누락을 가리지 않도록 한다. 새 기능이 없는 빈 패키지를 만들지 않는다.

이름 정리는 기능 개발과 분리해 작은 PR로 진행하고 팀의 주 구현 시간 45인시를 과도하게 쓰지 않는다. 상세 오너십은 [팀 문서](team.md), GT와 평가의 한계는 [데이터 문서](../system/data.md)를 따른다.
