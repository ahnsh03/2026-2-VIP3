# 2026-11-12 통합 기록 — src 정리·SVM 회귀·9090 연결

- 기록: 안승현 요청으로 수행한 작업을 Codex가 코드·검증 보고서·대화에 근거해 정리
- 파트 / 상태: 통합·개발 환경 / 아래 범위 확인 완료
- 코드 기준: [`2ad2783`](https://github.com/ahnsh03/2026-2-VIP3/commit/2ad278376ae7784bb43a57d10cae3a0af33af650), [리팩토링 PR #6](https://github.com/ahnsh03/2026-2-VIP3/pull/6)
- 작업 시간: 별도 측정 없음. 팀원의 개인 작업 시간으로 대신 기입하지 않음

## 한 일

`src`를 `morai_msgs`, `vip3_msgs`, `vip3_bringup`, `vip3_data`, `vip3_perception`, `vip3_parking` 6개 패키지로 정리했다. SVM은 `vip3_perception.svm`에 있고, 이후 SS-PSD는 같은 패키지의 별도 노드로 추가한다. 현재 쓰지 않는 TwinLite·핀홀 BEV·Capture 변환 경로는 삭제했다.

센서·네트워크 프리셋, 보정 수치, 차량 그림, 제공 주차 GT 31개는 그대로 보존했다. 컨테이너를 팀 기본 9090으로 맞춘 뒤 MORAI 센서·네트워크를 재연결했다.

## 재현 조건과 실행

- MORAI 26.R1.H3 / K-City 2025 / `2023_Hyundai_ioniq5`, ROS Noetic.
- 프리셋: `261110_VIP3_Sensor_26.R1.H3.json`, `261110_VIP3_Network_26.R1.H3.json`.
- SVM: `261111_kcity_1280x960_v1`, 카메라 4 × 1280×960, BEV 512×512·10m.
- [환경·9090 연결](../commands.md#setup), [센서 검사](../commands.md#bridge), [SVM·저장 영상 재생](../commands.md#svm), [bag 기록·재생](../commands.md#collection).

연결 검사에 사용한 명령(컨테이너 `/root/ws`, bridge와 MORAI 연결 후):

```bash
python3 scripts/verify_morai_topics.py --duration 15 --profile full \
  --json-out /data/sensor_checks/261112_bridge9090/final_topics.json
```

다시 측정할 때는 다른 출력 파일명을 사용한다.

## 결과와 근거

| 확인 항목 | 결과 | 근거·범위 |
|---|---|---|
| 패키지 이동 | 새 catkin build/install, Docker 이미지 빌드 통과 | [PR #6](https://github.com/ahnsh03/2026-2-VIP3/pull/6), [main CI](https://github.com/ahnsh03/2026-2-VIP3/actions/runs/43131193738) |
| 단위 검사 | perception 34 + data 13 + parking 42 + bringup 8 = **97개** | 컨테이너 및 이미지 빌드 검사. CI의 검사 구성과는 별도 |
| SVM 회귀 | 이전 코드와 remap·weight·mask·출력 픽셀 차이 **0** | 같은 캡처, 변화하는 20개 자세 + 워밍업 5개. 물리 정합 정확도 평가가 아님 |
| 데이터 경로 | snapshot 3묶음, bag 170메시지 기록·감사·재생→SVM | 별도 ROS master의 모의 송신. JPEG·stamp·SHA 보존과 시계 복원 확인 |
| 실제 카메라 수신 | front 10.020 / left 9.978 / right 9.979 / rear 9.997Hz | [15초 수신 보고서](evidence/261112-rosbridge-topics.json) |
| 실제 영상 형식 | 네 뷰 모두 1280×960 JPEG 디코딩 성공 | [디코딩 보고서](evidence/261112-camera-decode.json) |
| 실제 차량 네트워크 | Ego 44.531 / Object 44.519 / Collision 44.531Hz | 설정 50Hz보다 낮음. 메시지 26.R1 필드 검사 통과 |

SVM·데이터 회귀의 상세 로그와 원본은 **안승현 로컬, 아직 팀에 별도 전달 전**이다: 저장소 밖 `docs/experiments/261112-src-refactor/`, `data/sensor_checks/261112_src_refactor/`. 발표나 재현에 원본이 필요하면 보유자에게 요청한다. 공유한 두 연결 JSON은 로컬 원본을 그대로 복사했다.

## 발생한 문제와 해결

처음에는 VIP3 컨테이너가 정지되어 있었고 Docker 매핑도 팀 정본의 9090과 일치하지 않았다. 현재 Windows에서 9090 bind와 WebSocket 접속을 확인한 뒤 컨테이너를 9090으로 맞췄다. 카메라만 재연결됐을 때는 네 영상이 들어와도 `/Ego_topic`의 발행자가 없었다. MORAI 로그의 닫힌 WebSocket 송신 오류를 확인하고 **Network Settings의 Ego-0 연결을 별도로 Disconnect → Connect**한 뒤 전체 검사가 통과했다.

팀 연결 기준은 9090이다. 위 수신율은 해당 PC·측정 시점의 결과이며, 다른 PC에서 항상 50Hz가 나온다는 뜻은 아니다. 연결 문제는 [공통 진단 순서](../commands.md#bridge)를 따라 확인한다.

## 남은 작업과 인수인계

- 김동현: 팀원 PC의 연결 재현, 실제 MORAI 주차 접근 bag과 메타데이터 공유.
- 하수영: 다른 위치·겹침부·주행 자세의 SVM 확인, clean BEV와 SS-PSD 연결.
- 강도균: GT와 씬 좌표·입구 방향을 확인하고 목표 pose·경로·추종 구현.
- 안승현·장원태: 각 파트의 노트와 증거 확인, 발표에 사용할 검증 범위 정리.

**clean BEV 인지 토픽, SS-PSD ROS 검출·추가 학습, 자율주차 경로·추종은 아직 미구현**이다. 이번 9090 검사는 센서 수신 검사이며 새 보정의 라이브 SVM 정합이나 자율주차 성공을 검증한 것이 아니다.

## 발표에 사용할 내용

- 확인된 성과: “6개 패키지로 개발 기반을 정리하고, MORAI 4어안 카메라 약 10Hz 수신과 기존 SVM 출력 보존을 확인했다.”
- 사용할 표: 위 검증 결과. 수신율은 15초 측정 조건을 함께 표시한다.
- 영상·스크린샷: 이 노트에는 첨부하지 않았다. SVM 회귀 캡처는 로컬 보유자에게 요청하고 실제 주차 시연 영상과 구분한다.

## 확인 메모

- 안승현 — 팀 인수인계 관점의 추가 확인: 미확인
- 장원태 — 발표 자료 선정·확인: 미확인
