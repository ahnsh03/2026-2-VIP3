# 2026-11-12 SVM 재보정 — 통합 작업 기록

## 한 일

사용자가 다른 위치에서 이전보다 주차선이 어긋남을 보고해 원본 영상·TF로 재검사했다. 카메라 local TF에서 찾은 변환을 ego TF에도 적용한 오류를 수정했다. nominal mount를 유지하고 기존 각도 correction을 제거했으며, 다중 위치 렌즈 피팅과 영상 기반 지면 추정을 적용했다.

## 코드·입력·실행

- `vip3_perception/svm/pose.py`, 새 `ground.py`, `svm_node.py`, `config/svm.yaml`; 기본 프로필 `261112_kcity_ego_ground_v2`.
- 컨테이너 `/root/ws`: `roslaunch vip3_perception svm.launch`. 화면 `EGO CAL | PREVIEW`, status의 `calibration_profile`과 `ground_refinement` 확인. 센서/네트워크 설정과 포트 9090은 동일하다.
- 저장된 원본 5묶음에 같은 대응점을 사용했다. 큰 원본과 분석은 안승현 로컬 `../data/sensor_checks/261112_recalibration`, `../docs/experiments/261112-svm-recalibration`에 보유하며 아직 공유 전이다. 입력 해시는 아래 evidence에 있다.

## 결과·증거

[검증 표와 한계](../system/svm-calibration.md), [기계 판독 결과](../../src/vip3_perception/config/calibration/261112_validation.json), [같은 영상 비교](evidence/261112-svm-comparison.jpg).

가장 문제가 큰 정차 장면의 겹침 주차선 법선 차이는 중앙값 8.42→6.89cm, 95백분위 26.74→18.59cm였다. **잔여 오차가 크며 전체 캘리브레이션 완료가 아니다.** v1이 잘 맞던 두 장면은 중앙값 0.88→1.71cm, 1.23→1.54cm로 소폭 악화됐다. 영상 지면이 채택되지 않는 경우를 포함해 공개한다. 일부 렌즈 피팅 제외 장면도 후보 비교에 사용했으므로 독립 최종 시험으로 발표하지 않는다.

수학·상태·수집 회귀 테스트 97개와 센서/토픽/launch/import 검사를 통과했다. 실제 ROS 노드에 저장 JPEG·TF를 재생해 정상 처리, TF 중단, 영상 중단, 시각 리셋 복귀를 통과했다. 저장 투영은 재생 시 픽셀이 동일했다. 정차 재생 70프레임의 처리 중앙값 26.2ms/P95 55.9ms는 라이브 전체 지연이나 주행 성능이 아니다.

## 막힌 점·다음 작업

오전 검증 중 기존 ROS master가 종료되어 9090 bridge와 새 SVM을 다시 실행했고, 당시에는 재연결 대기였다. **11/12 저녁 재연결 후 라이브 보정 적용과 잔여 오차를 확인했다.** [후속 결과](261112-svm-live.md).

- 안승현: 재연결 후 새 위치·방향에서 카메라/TF를 함께 기록하고 원본과 SVM을 대조한다.
- 하수영: `GROUND ASSUMED`/`VISION`을 구분해 확인한다. 단일 평면으로 설명되지 않는 잔여 오차를 검증한 뒤 clean BEV·SS-PSD 연결을 진행한다. 현재 시각화 영상을 곧바로 학습 입력으로 확정하지 않는다.
- 장원태: 발표에는 개선·악화 수치와 저장 영상 검사 범위를 함께 표기한다. 자율주차 성공이나 라이브 전체 지연 측정으로 표현하지 않는다.

## 확인 메모

위 역할은 다음 확인 담당 제안이며, 팀원이 실제 확인한 결과는 이 아래에 날짜와 함께 추가한다.
