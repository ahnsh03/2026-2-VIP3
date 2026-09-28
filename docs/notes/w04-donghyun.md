# <4>주차 작업 기록 — <김동현>

**기간:** 2026-09-21 ~ 09-27 (개강 08/31 기준 주차 → [../course.md](../course.md) §3)

## 이번 주에 한 일

- 개발 환경 구성: WSL2(Ubuntu 20.04), Docker Desktop(WSL 연동), 컨테이너 이미지 빌드, `docker_ros_up.sh check` 3항목 [OK] (rosbridge 9090 기동 확인). MORAI 연결은 미실시.
- `src/vip3_eval/` 신규 작성: 차량 외곽(IONIQ 5, 4.635 × 1.892 m) 기준 주차 평가 함수
  - 지표: 코너 최대오차, 횡/종 오차, heading 오차, 칸 경계 여유, 성공 판정 2종, CSV 행 변환
  - 단위 테스트 12건 통과
- 주차 성공 기준 문헌 조사 및 기준 설계 (아래)

## 알게 된 것 / 공부한 것

- 주차 성공 기준 (문헌)

| 출처 | 성공 기준 | 비고 |
| --- | --- | --- |
| [E2E Parking Dataset](https://arxiv.org/abs/2504.10812) | 차량 중심이 칸 중심에서 횡 0.6 m · 종 1.0 m 이내, 방향 오차 10° 이하 | CARLA 벤치마크의 TSR 정의 |
| [SEG-Parking](https://arxiv.org/pdf/2509.13956) | 위치 오차 1.2 m 미만, 방향 오차 15° 미만 | 위보다 느슨 |

- 보고된 평균 오차는 위치 0.24~0.30 m, 방향 0.34~0.87° 수준이다. 문헌은 코너 오차가 아니라 차량 중심 위치와 yaw로 평가한다.
- IONIQ 5 제원([참고](https://en.wikipedia.org/wiki/Hyundai_Ioniq_5))으로 계산하면(칸 2.5 × 5.0 m 가정) 정렬 시 좌우 여유 0.304 m, 앞뒤 여유 0.1825 m이다. 중앙 정렬 상태에서도 heading 오차가 약 7.7°를 넘으면 차량이 칸 밖으로 나가며, 10°에서는 약 8 cm 이탈한다. 문헌의 10° 기준은 칸 안 주차를 보장하지 않으므로 두 기준을 병기했다.
- ROS는 호스트 버전과 무관하게 컨테이너 안에서만 실행한다 (setup.md 규약).
- 기준선 경계값에서 부동소수점 오차로 판정이 어긋나, mm 단위로 반올림한 뒤 비교하도록 했다.

## 막힌 것

- 증상: `build_ws.sh`가 `vip3_hd_map`에서 실패 (`setup.py` 0.2.0, `package.xml` 0.1.0 버전 불일치)
  - 조치: 로컬에서만 `package.xml`을 0.2.0으로 맞추고 진행. 저장소에는 미반영.

## 다음 주 계획

- 9/30~10/1 중 MORAI 연결 후 `scripts/audit_semantic_capture.py`로 Semantic 주차칸 클래스 확인
- MORAI 연결 후 `/Ego_topic` position 기준점(차량 중심 / 후륜축)과 heading 규약 확인, `ref_to_center_m` 반영
- KATRI 지정 칸의 실제 폭·길이 확인
- 실측 pose로 `vip3_eval` CSV 확보 (발표 6번 슬라이드 오차 표)

## 팀 공유 사항

- `vip3_eval` 입력: `car_pose=(x, y, heading_deg)`, `target_space`는 `vip3_parking_spaces.SCHEMA.md` 형식의 dict (center_point, angle, length, width 사용)
- 출력(CSV): run_id, corner_max_error_m, lateral_error_m, longitudinal_error_m, heading_error_deg, slot_margin_m, elapsed_time_s, collision_count, success_tsr, success_in_slot
- 성공 기준안: 차량 외곽이 칸 안에 들어가고 무충돌이면 성공(`success_in_slot`), 논문 비교용으로 `success_tsr`(횡 0.6 m, 종 1.0 m, 10° 이하, 무충돌)를 병기한다. 의견을 받고 싶다.