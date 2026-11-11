# 베이스라인과 적용 범위

## 채택한 구성

| 파트 | 기준 | 이번 프로젝트에서 적용 |
|---|---|---|
| 주차칸 인지 | **SS-PSD**, *Advancing Real-World Parking Slot Detection with Large-Scale Dataset and Semi-Supervised Baseline* (2025) | 공개 가중치 → MORAI BEV 직접 라벨 → 소규모 추가 학습 비교 |
| 센서·SVM | **AVM-SLAM**, 2024 개정본의 Around View Monitor 절 | 전후좌우 4어안, 렌즈 모델·외부 파라미터·지면 투영·영상 합성 구조 |
| 추가 비교 | **DMPR-PS** (2019) | 방향 표식 점 기반 검출·슬롯 구성의 단순 비교 모델; 일정 여유 때만 |
| 경로·제어 | GT 목표부터 검증하는 기하 경로·저속 추종 | 강도균이 한 칸 전방주차를 먼저 구현. 학습 기반 E2E 주차 재현은 현재 범위 밖 |

SS-PSD는 CRPS-D 데이터셋과 함께 공개된 주차칸 검출 기준선이며 teacher–student 방식의 반지도 학습을 제안한다. **T-ITS는 학술지 IEEE Transactions on Intelligent Transportation Systems의 약자**다. 공개 모델 사용·지도 추가 학습·반지도 재현은 서로 다른 실험으로 기록한다. [논문](https://arxiv.org/abs/2509.13133), [공식 코드·가중치 안내](https://github.com/zzh362/CRPS-D)

AVM-SLAM 전체는 4어안뿐 아니라 IMU·휠 인코더를 이용한 SLAM이다. 우리는 AVM 전처리를 참고하며 SLAM 전체를 구현했다고 주장하지 않는다. 논문의 센서 구성은 4카메라 구조의 근거이고, 현재 Ioniq5 장착 좌표는 **우리 MORAI 설정**이다. 차량 옆·대각선 사각은 실제 영상으로 검증한다. [논문 IV-A](https://arxiv.org/html/2309.08180v2#S4.SS1)

DMPR-PS는 초기 조사 조건인 2024년 이후 논문에는 해당하지 않지만 팀이 별도 비교 대상으로 선택했다. 인지 기준선을 다시 TwinLite segmentation으로 변경하는 계획은 없다. [DMPR-PS 공식 코드](https://github.com/Teoge/DMPR-PS)

## 입력과 재현 환경

- 목표 입력은 **차량 주변 10×10m, 512×512 컬러 BEV**다. 지금 `/perception/svm/panel`과 `preview`는 차량 그림·문구가 들어간 진단 영상이며, 제한된 정차 자세에서 확인한 보정 프로필을 사용한다. 바로 학습 데이터로 사용하지 않는다.
- clean BEV와 valid mask, 보정 설정, 시각 정보를 함께 내보내는 경로를 완성한 후 모델 전처리의 색상 순서·정규화·차량 마스크를 맞춘다.
- 외부 소스는 저장소 밖에 둔다. 확인한 커밋은 SS-PSD `e516cc49a0bfaa198130b223a1978c42a2c0449d`, DMPR-PS `72dc056abe0a953049cc62aca0cc6b85f12b03f0`이다. 위치는 `../external/parking/{CRPS-D,DMPR-PS}`이며 팀 clone에 자동 포함되지 않는다.
- 공개 가중치는 공식 저장소 링크에서 받는다. 사용하는 파일명·SHA-256·소스 커밋을 실험에 기록한다. **현재 팀 ROS용 SS-PSD 노드와 학습 환경은 아직 구현·배포되지 않았다.**
- 확인한 upstream은 추론·평가의 GPU 번호와 이미지 경로가 고정되어 있다. 단일 GPU와 MORAI 데이터에 맞춘 최소 adapter가 필요하다. [실행 준비 조건](../commands.md#ss-psd)에 구체적으로 적었다.

## 하드웨어와 비교 실험

4060 8GB / 노트북 3060 6GB에서 우선 **단일 이미지 추론**, 다음으로 소량 지도 추가 학습을 검증한다. batch 1–2부터 VRAM을 측정하고 필요하면 AMP를 적용한다. upstream 반지도 trainer의 기본 batch와 teacher–student 구성을 그대로 돌릴 수 있다고 가정하지 않는다. 지도 학습 경로도 원본 trainer의 옵션·loss를 확인해 별도 검증해야 한다.

| 비교 | 고정할 조건 | 기록 |
|---|---|---|
| 공개 가중치 vs MORAI 추가 학습 | 동일한 홀드아웃 run, 같은 SVM 설정·검출 기준 | 슬롯 precision/recall, 접점·입구 위치 오차, 지연, VRAM |
| GT 목표 vs 인지 목표 | 같은 칸·시작 자세·경로/제어기 | 칸 내 정차, 충돌, 횡·종·방향 오차, 소요 시간 |
| DMPR-PS vs SS-PSD (선택) | 동일한 영상·라벨·매칭 기준 | 인식 지표와 추론 시간 |

훈련/검증/시험은 프레임이 아닌 **run 단위**로 나눈다. 5개 후보 칸에 일반화했다고 주장하려면 해당 칸·접근 자세를 분리해 시험한다. 구체적인 GT·라벨 제약은 [데이터와 평가](../system/data.md)를 따른다.
