# 베이스라인과 적용 범위

## 채택한 구성

| 파트 | 기준 | 이번 프로젝트에서 적용 |
|---|---|---|
| 주차칸 인지 | **GCN**, *Attentional Graph Neural Network for Parking Slot Detection* (2021) | 공개 가중치 → MORAI SVM BEV 직접 라벨 → 소규모 추가 학습 비교 |
| 센서·SVM | **AVM-SLAM**, 2024 개정본의 Around View Monitor 절 | 전후좌우 4어안, 렌즈 모델·외부 파라미터·지면 투영·영상 합성 구조 |
| 추가 비교 | **DMPR-PS** (2019) | 방향 표식 점 기반 검출·슬롯 구성의 단순 비교 모델; 일정 여유 때만 |
| 경로·제어 | GT 목표부터 검증하는 기하 경로·저속 추종 | 강도균이 한 칸 전방주차를 먼저 구현. 학습 기반 E2E 주차 재현은 현재 범위 밖 |

GCN upstream은 PS2.0 기반 attentional graph neural network 주차칸 검출기다. 공개 가중치 모델 적용과 MORAI 직접 라벨 기반 추가 학습은 별도 실험으로 기록한다. [논문](https://arxiv.org/abs/2104.02576), [공식 코드·가중치 안내](https://github.com/Jiaolong/gcn-parking-slot)

AVM-SLAM 전체는 4어안뿐 아니라 IMU·휠 인코더를 이용한 SLAM이다. 우리는 AVM 전처리를 참고하며 SLAM 전체를 구현했다고 주장하지 않는다. 논문의 센서 구성은 4카메라 구조의 근거이고, 현재 Ioniq5 장착 좌표는 **우리 MORAI 설정**이다. 차량 옆·대각선 사각은 실제 영상으로 검증한다. [논문 IV-A](https://arxiv.org/html/2309.08180v2#S4.SS1)

DMPR-PS는 초기 조사 조건인 2024년 이후 논문에는 해당하지 않지만 팀이 별도 비교 대상으로 선택했다. 인지 기준선을 다시 TwinLite segmentation으로 변경하는 계획은 없다. [DMPR-PS 공식 코드](https://github.com/Teoge/DMPR-PS)

## 입력과 재현 환경

- 목표 입력은 **차량 주변 10×10m, 512×512 컬러 BEV**다. SVM은 이제 장식 없는 clean BEV 토픽도 발행한다. 보정 프로필은 제한된 정차 자세에서 확인한 preview이므로 여러 위치·방향에서 검증해야 한다.
- GCN adapter는 upstream의 512×512 BGR 입력 관례를 따르며, 슬롯 끝점을 픽셀·차량 기준 미터로 변환한다. SVM 보정 메타데이터와 source stamp를 결과에 보존한다.
- 외부 소스는 `../external/parking/gcn-parking-slot`에 두고 upstream 코드는 복사·수정하지 않는다. GCN 가중치는 저장소 외부에 두며 파일명·SHA-256·소스 커밋을 실험에 기록한다.
- ROS adapter와 clean BEV 배선은 구현했다. 공개 checkpoint 확보, 해당 환경에서 실행, MORAI 데이터 적합성 및 성능 검증은 남아 있다. [실행 준비 조건](../commands.md#gcn)을 따른다.

## 하드웨어와 비교 실험

4060 8GB / 노트북 3060 6GB에서 우선 **단일 이미지 추론**, 다음으로 소량 지도 추가 학습을 검증한다. batch 1–2부터 VRAM을 측정하고 필요하면 AMP를 적용한다. upstream 반지도 trainer의 기본 batch와 teacher–student 구성을 그대로 돌릴 수 있다고 가정하지 않는다. 지도 학습 경로도 원본 trainer의 옵션·loss를 확인해 별도 검증해야 한다.

| 비교 | 고정할 조건 | 기록 |
|---|---|---|
| 공개 가중치 vs MORAI 추가 학습 | 동일한 홀드아웃 run, 같은 SVM 설정·검출 기준 | 슬롯 precision/recall, 접점·입구 위치 오차, 지연, VRAM |
| GT 목표 vs 인지 목표 | 같은 칸·시작 자세·경로/제어기 | 칸 내 정차, 충돌, 횡·종·방향 오차, 소요 시간 |
| DMPR-PS vs GCN (선택) | 동일한 영상·라벨·매칭 기준 | 인식 지표와 추론 시간 |

훈련/검증/시험은 프레임이 아닌 **run 단위**로 나눈다. 5개 후보 칸에 일반화했다고 주장하려면 해당 칸·접근 자세를 분리해 시험한다. 구체적인 GT·라벨 제약은 [데이터와 평가](../system/data.md)를 따른다.
