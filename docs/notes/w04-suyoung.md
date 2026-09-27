# <4>주차 작업 기록 — <하수영>

**기간:** 2026-09-21 ~ 09-27  (개강 08/31 기준 주차 → [../course.md](../course.md) §3)

## 이번 주에 한 일

- AVM or BEV 이미지가 input으로 들어왔을 때 빈 주차공간 (1 slot)을 탐지하고 해당 주차공간의 좌표를 return 하는 딥러닝 모델을 찾음, 논문을 기반으로 여러 모델들의 factor들을 비교함

## 알게 된 것 / 공부한 것

참고한 자료(논문, 문서, 저장소)가 있으면 링크를 남긴다.

- E2E Parking: Autonomous Parking by the End-to-end Neural Network on the CARLA Simulator

| 항목 | E2E Parking |
|---|---|
| **입력 형태** | 차량 주변 카메라 여러 장 + 카메라 보정 정보 |
| **출력 표현** | 조향·속도 제어 신호. 슬롯 코너나 박스는 직접 출력하지 않음 (End to End 모델) |
| **사전학습 가중치** | 공개됨. 단, 공개 모델 성공률은 약 75%이며 최고 성능 모델은 아님 |
| **학습 데이터** | CARLA에서 사람이 직접 주차하며 카메라 영상과 제어 데이터를 수집 |
| **모델 크기** | 학습 : NVIDIA Tesla V100 GPU 배치 사이즈(Batch Size) 12, 22K 프레임 데이터셋(150 에포크) 96시간 소요 <br>추론 : NVIDIA Quadro RTX 5000 GPU 1스텝당 평균 추론 시간 \*\*74.92ms(약 13Hz)\*\* 기록 |
| **프로젝트 적합도** | **슬롯 위치를 전달하는 용도와는 출력 형태가 맞지 않음.** 주차 제어 시연용에 적합 |
| **슬롯 검출 정확도 PS2.0** | |
| **주의사항** | 라이선스 X |

출처: [논문 코드, 가중치 저장소](https://github.com/qintonguav/e2e-parking-carla)

- ParkingE2E

| 항목 | ParkingE2E |
|---|---|
| **입력 형태** | **Fisheye 카메라 4대 영상** + 목표 위치 |
| **출력 표현** | 픽셀 마스크나 슬롯 박스가 아니라 **주차 경로의 waypoint들** (End to End 모델) |
| **사전학습 가중치** | 공개된 추론용 가중치와 테스트 데이터가 있음. EfficientNet 사전학습 가중치도 사용 |
| **학습 데이터** | 전문가 주차 경로를 모방학습. ROS bag으로 우리 데이터를 만들 수 있지만, 카메라 영상과 주행 경로 기록이 필요 |
| **모델 크기** | 파라미터 수와 8 GB GPU 학습 가능 여부는 저장소에 명시되지 않음. 직접 확인 필요 |
| **프로젝트 적합도** | 4대 fisheye 카메라와 경로 계획이 목표라면 적합. **슬롯 코너나 빈자리만 검출하려는 과제에는 출력이 맞지 않음** |
| **슬롯 검출 정확도 PS2.0** | |
| **주의사항** | |

출처: [ParkingE2E 저장소](https://github.com/qintonguav/ParkingE2E)

- **VPS-NET**

| 항목 | VPS-Net |
|---|---|
| **입력 형태** | 차량 주변 카메라 영상으로 만든 **AVM/BEV 합성 이미지 1장** |
| **출력 표현** | 주차선 표시점과 슬롯 방향을 찾아 **주차 슬롯 위치를 추론**하고, 빈자리 여부도 분류 |
| **사전학습 가중치** | 탐지·분류 가중치 **공개**. PS2.0·PSV 데이터셋 라벨도 제공 |
| **학습 데이터** | 공개된 주차장 AVM 이미지와 주차 슬롯 라벨 사용. **우리 데이터로 추가 학습하려면 슬롯 표시점·빈자리 라벨 필요** |
| **모델 크기** | 논문에서 분류기는 비교 모델보다 파라미터가 적다고 설명하지만, RTX 4060 Ti 8 GB에서 학습 가능한지는 직접 확인 필요 |
| **프로젝트 적합도** | **슬롯 위치와 빈자리 여부를 전달하는 목적에 잘 맞음.** 단, 입력 AVM 이미지가 필요함 |
| **슬롯 검출 정확도 PS2.0** | precision **99.63%**, recall **99.31%**. 슬롯 점 위치 오차 **1.03 ± 0.64 px** |
| **주의사항** | 라이선스 X |

출처: [논문](https://www.mdpi.com/1424-8220/20/7/2138) · [코드·가중치·데이터 안내](https://github.com/weili1457355863/VPS-Net)

- **DMPR-PS**

| 항목 | DMPR-PS |
|---|---|
| **입력 형태** | 차량 주변 카메라로 만든 **surround-view/AVM 이미지** |
| **출력 표현** | 주차선의 **방향성 표시점**을 검출하고, 점들을 조합해 슬롯 위치·방향을 계산 |
| **사전학습 가중치** | 논문 재현용 가중치 공개 |
| **학습 데이터** | PS2.0 데이터와 라벨 공개. 자체 데이터는 방향성 표시점 라벨을 만들어 학습 가능 |
| **모델 크기** | 학습 : **Nvidia Titan Xp** 배치 크기(batch size) 24, 12 에포크(epoch) 조건으로 모델을 학습 <br> 추론 : 논문은 Titan Xp에서 프레임당 12 ms를 보고 |
| **프로젝트 적합도** | **슬롯 위치·방향을 전달하는 용도에 적합.** 단, 슬롯 전체 마스크가 아니라 표시점을 검출한 뒤 기하 규칙으로 슬롯을 구성 |
| **슬롯 검출 정확도 PS2.0** | precision **99.42%**, recall **99.37%** |
| **주의사항** | |

출처: [논문(IEEE)](https://ieeexplore.ieee.org/document/8784735/) · [코드·가중치·PS2.0 라벨](https://github.com/Teoge/DMPR-PS) · [커스텀 데이터 라벨링 도구](https://github.com/Teoge/MarkToolForParkingLotPoint)

- Yolo OBB

| 항목 | YOLO OBB |
|---|---|
| **입력 형태** | 일반 카메라 영상 또는 BEV 합성 이미지 1장 |
| **출력 표현** | 회전 박스(중심 좌표, 너비·높이, 회전각) ([Ultralytics OBB 문서](https://docs.ultralytics.com/tasks/obb/)) |
| **사전학습 가중치** | 기본 OBB 가중치 제공. 주차 슬롯용 가중치는 별도 확인 필요 ([모델 문서](https://docs.ultralytics.com/models/yolo11/)) |
| **학습 데이터** | 회전 박스 꼭짓점 좌표 라벨 필요. 기존 점 라벨은 변환해 사용할 수 있음 ([OBB 데이터 형식](https://docs.ultralytics.com/datasets/obb/)) |
| **모델 크기** | RTX 4060 Ti 8 GB에서의 학습 가능성은 모델·해상도·배치 크기에 따라 달라지며, 앞선 평가는 일반적인 추정 |
| **프로젝트 적합도** | 슬롯 위치와 방향 검출에 적합. 정확한 코너 좌표가 필요하면 박스 꼭짓점에서 계산 가능 |
| **슬롯 검출 정확도 PS2.0** | |
| **주의사항** | |

## 막힌 것

- 증상: DMPR-PS 모델 실행 시 CUDA 작동 X
- 시도한 것: Ubuntu 20.04 버전에 맞는 CUDA, PyTorch 호환 문제가 생기는 것으로 파악하고 wsl Linux 내에 별도의 가상 환경을 만들어 실행 했으나 실패 
- 다음 시도: --disable_cuda 코드를 이용해 CPU로 구동해 모델 실행 성공 

## 다음 주 계획

- CUDA 환경 재설정, 모델 코드 이식

## 팀 공유 사항

다른 사람이 알아야 할 결정, 인터페이스 변경, 요청 사항.

- **최종 모델 결정안** => **DMPR-PS**
- 선택 사유
    - rule based 로직의 주차를 구현하는 해당 프로젝트에서는 직접적인 슬롯 검출 과정이 보이고, 좌표 값이 return 되는 DMPR-PS가 가장 적합한 선택
    - VPS-NET은 현재 시나리오 가정에서는 불필요한 Occupancy detection 로직이 포함되어 있으므로 Training Data Labeling이 복잡, 라이선스 명시 X
    - E2E Model 들은 Output이 바로 제어 로직이므로 중간 검출 단계가 BlackBox, Pretrained 가중치가 공개되어있지만 정확도가 낮고, 모델의 크기 정보가 없음, 학습 시 데이터가 숙련된 운전자의 주차 경로임
    - Yolo OBB는 기본적으로 객체 인식 모델인데, 여러 Pretrained 가중치가 있지만 E2E 모델과 마찬가지로 주차 데이터 셋에 대한 가중치가 없음 -> 우리 데이터셋을 통해 커스텀 모델을 만들어야 할 가능성 높음 <br>(참고 https://github.com/danielbob32/ParkingSpace)
- 실행 결과 (DMPR-PS / PS 2.0) : [그림1](https://drive.google.com/file/d/1yn7LUN55zKC_9r4u-MNvEHJYzi6Kr60f/view?usp=sharing) [그림2](https://drive.google.com/file/d/1RVBReln2iIMUY63_ejQK6gwHjMplPUIn/view?usp=sharing)
