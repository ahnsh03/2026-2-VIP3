# 관련 연구 — 자율주차 참고 목록

> **문서 역할:** 참고할 논문·코드·가중치·데이터셋 목록
> **담당:** 하수영 (초안: 안승현, 2026-09-23)

## 0. 코드는 각자 저장소 밖에 받는다

아래 저장소들은 **팀 저장소에 넣지 않는다.** 필요한 사람이 저장소 **밖**에 clone 해서
읽기 전용으로 참고한다. 권장 위치는 저장소와 나란한 `../external/parking/` 이다
([setup.md](setup.md) §3 의 폴더 구조).

```bash
mkdir -p ../external/parking && cd ../external/parking
git clone --depth 1 https://github.com/Teoge/DMPR-PS.git
git clone --depth 1 https://github.com/qintonguav/e2e-parking-carla.git
# … 필요한 것만
```

가져다 쓸 코드가 생기면 필요한 부분만 저장소 안으로 이식하고 출처를 커밋 메시지에 남긴다
([porting-from-asmc.md](porting-from-asmc.md) §5).

## 1. 코드 저장소

"확인한 커밋"은 2026-09-23 에 실제로 열어 본 버전이다. 원격이 바뀌었을 수 있다.

| 우선 | 저장소 | 무엇 | 입력 | 프레임워크 | 확인한 커밋 |
|---:|---|---|---|---|---|
| **1** | [e2e-parking-carla](https://github.com/qintonguav/e2e-parking-carla) | 시뮬레이터 기반 end-to-end 주차 (IV 2024). **우리와 조건이 가장 비슷하다** — 핀홀 카메라 4대, 시뮬에서 데이터 생성·평가 | RGB 4대 400×300 FOV 100 | PyTorch + CARLA 0.9.11 | `c3d0387` |
| **2** | [ParkingE2E](https://github.com/qintonguav/ParkingE2E) | 같은 팀의 실차판 (IROS 2024). **ROS1** 이라 우리 런타임과 구조가 같다 | 어안 4대 480×270 | PyTorch + ROS1 | `acec907` |
| **3** | [DMPR-PS](https://github.com/Teoge/DMPR-PS) | 주차 슬롯 검출의 사실상 표준 기준선 (ICME 2019). 가볍고 재현이 쉽다 | AVM 합성 1장 | PyTorch | `72dc056` |
| **4** | [gcn-parking-slot](https://github.com/Jiaolong/gcn-parking-slot) | DMPR-PS 후속. GNN 으로 코너 점을 슬롯으로 묶는다 (ICRA 2021) | AVM 합성 1장 | PyTorch | `f8c3b44` |
| 5 | [context-based-parking-slot-detect](https://github.com/dohoseok/context-based-parking-slot-detect) | 평행/직각 주차를 문맥으로 분기. **데이터셋(PIL-park)** 이 값어치 | AVM 합성 1장 | **TensorFlow 1.x** | `dd66450` |
| 참고 | [awesome-parking-slot-detection](https://github.com/lymhust/awesome-parking-slot-detection) | 논문 목록 (2020년까지) | — | — | `ed6b925` |
| 참고 | [CameraCalibration](https://github.com/dyfcalid/CameraCalibration) | 카메라 캘리브레이션 예제 | — | Python/OpenCV | `67ad1b9` |
| 참고 | [SurroundCameraCalib](https://github.com/OpenCalib/SurroundCameraCalib) | 서라운드 카메라 캘리브레이션 (266 MB) | — | C++ | `c0295a8` |
| 참고 | [Surround-View-System-using-4-Fisheye-Cameras-in-Raspberry-Pi-3](https://github.com/hanifizzudinrahman/Surround-View-System-using-4-Fisheye-Cameras-in-Raspberry-Pi-3) | 어안 4대 AVM 스티칭 예제 (59 MB) | — | Python/OpenCV | `14fbfc2` |

주의할 것:

- `e2e-parking-carla`, `context-based-parking-slot-detect` 는 **라이선스 표기가 없다.**
  코드를 이식하기 전에 저자에게 확인해야 한다
- `context-based-parking-slot-detect` 는 TensorFlow 1.x 라 우리 컨테이너(PyTorch)와 안 맞는다.
  데이터셋 목적으로만 본다
- **AVM 스티칭 코드는 가져올 필요가 없다.** `drivable_bev` 가 이미 카메라별 지면 호모그래피로
  `base_link` 격자에 투영하고 품질 가중 블렌딩까지 한다. 그게 곧 AVM 합성이다

## 2. 논문

### clone 안에 PDF 가 들어 있는 것

| 논문 | 파일 |
|---|---|
| E2E Parking (IV 2024) | `e2e-parking-carla/resource/E2E_APA_IV24_final.pdf` (슬라이드도 같은 폴더) |
| DMPR-PS (ICME 2019) | `DMPR-PS/DMPR-PS.pdf` |

### arXiv (무료)

| 논문 | 연도 | 링크 |
|---|---|---|
| **ParkingE2E: Camera-based End-to-end Parking Network** (IROS 2024) | 2024 | https://arxiv.org/abs/2408.02061 |
| **Attentional Graph Neural Network for Parking-slot Detection** (ICRA 2021) — `gcn-parking-slot` | 2021 | https://arxiv.org/abs/2104.02576 |
| **PSDet: Efficient and Universal Parking Slot Detection** (IV 2020) | 2020 | https://arxiv.org/abs/2005.05528 |
| **SPFCN: Select and Prune the FCN for Real-time Parking Slot Detection** | 2020 | https://arxiv.org/abs/2003.11337 |
| **End-to-End Trainable One-Stage Parking Slot Detection** | 2020 | https://arxiv.org/abs/2003.02445 |
| **VH-HFCN: Parking Slot and Lane Markings Segmentation on Panoramic Surround View** (IV 2018) | 2018 | https://arxiv.org/abs/1804.07027 |
| **Enhanced Parking Perception by Multi-Task Fisheye Cross-view Transformers** | 2024 | https://arxiv.org/abs/2408.12575 |
| **F2BEV: BEV Generation from Surround-View Fisheye Images** | 2023 | https://arxiv.org/abs/2303.03651 |
| **E2E Parking Dataset: An Open Benchmark for End-to-End Autonomous Parking** | 2025 | https://arxiv.org/abs/2504.10812 |
| **AVP-SLAM: Semantic Visual Mapping and Localization in the Parking Lot** (IROS 2020) | 2020 | https://arxiv.org/abs/2007.01813 |

### 학회·저널 (학교 계정이 필요할 수 있다)

| 논문 | 링크 |
|---|---|
| DMPR-PS (ICME 2019) | https://ieeexplore.ieee.org/document/8784735 |
| **Context-Based Parking Slot Detection With a Realistic Dataset** (IEEE Access 2020) | https://ieeexplore.ieee.org/document/9199853 |
| Vision-Based Parking-Slot Detection: DCNN + ps2.0 benchmark (TIP 2018) | https://cslinzhang.github.io/deepps/ |
| **Review of Vision-Based Deep Learning Parking Slot Detection on Surround View Images** (2023, **서베이**) | https://www.ncbi.nlm.nih.gov/pmc/articles/PMC10422310/ |

**처음 읽을 3편:** 서베이(마지막 항목) → E2E Parking (IV 2024) → ParkingE2E (IROS 2024).
이유는 [roadmap.md](roadmap.md) §7.

## 3. 사전학습 가중치

| 저장소 | 가중치 | 링크 |
|---|---|---|
| DMPR-PS | detector weights | https://drive.google.com/open?id=1OuyF8bGttA11-CKJ4Mj3dYAl5q4NL5IT |
| e2e-parking-carla | pretrained (성공률 약 75 %) | https://drive.google.com/file/d/1XOlzBAb9W91R6WOB-srgdY8AZH3fXlML/view |
| ParkingE2E | pretrained + test data + demo rosbag | 저장소 README 안의 Google Drive 링크 |
| gcn-parking-slot | Model0 / Model1 (ps2.0) | 저장소 README 안의 Baidu 링크 (중국 계정 필요) |

## 4. 데이터셋

| 이름 | 내용 | 링크 |
|---|---|---|
| **ps2.0 (Tongji)** | AVM 합성 영상 + 슬롯 라벨. DMPR-PS·gcn-parking-slot 의 학습셋 | https://cslinzhang.github.io/deepps/ |
| **PIL-park** | 문맥(평행/직각) 라벨 포함 | `context-based-parking-slot-detect` README |
| 다양한 장면 주차 슬롯 | | https://github.com/wuzzh/Parking-slot-dataset |

> 전부 **실차 AVM 합성 영상**이다. MORAI KATRI 영상과 텍스처·조명·선 두께가 다르다.
> 사전학습·비교 기준으로 쓰고, 부족하면 우리 데이터로 다시 학습한다 ([roadmap.md](roadmap.md) §3).

## 5. 이 목록에서 나온 결론이 적힌 곳

| 내용 | 문서 |
|---|---|
| 논문 두 편의 실제 카메라 장착값과 우리 SVM 배치 대조 | [sensors.md](sensors.md) §5 "외부 검증" |
| TwinLiteNet+ 가 주차에 맞지 않는 이유 · DMPR-PS 스케일 정합 | [roadmap.md](roadmap.md) §2 |
