# twinlite_morai

기존 TwinLiteNet+ 데이터·학습 adapter. **현재 주차 인지 기준선은 SS-PSD**이며 이 코드는 필요할 때만 재사용한다.
[학습·평가 명령](../../../docs/commands.md#legacy) · [베이스라인](../../../docs/project/baseline.md)

`dataset.py`는 JSONL manifest와 head별 valid mask를 읽고, `losses.py`·`training.py`는 focal/Tversky와 학습 루프를 제공한다. 모델 입력은 3×384×640이며 geometry metadata로 원본 좌표를 복원한다. 이를 SS-PSD의 512² BEV·point/slot 라벨과 혼용하지 않는다.

upstream `TwinLiteNetPlus`는 `90f1b8695ae311d5123b05f8534b2e11e42499d2`에 고정하며 로더가 커밋을 검사한다. 기본 checkpoint는 `weights/twinlite/v5_binary_lane/best_mean.pt`, 다른 head 후보는 `v6_road_marking/best_mean.pt`다. 과거 ASMC 실험 수치·비교 분석은 현재 팀 실행 문서에서 제외했다.

## 실행 환경과 진입점

별도 `vip3-perception-train` 컨테이너는 `/data`를 읽기 전용으로, repo를 `/root/ws`로 연결한다. 학습 결과는 Git 제외 `artifacts/`에 쓴다. GPU와 Docker NVIDIA 지원이 필요하다.

| 스크립트 | 역할 |
|---|---|
| `scripts/smoke_twinlite_data_loss.py` | 작은 forward/backward로 데이터·loss 확인 |
| `scripts/train_twinlite_tiny.py` | 지정 프레임 tiny-overfit |
| `scripts/train_twinlite.py` | 본학습·resume·초기 가중치 로드 |
| `scripts/evaluate_twinlite.py` | 고정 checkpoint 평가 |
| `scripts/render_twinlite_predictions.py` | 예측 overlay 저장 |

split은 run 단위로 분리한다. tiny selection 파일의 프레임이 실제 dataset에 있어야 한다. 이 도구가 존재한다는 이유로 SS-PSD 학습 환경까지 준비됐다고 판단하지 않는다.
