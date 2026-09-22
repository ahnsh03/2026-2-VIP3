# TwinLiteNet+ MORAI training adapter

> **역할:** Active Module — MORAI manifest, TwinLiteNet+ loader, 2-head loss와 학습/eval 연결
> **통합 경계:** [Perception 개발·통합 가이드](../../../docs/packages/asmc_perception/README.md)
> **현재 live 후보:** Medium, lane+drivable, v5 actor-negative(B) 18 epoch EMA
> **다음 학습 후보:** Medium, drivable+4-class road-marking, v6
> **데이터 정본:** [perception-dataset-labeling-split.md](../../../docs/contracts/perception/dataset-labeling-split.md)
> **평가 기록:** [perception-v4-training-evaluation.md](../../../docs/records/perception/twinlitenet-v4-evaluation.md)
> **다음 단계:** [인지 융합 로드맵 §7](../../../docs/packages/asmc_perception/roadmap.md#7-카메라-live-개발과-평가-계획)

`$ASMC_DATA/dataset_versions/<version>/<split>.jsonl`을 읽어 run/view provenance를
유지한 채 세 카메라 표본을 하나의 공유 가중치 모델에 공급한다.

## 현재 상태 요약

| 항목 | 값 |
|---|---|
| live dataset | `twinlite_morai_v5_marking_actor_negative` |
| next dataset | `twinlite_morai_v6_road_marking_actor_negative` |
| train/val | 26,001 / 1,404 samples |
| 입력 | `384x640`, view별 aspect-ratio 유지 letterbox |
| live heads | `drivable`, binary `lane` |
| next heads | `drivable`, `road_marking={background,white,yellow,stopline}` |
| 모델 | TwinLiteNet+ Medium, pinned upstream |
| live 후보 | `twinlite_medium_morai_v5_actor_negative_init_ema_b12_e18_20260906/best_mean.pt` |
| 공통 GT 평가 | drivable `0.98364`, lane `0.89332`, mean `0.93848` |
| 다음 작업 | v6 4-class 본학습과 class/view별 평가; live 기본은 검증 전까지 B 유지 |

이 README는 모듈 사용법을 다룬다. 데이터 정책의 근거와 run 목록은 데이터 계약에,
실험 수치와 한계는 평가 기록에만 추가한다.

## 데이터 계약

한 JSONL 행은 한 카메라 프레임의 학습 계약이다.

- `image`: `$ASMC_DATA` 기준 원본 Intensity PNG 상대경로
- `targets.drivable`: 미리 letterbox한 `{0,1}` 정답
- 활성 task에 따라 `targets.lane={0,1}` 또는
  `targets.road_marking={0:background,1:white,2:yellow,3:stopline}`
- `valid_masks.lane`: 원본 영상 영역 1, 패딩 영역 0
- `valid_masks.drivable`: A에서는 actor ignore 제외, B에서는 전체 기하 유효 영역
- `auxiliary_targets`: `drivable_ignore`와 원본 해상도 `actor_occupancy_gt` 경로
- `geometry`: RGB에 적용할 resize/padding과 추론 결과 복원 정보
- `run_id`, `view`, `frame_id`: split 누수 검사와 조건별 평가용 provenance

RGB는 로딩 시 alpha를 제거하고 BGR→RGB 변환 후 `geometry`와 동일하게 letterbox한다.
반환 tensor는 다음과 같다.

```text
image              float32 [3,384,640], range [0,1]
targets.drivable   int64   [384,640], class id {0,1}
targets.lane       int64   [384,640], class id {0,1}       # binary task
targets.road_marking int64 [384,640], class id {0,1,2,3}   # marking task
valid_masks.<active head> bool [384,640]
valid_masks.drivable   bool    [384,640]
```

## 모델·loss 연결

공식 TwinLiteNet+의 `(drivable_logits, secondary_logits)` 반환 순서와 같은 이름의 dict
출력을 모두 지원한다. Binary task는 `[B,2,H,W]` 두 출력이고, road-marking task는
`[B,2,H,W]`와 `[B,4,H,W]`다. 외부 checkout은 수정하지 않고 팀 adapter가 `out_ll`
마지막 블록만 4-class로 교체한다.

```python
from torch.utils.data import DataLoader
from twinlite_morai import (
    MaskedTwinLiteLoss,
    MoraiTwinLiteDataset,
    move_batch_to_device,
    twinlite_training_step,
)

dataset = MoraiTwinLiteDataset("/data", split="train")
loader = DataLoader(dataset, batch_size=16, shuffle=True, num_workers=8)
criterion = MaskedTwinLiteLoss()

for batch in loader:
    batch = move_batch_to_device(batch, device)
    optimizer.zero_grad(set_to_none=True)
    outputs, losses = twinlite_training_step(model, batch, criterion)
    losses["loss"].backward()
    optimizer.step()
```

Loss는 head별 focal + Tversky의 합이다. Road-marking은 softmax 기반 multiclass
focal/Tversky이며 선택적으로 `--road-marking-class-weights BG WHITE YELLOW STOPLINE`을
받는다. 기본 alpha/gamma는 TwinLiteNet+ 설정을
유지하되, 원본의 front 전용 `[:, :, 12:-12]` crop 대신 head별 `valid_masks`를 적용한다.
따라서 front의 상하 padding과 left/right의 좌우 padding은 두 head에서 제외되고,
차량·보행자·obstacle 픽셀은 A에서만 drivable loss/metric에서 제외된다. B에서는 해당
픽셀을 target 0, valid 1로 학습한다. v1의 공통 `valid_mask` manifest도 하위 호환한다.

## 실제 데이터 smoke

학습 컨테이너에서 데이터 두 장을 읽고 임시 2-head network로 forward/backward한다.

```bash
cd /root/ws
python3 scripts/smoke_twinlite_data_loss.py \
  --data-root /data --dataset-version twinlite_morai_v1 \
  --split train --batch-size 2
```

이 스크립트의 network는 배관 검사용이며 실제 TwinLiteNet+ 모델이 아니다.

## 공식 external 모델 연결과 tiny-overfit

공식 모델 소스는 팀 레포에 복사하지 않고 로컬 워크스페이스의
`external/baselines/TwinLiteNetPlus`에 유지한다. 학습 컨테이너에서는 이를
`/opt/baselines/TwinLiteNetPlus`로 읽기 전용 마운트한다. Loader는 실행 전에 upstream
commit이 `90f1b8695ae311d5123b05f8534b2e11e42499d2`인지 확인하고 다르면 중단한다.

2026-09-07 v6 검증에서는 날씨 2종×view 3종마다 white/yellow/stopline이 많은 표본을
하나씩 택한 18장 selection을 사용했다. VS Code 터미널이나 WSL 원격 연결이 끊겨도
프로세스가 종료되지 않도록 `docker exec -d` 기반 job runner로 실행한다.

```bash
./scripts/docker_perception_up.sh up

./scripts/docker_training_job.sh start \
  twinlite_v6_tiny_balanced18_e80_20260907 -- \
  python3 scripts/train_twinlite_tiny.py \
  --data-root /data \
  --dataset-version twinlite_morai_v6_road_marking_actor_negative \
  --selection-file config/twinlite_v6_tiny_overfit_samples.json \
  --config medium --epochs 80 --batch-size 3 --num-workers 0 --lr 5e-4 \
  --init-checkpoint \
  artifacts/perception_eval/runs/twinlite_medium_morai_v5_actor_negative_init_ema_b12_e18_20260906/best_mean.pt \
  --init-weights ema \
  --output-dir \
  artifacts/perception_eval/runs/twinlite_medium_morai_v6_tiny_balanced18_init_b_e80_20260907

./scripts/docker_training_job.sh status twinlite_v6_tiny_balanced18_e80_20260907
./scripts/docker_training_job.sh logs twinlite_v6_tiny_balanced18_e80_20260907 40
```

80 epoch, AMP, 별도 class weight 없음으로 137.4초가 걸렸다. Loss는
`1.55734 → 0.82984`, drivable IoU는 `0.98441 → 0.99029`, road-marking foreground
mIoU는 `0.03041 → 0.79188`이었다. 최종 class IoU는 white `0.76539`, yellow
`0.72264`, stopline `0.88762`이며 18장 overlay에서 GT/prediction 위치와 색 매핑도
확인했다. 따라서 2→4 classifier 교체, label encoding, multiclass loss/metric과 CUDA
backward가 함께 동작한다.

결과의 `checkpoint.pt`, `latest.pt`, `metrics.json`, prediction panel은 Git에서 제외되는
`artifacts/perception_eval/runs/`에 저장한다. Tiny-overfit checkpoint는 배관 검증용이며
validation 성능이나 배포 모델로 사용하지 않는다.

긴 학습은 같은 runner를 사용하고 학습 중 `docker_perception_up.sh up`으로 컨테이너를
재생성하거나 Docker Desktop/WSL을 종료하지 않는다. `train_twinlite.py`는 매 epoch
optimizer, scheduler, AMP scaler, EMA까지 `latest.pt`에 원자적으로 저장하므로 예기치 않은
중단 뒤 같은 run 이름과 `--resume <run>/latest.pt`로 이어갈 수 있다. Dataset은 read-only,
산출물은 repo의 `artifacts/` write mount에 둔다. RTX 4060 Ti 8GB의 검증된 기본은
`384x640`, Medium, batch 12, worker 6, AMP다.

## 초기 v1 전체 데이터 학습 기록

RTX 4060 Ti 8GB에서는 `384x640` Medium 학습에 batch 12를 사용한다. Batch 24는
메모리 약 7.5GB를 점유하면서 처리량이 크게 저하됐으므로 사용하지 않는다. 유효 영상
영역은 거리에 따른 별도 가중치 없이 모든 픽셀을 동일하게 취급하고, letterbox padding만
`valid_mask=0`으로 loss에서 제외한다.

```bash
docker exec asmc-perception-train bash -lc '
  cd /root/ws
  python3 scripts/train_twinlite.py \
    --data-root /data --dataset-version twinlite_morai_v1 \
    --config medium --epochs 10 --batch-size 12 --num-workers 6 \
    --log-interval 50 \
    --run-name twinlite_medium_morai_v1_uniform_b12_e10_20260902
'
```

산출물은 `artifacts/perception_eval/runs/<run-name>/`에 둔다. `latest.pt`는 재개용,
`best_mean.pt`는 두 head 평균 IoU 기준 배포 후보, `best_drivable.pt`와
`best_lane.pt`는 head별 분석용이다. 각 checkpoint에는 raw model, EMA, optimizer,
scheduler, AMP scaler, upstream commit과 validation 결과가 함께 저장된다.

## v2/v3/v4 고정 평가와 재학습

`evaluate_twinlite.py`는 optimizer 없이 `eval()`/`no_grad()`로만 실행하며 checkpoint
SHA-256, raw/EMA 선택, split/run/view/weather별 foreground IoU를 기록한다.

```bash
docker exec asmc-perception-train bash -lc '
  cd /root/ws
  python3 scripts/evaluate_twinlite.py \
    --data-root /data --dataset-version twinlite_morai_v2_curated \
    --split val --split test \
    --checkpoint artifacts/perception_eval/runs/twinlite_medium_morai_v1_uniform_b12_e10_20260902/best_mean.pt \
    --output-dir artifacts/perception_eval/baselines/twinlite_medium_morai_v1_on_v2_curated_20260904
'
```

2026-09-04 기준 결과는 val 평균 head IoU `0.7542`, test `0.4499`다. 이 값은 v2
추가 학습 전 5pm OOD 기준선이다. 이후 5pm을 v3 train으로 이동했으므로 재학습 후 공정한
test로 재사용하지 않는다.

실제 재학습 버전은 `twinlite_morai_v4_comp8_train_val_sample_b`이다. 기존 12개 run
전체 train 26,001 samples와 이후 별도 수집한 sample-b 6개 run val 1,404 samples로
구성한다. 첫 sunny 11am의 날씨 전환 4 capture는 val manifest에서 명시적으로 제외했다.
기존 checkpoint의 EMA **모델 가중치만** 가져오고 optimizer/scheduler/EMA/best는 새로
시작하도록 `--init-checkpoint`와 `--init-weights`를 사용한다. `--resume`은 같은 run의
중단 재개 전용이며 `--init-checkpoint`와 함께 사용할 수 없다.

본학습 전 무학습 preflight:

```bash
docker exec asmc-perception-train bash -lc '
  cd /root/ws
  python3 scripts/train_twinlite.py \
    --data-root /data \
    --dataset-version twinlite_morai_v4_comp8_train_val_sample_b \
    --config medium --epochs 10 --batch-size 12 --num-workers 0 \
    --lr 2e-4 \
    --init-checkpoint artifacts/perception_eval/runs/twinlite_medium_morai_v1_uniform_b12_e10_20260902/best_mean.pt \
    --init-weights ema \
    --output-root artifacts/perception_eval/preflight \
    --run-name twinlite_medium_morai_v4_clean_init_ema_20260904 \
    --preflight-only
'
```

`--preflight-only`는 train/val에서 한 batch씩 no-grad forward만 수행하고
`training_performed=false`를 기록한 뒤 종료한다. v4 preflight는 train/val 각각 12 sample,
두 출력 `[12,2,384,640]`과 finite loss를 확인했다. 실제 2-head 재학습에서는 같은 명령에서
`--preflight-only`를 제거하고 output root를 `artifacts/perception_eval/runs`로 바꾼다.

기존 EMA의 v4 무학습 기준은 val 전체 평균 `0.8210`, front `0.7503`, foggy `0.7582`,
foggy 3pm `0.7167`이다. 재학습 checkpoint는 전체 평균뿐 아니라 이 세 취약 구간이
기준보다 회귀하지 않는지도 함께 확인한다.

v4 2-head 본학습 명령:

```bash
docker exec asmc-perception-train bash -lc '
  cd /root/ws
  python3 scripts/train_twinlite.py \
    --data-root /data \
    --dataset-version twinlite_morai_v4_comp8_train_val_sample_b \
    --config medium --epochs 10 --batch-size 12 --num-workers 6 \
    --lr 2e-4 --log-interval 50 \
    --init-checkpoint artifacts/perception_eval/runs/twinlite_medium_morai_v1_uniform_b12_e10_20260902/best_mean.pt \
    --init-weights ema \
    --output-root artifacts/perception_eval/runs \
    --run-name twinlite_medium_morai_v4_comp8_init_ema_b12_e10_20260904
'
```

학습 도중 중단되면 같은 run의 `latest.pt`를 `--resume`으로 재개한다.
`--init-checkpoint`는 새 학습 시작에만 사용하고 `--resume`과 함께 지정하지 않는다.

## 현재 18 epoch 배포 후보

10 epoch의 종료 LR을 이어받지 않고 동일 초기 EMA, seed, v4 manifest에서 18 epoch
polynomial schedule을 새로 수행했다. 학습과 frozen validation이 모두 정상 종료됐으며
현재 live 후보는 다음 파일이다.

```text
artifacts/perception_eval/runs/
  twinlite_medium_morai_v4_comp8_init_ema_b12_e18_20260904/
    best_mean.pt
```

고정 평가 결과:

```text
artifacts/perception_eval/analysis/
  twinlite_medium_morai_v4_e18_on_v4_val_20260905/
    metrics.json
    overlays_selected/
```

`best_mean.pt`에서는 `ema_state_dict`를 우선 사용한다. 현재 validation은 매 epoch model
selection에 사용했으므로 독립 test가 아니며, 단순 추가 resume보다 live 조건별 검증과
새 test run을 우선한다.

## 2026-09-07 actor 정책 선택과 4-class 준비

동일한 1,404-sample v5 validation과 정적 표면 공통 GT에서 A(actor-ignore)와
B(actor-negative)를 다시 비교했다. B는 도로 문맥 안 actor를 drivable로 잘못 통과시킨
비율을 `14.26% → 3.20%`로 낮췄다. 일반 도로 false-hole은 `1.304% → 1.353%`로
`+0.049%p` 증가했지만, 공통 GT drivable IoU는 `0.98332 → 0.98364`, lane IoU는
`0.89301 → 0.89332`로 회귀하지 않았다. 따라서 B를 다음 학습의 잠정 기본 정책으로
채택한다. 근거와 선택 overlay는 아래에 보존한다.

```text
artifacts/perception_eval/analysis/
  twinlite_actor_policy_ab_on_v5_val_20260907/
    metrics.json
    per_sample.jsonl
    overlays_selected/
```

4-class 활성 dataset은 B와 동일한 train/val run, frame exclusion, `384×640` cache를
사용하는 `twinlite_morai_v6_road_marking_actor_negative`다. 총 27,405 sample의 경로,
shape와 mask 값 범위를 전수 검사했고 활성 head는 `drivable,road_marking`이다. B
checkpoint로 초기화할 때 encoder와 drivable head를 포함한 322개 tensor는 가져오고,
클래스 수가 달라진 `out_ll`의 14개 tensor만 재초기화한다. 무학습 preflight에서 실제
train/val 두 장씩에 대해 `[2,2,384,640]`, `[2,4,384,640]` 출력과 finite loss를 확인했다.

```text
artifacts/perception_eval/preflight/
  twinlite_medium_morai_v6_road_marking_init_b_20260907/
```

균형 18장 v6 tiny-overfit은 위 결과로 통과했다. 이 selection은 희소한 정지선까지 배관을
검증하도록 의도적으로 농축한 표본이므로 일반화나 class 분포 추정에는 사용하지 않는다.
본학습과 새 validation 검증은 아직 수행하지 않았으며, 새 checkpoint가 검증되기 전 ROS
live 추론과 시각화 기본값은 v5 B binary 모델을 유지한다.

## live 패키지와의 경계

이 모듈은 TwinLiteNet+ 전용 model/checkpoint/output adapter를 제공한다. 다음에 만들
`camera_semantic_perception` ROS 패키지는 영상 구독, view 선택, latest-only buffer,
성능계측과 표준 probability topic을 담당한다. 모델을 교체할 때 ROS node를 다시 쓰지 않고
backend adapter만 추가하는 구조를 사용한다.
