# twinlite_morai — TwinLiteNet+ 학습 adapter

> **문서 역할:** 패키지 가이드 — 학습·평가 절차
> **담당:** 하수영
> **⚠ 이 모델은 주차용 기본이 아니다.** 주차에는 BEV 입력 + 슬롯 엔티티 출력 모델이
> 맞다 ([roadmap.md](../../../docs/roadmap.md) §2). 여기 남긴 이유는 **학습·평가 배관이
> 통째로 돌아가는 상태**라 새 모델을 얹을 자리이고, 기하 비교군의 마스크 공급원이기 때문이다.
> **데이터 계약:** [src/data_collection/README.md](../../data_collection/README.md)
> **추론 패키지:** [camera_semantic_perception](../camera_semantic_perception/README.md)

`$VIP3_DATA/dataset_versions/<version>/<split>.jsonl` 을 읽어 run/view 출처를 유지한 채
**4대 카메라 표본을 하나의 공유 가중치 모델**에 공급한다. ROS 도 시뮬레이터도 필요 없다 —
디스크의 PNG 와 JSONL 만 읽는다.

## 지금 할 수 있는 것 / 없는 것

**데이터셋이 아직 없다.** 수집이 끝나기 전에 할 수 있는 것은 단위 테스트뿐이다.

```bash
./scripts/docker_train_up.sh build && ./scripts/docker_train_up.sh up
docker exec vip3-perception-train bash -lc '
  cd /root/ws && PYTHONPATH=src/perception python3 -m unittest discover \
    -s src/perception/twinlite_morai/test -p "test_*.py" -v'
```

그 다음 마일스톤은 학습이 아니라 **v5 를 KATRI 주차장 4카메라 live 에 붙여 눈으로 보는
것**이다 ([camera_semantic_perception](../camera_semantic_perception/README.md)).
쓸 만하면 학습 파이프라인은 골격만 확보해 두고 기반 논문 모델에 자원을 몰 수 있다.
VIP3 데이터로 처음부터 학습하려면 수집→라벨→학습에 **2~3주**가 든다 (7주 중 3주다).

## 체크포인트 두 개

| | v5 (기본) | v6 (업그레이드 후보) |
|---|---|---|
| 경로 | `weights/twinlite/v5_binary_lane/best_mean.pt` | `weights/twinlite/v6_road_marking/best_mean.pt` |
| sha256 | `5d233b52…` | `a26311b3…` |
| task | `drivable_lane_binary` | `drivable_road_marking_4class` |
| head | drivable, lane (binary) | drivable, road_marking (bg/white/yellow/stopline) |
| epoch | 18 | 11 |
| drivable IoU | 0.98364 | 0.98343 |
| mean | 0.93848 | **0.93949** |

**기본은 v5 다.** 숫자만 보면 v6 가 근소하게 낫지만 **right 뷰 yellow IoU 가 0.1805 로
무너져 있어** 클래스별 신뢰도가 고르지 않다. v5 는 ASMC live 설정 4곳이 전부 가리키던
파일이라 bring-up 이 깨졌을 때 "체크포인트 탓"을 배제할 수 있다.

**주차칸 선이 Semantic 에서 `white_lane` 으로 확인되는 순간 v6 가 명백히 낫다**
(white/yellow 분리). 그 확인이 v5/v6 선택과 주차 데이터셋 설계를 동시에 결정한다 —
[data_collection README](../../data_collection/README.md) 의 `audit_semantic_capture.py`.

두 체크포인트 모두 upstream pin 은 `90f1b8695ae311d5123b05f8534b2e11e42499d2` 다.

### upstream 핀은 풀지 않는다

`external_model.py` 가 실행 전에 `/opt/baselines/TwinLiteNetPlus` 의 commit 을 확인하고
다르면 중단한다. 세 곳에서 검사한다 — `--resume`, `--init-checkpoint`, `evaluate_twinlite.py`.
**핀이 안 맞으면 가져온 가중치를 아예 못 쓴다.** 호스트 clone 이 그 커밋인지 먼저 본다:

```bash
git -C ../external/baselines/TwinLiteNetPlus rev-parse HEAD
```

## 데이터 계약

한 JSONL 행이 한 카메라 프레임의 학습 계약이다.

```
image                     $VIP3_DATA 기준 상대경로 (원본 Intensity PNG)
targets.drivable          미리 letterbox 한 {0,1}
targets.lane              {0,1}                        binary task
targets.road_marking      {0,1,2,3}                    4-class task
valid_masks.<head>        head 별 유효 영역 (letterbox padding 은 0)
geometry                  추론 결과를 원본 좌표로 되돌리는 유일한 근거. 재계산 금지
run_id / frame_id / view  split 누수 검사와 조건별 평가용
```

반환 텐서:

```
image                  float32 [3,384,640]  range [0,1]
targets.<head>         int64   [384,640]
valid_masks.<head>     bool    [384,640]
```

Loss 는 head 별 focal + Tversky 다. 원본의 front 전용 `[:, :, 12:-12]` crop 대신 head 별
`valid_masks` 를 쓴다. 그래서 front/rear 의 상하 padding 과 left/right 의 좌우 padding 이
두 head 모두에서 제외된다.

## 명령

```bash
export VIP3_DATA="$VIP3_PROJECT/data"
./scripts/docker_train_up.sh up

# 1) 데이터 배관 smoke (2장 forward/backward)
docker exec vip3-perception-train bash -lc '
  cd /root/ws && python3 scripts/smoke_twinlite_data_loss.py \
    --data-root /data --dataset-version vip3_katri_parking_v1 --split train --batch-size 2'

# 2) tiny-overfit 게이트 — 본학습 전 필수. 배관이 살아있음을 증명한다 (ASMC 기준 137초)
./scripts/docker_training_job.sh start vip3_tiny_e80_$(date +%Y%m%d) -- \
  python3 scripts/train_twinlite_tiny.py \
    --data-root /data --dataset-version vip3_katri_parking_v1 \
    --selection-file config/perception/twinlite_tiny_overfit_samples.json \
    --config medium --epochs 80 --batch-size 3 --num-workers 0 --lr 5e-4 \
    --init-checkpoint /root/ws/weights/twinlite/v5_binary_lane/best_mean.pt --init-weights ema

# 3) 무학습 preflight — 데이터·모델·초기화가 붙는지만 본다
docker exec vip3-perception-train bash -lc '
  cd /root/ws && python3 scripts/train_twinlite.py \
    --data-root /data --dataset-version vip3_katri_parking_v1 \
    --config medium --epochs 18 --batch-size 12 --num-workers 0 --lr 2e-4 \
    --init-checkpoint /root/ws/weights/twinlite/v5_binary_lane/best_mean.pt --init-weights ema \
    --output-root artifacts/perception_eval/preflight \
    --run-name vip3_preflight_$(date +%Y%m%d) --preflight-only'

# 4) 본학습 (RTX 4060 Ti 8GB 검증 기본: 384x640 / medium / b12 / w6 / AMP)
./scripts/docker_training_job.sh start vip3_b12_e18_$(date +%Y%m%d) -- \
  python3 scripts/train_twinlite.py \
    --data-root /data --dataset-version vip3_katri_parking_v1 \
    --config medium --epochs 18 --batch-size 12 --num-workers 6 --lr 2e-4 --log-interval 50 \
    --init-checkpoint /root/ws/weights/twinlite/v5_binary_lane/best_mean.pt --init-weights ema \
    --output-root artifacts/perception_eval/runs --run-name vip3_b12_e18_$(date +%Y%m%d)

./scripts/docker_training_job.sh status vip3_b12_e18_$(date +%Y%m%d)
./scripts/docker_training_job.sh logs   vip3_b12_e18_$(date +%Y%m%d) 40

# 5) 고정 평가
docker exec vip3-perception-train bash -lc '
  cd /root/ws && python3 scripts/evaluate_twinlite.py \
    --data-root /data --dataset-version vip3_katri_parking_v1 --split val \
    --checkpoint /root/ws/weights/twinlite/v5_binary_lane/best_mean.pt \
    --output-dir artifacts/perception_eval/baselines/v5_on_katri_val'

# 6) 예측 패널 렌더 (RGB | GT | PRED)
docker exec vip3-perception-train bash -lc '
  cd /root/ws && python3 scripts/render_twinlite_predictions.py \
    --data-root /data --dataset-version vip3_katri_parking_v1 --split val \
    --checkpoint /root/ws/weights/twinlite/v5_binary_lane/best_mean.pt \
    --max-samples 24 --weights ema --output-dir artifacts/perception_eval/analysis/overlays'
```

**`docker_training_job.sh` 를 쓴다.** VS Code 터미널이나 WSL 원격 연결이 끊겨도 학습이
죽지 않는다. 학습 중에 `docker_ros_up.sh up` 으로 컨테이너를 재생성하거나 Docker
Desktop/WSL 을 종료하지 않는다.

`--resume` 은 **같은 run 의 중단 재개 전용**이고 `--init-checkpoint` 와 같이 못 쓴다.
매 epoch `latest.pt` 에 optimizer·scheduler·AMP scaler·EMA 까지 원자적으로 저장하므로
예기치 않게 끊겨도 같은 run 이름 + `--resume <run>/latest.pt` 로 이어간다.

`best_mean.pt` 에서는 `ema_state_dict` 를 우선 사용한다.

## 4뷰로 바뀐 것

1. 뷰 튜플이 `("all","front","left","right","rear")` — 코드 레벨의 유일한 blocker 였다
2. **후방은 학습 뷰다.** ASMC 의 4번 카메라는 lane/drivable 학습에서 제외돼 있었다
3. 광학이 전방과 같아 letterbox 기하도 같다 (`720×1280 → 360×640`, pad t/b 12 px)
4. 같은 프레임 수에서 표본이 3배 → **4배** (디스크·학습시간 +33 %)
5. `stopline`/`road_marking` 의 front-only 가정 제거

## 고친 버그 2건

- `smoke_twinlite_data_loss.py` — `valid_masks` dict 를 device 로 안 옮겨 `--device cuda` 에서
  device mismatch. `move_batch_to_device` 로 교체
- `evaluate_twinlite.py` — `HEADS=("drivable","lane")` 하드코딩 + 기본 task 때문에
  **v6 4-class 체크포인트를 평가할 수 없었다** (`road_marking logits need 1 or 2 channels,
  got 4`). `task_from_checkpoint` + `configure_twinlite_task` 로 교체

## 주차용으로 바꿔야 할 것 (기반 논문 확정 후)

as-is 포트를 먼저 끝내고, 아래는 **별도 dataset version** 으로 올린다. 기존 v1 을 덮어쓰지
않는다.

- **4-class 재정의**: `("background","parking_slot_line","lane_line","curb_or_wheelstop")`.
  `stopline` 은 주차에서 무의미하고 휠스톱·연석이 주차 종료 조건에 직결된다.
  클래스 수가 같아서 `ROAD_MARKING_CLASS_NAMES` 만 바꾸면 2→4 전이 로직을 그대로 쓴다
- **근거리 crop 을 valid mask 로.** 20 m 밖은 주차에서 노이즈다. 영상을 물리적으로 자르면
  letterbox 계약이 깨지므로, `valid_masks.<head>` 에서 먼 영역을 0 으로 만든다.
  **코드 변경 0줄, mask baker 만 수정.** 이 포트에서 가장 값싼 주차 특화다
- **후방 뷰 가중.** 주차는 후진이 기본이다. run 단위 split 원칙은 유지하되 view 별 class
  IoU 를 반드시 분리 보고한다
- **슬롯 점유(빈 칸/찬 칸)는 segmentation 으로 풀지 않는다.** `actor_occupancy_gt` 를
  주차칸 폴리곤과 교차시키는 후처리가 자연스럽다

## 확인해야 할 것

- [ ] **주차칸 선이 MORAI Semantic 에서 무슨 색인가** — v5/v6 선택과 데이터셋 설계를 동시에 결정
- [ ] run_id 조건 축. ASMC 의 (날씨 × 시각) 대신 (슬롯 위치 × 진입 방향 × 인접 차량)이
      맞는 축이다. 정해지면 `evaluate_twinlite.py` 의 `_condition()` 을 고친다
- [ ] **GPU 접근성.** RTX 4060 Ti 가 안승현 로컬에만 있으면 김동현·하수영이 학습을 못 돌린다.
      tiny-overfit(137초)은 어디서든 되지만 18 epoch 본학습은 v5 가 1,784초, v6 가 6,470초다.
      팀 공용 학습 머신·시간대 합의가 필요하다
