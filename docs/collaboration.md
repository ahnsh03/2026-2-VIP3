# 협업 규약 — `main` 직접 push

> 경로 경계는 [architecture.md](architecture.md)가 정본이다. 이 문서는 그 경계를
> 전제로 한 Git 운용만 다룬다.

팀은 **모여서 개발하지 않고 각자 비대면으로 진행**한다. 옆에서 "이 파일 지금 건드릴게"라고
말할 수 없다는 뜻이므로, 경로 오너십과 잦은 push가 그만큼 더 중요하다.

## 1. 기본 흐름

```bash
git checkout main
git pull                      # 항상 최신 main에서 시작

# 자기 오너십 경로만 수정
git add <내-영역-경로>
git commit -m "feat(parking): 후진 진입 경로 생성 골격"
git push origin main
```

- 작업 단위를 **짧게** 끊고 **자주** push한다. 오래 들고 있을수록 merge가 비싸진다.
- push가 거절되면 `git pull --rebase` 후 다시 push한다.
- `git add .` 대신 이번 작업 경로만 stage한다.

## 2. 브랜치·PR을 쓰는 경우 (예외)

| 상황 | 행동 |
|------|------|
| 공용 message·launch·Docker 등 전원에 영향 | 짧은 브랜치 + PR (리뷰 목적) |
| 남의 오너십 경로를 고쳐야 함 | 사전 합의 후 PR 또는 pair |
| `main`을 깨뜨릴 위험이 있는 실험 | 실험 브랜치 — 단, 며칠 내로 합치거나 폐기 |

몇 주치를 쌓아두는 개인 브랜치는 만들지 않는다.

## 3. 커밋 메시지

```
feat(scope): 기능 추가
fix(scope): 버그 수정
refactor(scope): 동작 유지한 구조 변경
docs: 문서만 변경
chore(scope): 빌드·설정
```

`scope`에는 **오너십 영역**을 넣는다 (`perception`, `parking`, `control`, `bridge` 등).
히스토리만 봐도 누가 어디를 건드렸는지 추적된다.

## 4. push 전 확인

- [ ] 최신 `main`에서 시작했다 (`git pull`)
- [ ] 이번 작업과 무관한 diff가 없다 (`git status --short`, `git diff --staged`)
- [ ] `build/`, `devel/`, bag, checkpoint, 비밀값, 호스트 절대경로가 없다
- [ ] 공용 계약(message/topic)을 바꿨다면 consumer도 함께 확인했다
- [ ] 실행법·파라미터가 바뀌었다면 해당 문서도 갱신했다

## 5. `main`이 깨졌을 때

1. 팀 채널에 커밋 링크와 증상을 공유한다.
2. 작은 수정이면 바로 `fix(...)` 커밋으로 `main`에 올린다.
3. 원인이 불명확하면 `git revert <commit>`으로 먼저 복구한다.

공유 이력을 지우는 `git push --force`와 `git reset --hard` 후 강제 push는 하지 않는다.
