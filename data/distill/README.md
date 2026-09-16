# 탐사 할당 지식 증류 데이터

explorer 컨테이너가 `/data/distill` 로 이 폴더를 마운트해서 **실행마다 파일 하나**를 쌓는다.
내용물은 저장소에 올라가지 않는다 (`.gitignore`). 자세한 설명은
[12장 9절](../../12_TASK_ALLOCATION.md#9-증류-데이터) 에 있다.

```
data/distill/
├── raw/                 # 원시 로그. 한 파일 = 한 실행 = {run_id}.jsonl
│   └── legacy/          # 기록 형식이 바뀌기 전 로그 (프롬프트·교사 원문 없음)
└── export/              # export_distill.py 결과 (다시 만들 수 있으니 지워도 된다)
```

```bash
python3 src/webots_goal_bridge/scripts/export_distill.py            # raw -> export
```

**raw 는 지우지 말 것.** export 는 언제든 다시 만들 수 있지만 raw 는 실험을 다시 돌려야 한다.
