# 13. 임무계획 지식 증류 (교사 LLM → 작은 모델)

> 📖 [책 목차](Readme.md#-목차) · ← [12. 탐사 할당](12_TASK_ALLOCATION.md)

[12장](12_TASK_ALLOCATION.md)의 탐사 할당을 **큰 교사 모델이 아니라 작은 학생 모델**이
하게 만드는 절차다. 데이터는 이미 쌓이고 있다 ([12장 9절](12_TASK_ALLOCATION.md#9-증류-데이터)).

## 목차
- [1. 왜 증류하나](#1-왜-증류하나)
- [2. 전체 흐름](#2-전체-흐름)
- [3. 데이터 모으기](#3-데이터-모으기)
- [4. 데이터셋 만들기](#4-데이터셋-만들기)
- [5. 학생 모델 고르기](#5-학생-모델-고르기)
- [6. 학습](#6-학습)
- [7. 평가](#7-평가)
- [8. 서빙하고 할당기에 꽂기](#8-서빙하고-할당기에-꽂기)
- [9. 함정](#9-함정)

---

## 1. 왜 증류하나

실측이 이유를 그대로 보여준다 ([12장 6절](12_TASK_ALLOCATION.md#6-측정-결과)).

| | 교사 (nemotron-3-super, NIM) |
|---|---|
| 호출 한 번 | 평균 58~97초 |
| 주기(60초) 초과 | 43회 |
| 예산에서 잘린 호출 | 175회 중 113회 (65%) |
| 폴백(베이스라인으로 물러남) | 주기 96회 중 19회 |

**판단은 쓸 만한데 속도가 못 쓸 수준이다.** 교사가 낸 유효한 답 77개 중 62개가 거리
베이스라인과 다른 배정이었다. 같은 판단을 **수백 ms 안에** 내는 모델이 있으면 온라인
계획기로 쓸 수 있다 — 그게 증류의 목표다.

부수적으로, 학생은 **로컬에서 돈다.** API 한도·과금·503(실측 11회)에서 자유롭다.

## 2. 전체 흐름

```
 실험 (run_explore_compare.sh)
   └─▶ data/distill/raw/{run_id}.jsonl      교사 프롬프트·응답·결과가 그대로
        └─▶ export_distill.py
             └─▶ data/distill/export/sft_{train,val}.jsonl
                  └─▶ QLoRA 학습 (Colab / Kaggle / 3060 Ti)
                       └─▶ vLLM·Ollama 로 서빙 (OpenAI 호환)
                            └─▶ 할당기에 base_url/model 만 바꿔 꽂기
                                 └─▶ run_explore_compare.sh 로 재평가 ↺
```

마지막 고리가 중요하다. **학생을 다시 시뮬에 꽂아 같은 지표로 재는 것**까지가 한 바퀴다.

## 3. 데이터 모으기

```bash
# Webots 에서 월드를 열고 ▶ 한 뒤 (기본 실험: oneroom 내부 2대)
src/webots_goal_bridge/scripts/run_explore_compare.sh mac 4
```

한 실행에서 교사 유효 샘플이 **약 13개** 나온다 (LLM 시행 6회에서 77개). 지금 갖춘 양은
소규모 LoRA 를 한 번 돌려 볼 정도이고, 쓸 만한 학생을 만들려면 **수천 개** 급이 필요하다.

양보다 먼저 봐야 할 것은 **다양성**이다. 지금 데이터는 전부 같은 월드·같은 편대·같은
출발점이라 학생이 "이 방"만 외울 수 있다. 늘리는 축:

| 축 | 방법 |
|---|---|
| 월드 | [02장](02_WORLD_GEN.md)으로 시드를 바꿔 생성 (`--single-room` 말고 방·복도 있는 것도) |
| 편대 | 매니페스트에 Spot 추가, 3대 구성 (맥북은 2대가 한계 — [10장](10_MAP_MERGE.md)) |
| 출발점 | 매니페스트 좌표를 바꾼다 (지금은 서쪽 출입구 안쪽 고정) |
| 탐사 범위 | `EXPLORE_BOUNDS` 로 구역을 나눠 부분 정찰 |

`EXPLORE_TAG` 에 축을 적어 두면 나중에 섞거나 거를 때 파일명만으로 갈린다.

## 4. 데이터셋 만들기

```bash
python3 src/webots_goal_bridge/scripts/export_distill.py                      # 기본
python3 src/webots_goal_bridge/scripts/export_distill.py --include-reasoning  # 추론까지
python3 src/webots_goal_bridge/scripts/export_distill.py --min-gain 300       # 결과가 좋았던 판단만
```

한 줄은 그대로 SFT 에 넣는 형식이다.

```json
{"messages": [{"role": "user", "content": "<12장 3절 관측이 박힌 프롬프트 전문>"},
              {"role": "assistant", "content": "{\"assignments\": [...], \"reason\": \"...\"}"}],
 "meta": {"run_id": "...", "round": 7, "prompt_version": "alloc-v2",
          "agrees_with_baseline": false, "outcome": {"gain_per_min": 412.0}}}
```

- **`prompt_version` 이 섞이지 않게 거른다.** v1 은 "분리 최대화", v2 는 "목표 수·이동거리
  최소화" 라 지시가 다르다. 섞어 학습하면 학생이 어느 쪽을 배울지 모른다
- `outcome.gain_per_min` 은 그 판단 뒤 실제로 얼마나 더 탐사됐는지다. `--min-gain` 으로
  거르면 **거부 샘플링**이 된다 (교사의 나쁜 판단을 빼고 배운다)
- `--include-reasoning` 은 `<think>…</think>` 를 답 앞에 붙인다. 추론까지 배우면 품질이
  오를 수 있지만 **출력 토큰이 늘어 지연이 커진다** — 증류의 목적과 정면으로 부딪친다.
  먼저 추론 없이 해 보고, 부족하면 켠다

## 5. 학생 모델 고르기

입력이 **한국어 지시 + JSON 관측**이고 출력이 **짧은 JSON** 이다. 필요한 것은 긴 추론이
아니라 **형식을 지키는 안정성**이다.

| 후보 | 크기 | 메모 |
|---|---|---|
| Qwen2.5-1.5B-Instruct | 1.5B | 한국어·JSON 둘 다 무난. 3060 Ti 8GB 에서 QLoRA 가능 |
| Qwen2.5-3B-Instruct | 3B | 품질 여유. Colab T4(16GB) QLoRA 권장 |
| Llama-3.2-3B-Instruct | 3B | 한국어는 Qwen 보다 약한 편 |

프롬프트가 약 3.5KB(**1.5k 토큰 안팎**)라 4k 컨텍스트면 충분하다.

**하드웨어.** 3060 Ti 8GB → 1.5B QLoRA(4bit, `bf16`, 배치 1 + 누적)가 현실적이고 3B 는
빡빡하다. Colab T4 16GB 무료 티어면 3B QLoRA 가 돈다. 맥북(MPS)은 학습용으로 쓰지 않는다 —
`bitsandbytes` 4bit 가 안 돌아간다.

## 6. 학습

```bash
pip install "transformers>=4.44" "trl>=0.9" peft bitsandbytes accelerate datasets
```

```python
# train_student.py — Colab/Kaggle 셀에 그대로 붙여도 된다
from datasets import load_dataset
from peft import LoraConfig
from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig
from trl import SFTConfig, SFTTrainer

BASE = "Qwen/Qwen2.5-1.5B-Instruct"
ds = load_dataset("json", data_files={"train": "sft_train.jsonl", "val": "sft_val.jsonl"})

tok = AutoTokenizer.from_pretrained(BASE)
model = AutoModelForCausalLM.from_pretrained(
    BASE, device_map="auto", torch_dtype="bfloat16",
    quantization_config=BitsAndBytesConfig(load_in_4bit=True,
                                           bnb_4bit_compute_dtype="bfloat16"))

trainer = SFTTrainer(
    model=model,
    train_dataset=ds["train"], eval_dataset=ds["val"],
    peft_config=LoraConfig(r=16, lora_alpha=32, lora_dropout=0.05,
                           target_modules="all-linear", task_type="CAUSAL_LM"),
    args=SFTConfig(
        output_dir="student", num_train_epochs=3,
        per_device_train_batch_size=1, gradient_accumulation_steps=8,
        learning_rate=1e-4, warmup_ratio=0.03, lr_scheduler_type="cosine",
        max_seq_length=4096, packing=False,          # 🚨 샘플을 이어 붙이지 않는다
        # 🚨 답(assistant)만 학습한다. 프롬프트까지 배우면 관측을 외우는 데 용량을 쓴다
        assistant_only_loss=True,
        bf16=True, logging_steps=5, eval_strategy="epoch", save_strategy="epoch"),
)
trainer.train()
trainer.save_model("student")        # LoRA 어댑터
```

샘플이 수백 개 규모면 **에폭 2~3, LoRA r=16** 정도에서 시작한다. 데이터가 적을수록
과적합이 빠르니 `eval_loss` 가 오르기 시작하면 멈춘다.

## 7. 평가

세 단계로 나눠서 본다. 뒤로 갈수록 비싸고 진짜에 가깝다.

**① 형식 (즉시, 비용 0)** — `sft_val.jsonl` 로 생성해 보고 JSON 파싱률, 스키마 위반율,
목록에 없는 후보 id 비율을 센다. 여기서 90% 를 못 넘으면 다음 단계는 의미가 없다.

**② 판단 일치 (오프라인)** — 같은 프롬프트에 대해 학생 답과 **교사 답**, 그리고
**거리 베이스라인 답**을 비교한다. `data/distill/export/disagreements.jsonl` 이 이미
"교사와 베이스라인이 갈린 관측" 만 모아 둔 것이라, 학생이 **어느 쪽을 따라가는지** 보면
증류가 됐는지 바로 안다.

**③ 시뮬 재평가 (실물)** — 서빙해서 할당기에 꽂고 같은 실험을 돌린다. 12장 6절과 같은
지표(90% 도달 시각, 90%까지의 탐사점 수·이동거리)로 **교사·베이스라인·학생**을 나란히 본다.

```bash
EXPLORE_STRATEGY=llm src/webots_goal_bridge/scripts/run_explore_compare.sh mac 3
python3 src/webots_goal_bridge/scripts/analyze_explore_compare.py
```

## 8. 서빙하고 할당기에 꽂기

할당기는 **OpenAI 호환이면 무엇이든** 붙는다. 코드는 그대로 두고 주소만 바꾼다.

```bash
# vLLM (권장 — 지연이 가장 짧다)
pip install vllm
python -m vllm.entrypoints.openai.api_server \
  --model Qwen/Qwen2.5-1.5B-Instruct --enable-lora --lora-modules student=./student \
  --max-model-len 4096 --port 8000

# 또는 Ollama (LoRA 를 합친 GGUF 를 만든 뒤)
ollama serve
```

compose 에서 할당기 쪽만 바꾸면 된다 ([12장 2절](12_TASK_ALLOCATION.md#실행)).

```bash
EXPLORE_STRATEGY=llm \
EXPLORE_BASE_URL=http://host.docker.internal:8000/v1 \
EXPLORE_MODEL=student \
  docker compose -f docker-configs/mac/docker-compose.yml --profile explore up -d explorer
```

> ⚠️ 지금 compose 는 `base_url`·`model`·`system_prompt` 를 환경변수로 노출하지 않는다.
> 런치 인자로는 이미 받으므로(`allocator.launch.py`), 학생을 붙일 때 compose 에
> `base_url:=${EXPLORE_BASE_URL:-...}` 세 줄을 추가하면 된다.
> **값이 빈 인자는 안 된다** — 기본값은 `none` 이 아니라 실제 기본 주소로 둔다
> ([12장 7절 ⑦](12_TASK_ALLOCATION.md#-값이-빈-launch-인자name-는-거부된다)).

학생에게는 **시스템 프롬프트를 비운다** (`system_prompt:=none`). `detailed thinking off` 는
Nemotron 전용 토글이라 다른 모델에는 잡음이다. `max_tokens` 도 256 정도로 줄인다 — 학생은
추론을 길게 하지 않으므로 그 이상은 지연만 늘린다.

## 9. 함정

- **프롬프트 판을 섞지 마라.** `alloc-v1`(분리 최대화)과 `alloc-v2`(목표 수·이동거리 최소화)는
  지시가 다르다. `meta.prompt_version` 으로 갈라서 쓴다
- **폴백 주기는 교사의 답이 아니다.** 내보내기가 이미 뺀다 (`fell_back`)
- **train/val 은 실행 단위로 나뉜다.** 같은 실행의 연속 주기는 관측이 거의 같아서 행 단위로
  섞으면 검증 점수가 샌다. 실행이 적으면 val 비중이 튄다 (실측: 6실행에서 48%)
- **어려운 상황의 데이터가 빈다.** 교사가 잘리거나 실패한 주기에는 정답이 없다. 후보가 많고
  복잡한 상황이 정확히 그런 주기라, 학생이 배우지 못한 구간이 남는다. 예산을 키운 별도 수집이
  필요할 수 있다
- **학생이 베이스라인만 따라 한다면 증류할 것이 없었다는 뜻이다.** ②에서 그걸 먼저 확인한다
- **속도를 재라.** 학생의 호출 지연이 주기(60초)보다 훨씬 짧아야 의미가 있다. 할당기가
  `집계` 로그에 평균 호출 시간을 찍는다

---

← [12. 탐사 할당](12_TASK_ALLOCATION.md) | [📖 책 목차](Readme.md#-목차)
