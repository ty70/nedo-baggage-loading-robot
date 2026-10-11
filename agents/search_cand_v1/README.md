# search_cand_v1

`extreme_point_v5` を基準に、搬入系を維持したまま **3D-BPP の候補探索だけを強化**した実験版です。

## v1 changes

### 1. Deferred Retry

1回目に `NO-GEOMETRIC-CANDIDATE` だった荷物を即座に捨てません。
他の荷物が配置されて Extreme Point / 支持面が変化した後、保留荷物を再試行します。

- 1周で1個以上配置できた場合: 保留荷物を再試行
- 1周で1個も配置できなかった場合: そのtrialを終了

### 2. Multi-Start candidate search (offline optimize only)

完全な荷物リストが渡される `optimize()` では、複数の構築順序で同じExtreme Point配置器を実行します。

初期候補:

- v5 base order
- volume descending
- footprint descending
- maximum-edge descending
- height descending
- height ascending
- seeded within-category permutations

各trialは同じ包含判定、棚干渉、support判定、final AABB、static shelf path判定を使います。

### 3. Best layout selection

trialは次の辞書式目的で比較します。

1. packed physical volume を最大化
2. placed item count を最大化
3. mass-weighted CoG Z を低く
4. v5 candidate score合計を低く

### 4. Offline BPP selection is final

`optimize()` を実行したケースでは、Multi-Startで採用されなかった荷物を後からrolling plannerへ再投入しません。
3D-BPPが採用した荷物だけを搬入対象にします。

## Intentionally unchanged

- Extreme Point geometry
- orientation feasibility checks
- support / soft / mass constraints
- static shelf path check
- C++ AABB checker integration
- final BPP target semantics
- InsertionPlanner runtime separation
- BPP JSON dump format

## Search controls

Environment variables:

```bash
SC_SEARCH_TRIALS=16
SC_SEARCH_SEED=20260906
SC_SEARCH_MAX_SECONDS=60
```

Default is 16 trials and a 60-second internal search budget, leaving substantial margin under the 180-second offline optimization limit.

## Run

```bash
cd ~/simulator

FILE_NAME=search_cand_v1
mkdir -p results/$FILE_NAME

EP_CPP_MODE=cpp \
EP_NEED_3D_BPP=./results/$FILE_NAME/bpp_layout_$FILE_NAME.json \
python -m scripts.run_test \
  --module-path agents/$FILE_NAME/ \
  --config-path configs/sample_config.json \
  --verbose True \
  --render-mode None \
  --result-dir results/$FILE_NAME/ \
  --result-fname evaluation_results_$FILE_NAME.json \
  2>&1 | tee ./results/$FILE_NAME/$FILE_NAME.log
```

## Main log checkpoints

```text
[search_cand_v1][SEARCH] ...
[search_cand_v1][SEARCH-BEST] ...
[search_cand_v1][INSERT-GRAPH] ... cycles=[]
```
