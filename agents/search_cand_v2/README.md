# search_cand_v2

`search_cand_v1` を基準に、Multi-Start の候補選択だけを強化した版です。

## v2 の選択フロー

```text
複数の荷物順序で Extreme Point 3D-BPP を構築
                ↓
各レイアウトの固定 target / orientation はそのまま保持
                ↓
Insertion feasibility dry-run
  - static shelf path
  - support dependency
  - runtime support
  - packed cargo against transport path
                ↓
small beam で複数の搬入順序を探索
                ↓
安全に搬入可能な volume / item count を最優先
                ↓
同点なら geometric BPP volume / item count / CoG
                ↓
最良候補の「安全に搬入可能な部分集合」だけを採用
                ↓
既存の InsertionPlanner で実搬入
```

## v1 から変更しない部分

- Extreme Point 生成
- 6 orientation 評価
- C++ AABB checker
- support 判定
- BPP target / orientation の定義
- Deferred Retry
- InsertionPlanner 本体
- BPP JSON 出力形式

## 追加環境変数

- `SC_FEASIBILITY_BEAM=12`
- `SC_FEASIBILITY_BRANCH=8`

## 実行例

```bash
FILE_NAME=search_cand_v2
mkdir -p results/$FILE_NAME

EP_CPP_MODE=cpp \
EP_NEED_3D_BPP=./results/$FILE_NAME/bpp_layout_$FILE_NAME.json \
SC_SEARCH_TRIALS=16 \
SC_SEARCH_MAX_SECONDS=60 \
SC_FEASIBILITY_BEAM=12 \
SC_FEASIBILITY_BRANCH=8 \
python -m scripts.run_test \
  --module-path agents/$FILE_NAME/ \
  --config-path configs/sample_config.json \
  --verbose True \
  --render-mode None \
  --result-dir results/$FILE_NAME/ \
  --result-fname evaluation_results_$FILE_NAME.json \
  2>&1 | tee ./results/$FILE_NAME/$FILE_NAME.log
```

見るべきログ:

```text
[search_cand_v2][SEARCH] ...
geom_placed=34 geom_volume=...
safe_placed=... safe_volume=...
blocked=[...]

[search_cand_v2][SEARCH-BEST] ...
```

`safe_*` が候補選択の主目的です。v1 のように geometric に大量充填できても、搬入 dry-run で詰まる荷物は最終採用から除外されます。
