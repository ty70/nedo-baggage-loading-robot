# search_cand_v3

`search_cand_v2` を基準に、**3D-BPP候補数と搬入順序探索の幅を大きく増やした版**です。

v2で導入した「Multi-Start候補を、幾何学的な充填率だけでなく搬入可能性まで評価する」方針は維持します。v3では、その安全評価器を変更するのではなく、より多くのBPPレイアウトと搬入順序を探索します。

## v3 の探索規模

デフォルト値:

- `SC_SEARCH_TRIALS=64`（v2: 16）
- `SC_SEARCH_MAX_SECONDS=180`（v2: 60）
- `SC_FEASIBILITY_BEAM=32`（v2: 12）
- `SC_FEASIBILITY_BRANCH=16`（v2: 8）

評価基盤の `optimization_timeout=180s` を上限として使う設計です。

## 3D-BPP候補の種類

最初に以下の決定的な順序を試します。カテゴリ順（通常/優先/ソフト等）は維持し、そのカテゴリ内の並びだけを変えます。

- base
- mass_desc
- volume_desc
- footprint_desc
- max_edge_desc
- min_edge_desc
- height_desc
- height_asc
- density_desc
- compact_desc
- slender_desc
- small_first

その後、固定seedによるカテゴリ内シャッフルを使って、要求された最大64通りまで異なる構築順を生成します。

## v2から維持する安全評価

各BPPレイアウトについて固定target/orientationを変えずに、以下をdry-run評価します。

- static shelf path
- support dependency
- runtime support
- 既に仮想配置した荷物との搬入経路
- 複数の搬入順序をBeam Searchで探索

候補選択は、`safe_volume`、`safe_placed` を最優先し、その後に幾何学的なBPP体積・個数・CoG等で比較します。

## v3で変更しない部分

- Extreme Point生成
- 6 orientation評価
- C++ AABB checker
- support判定
- BPP target/orientationの定義
- Deferred Retry
- InsertionPlanner本体
- BPP JSON出力形式
- safe subsetのみを実搬入候補として採用するv2の安全装置

## 実行例

```bash
FILE_NAME=search_cand_v3
mkdir -p results/$FILE_NAME

EP_CPP_MODE=cpp \
EP_NEED_3D_BPP=./results/$FILE_NAME/bpp_layout_$FILE_NAME.json \
SC_SEARCH_TRIALS=64 \
SC_SEARCH_MAX_SECONDS=180 \
SC_FEASIBILITY_BEAM=32 \
SC_FEASIBILITY_BRANCH=16 \
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
[search_cand_v3][SEARCH] trial=... strategy=...
geom_placed=... geom_volume=...
safe_placed=... safe_volume=...
blocked=[...] beam_states=...

[search_cand_v3][SEARCH-BEST] ...
```

重要なのは、v2の16候補では見つからなかった「幾何学的にも十分詰められ、かつ安全に搬入できる」レイアウトを64候補・より広い搬入Beamから探すことです。
