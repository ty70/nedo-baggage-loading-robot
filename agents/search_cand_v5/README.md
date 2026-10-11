# search_cand_v4

`search_cand_v3` を基準に、**Multi-Startの次の探索段階として、各荷物の配置候補にも小さなBeam Searchを導入した版**です。

v2/v3で導入した「3D-BPP候補を搬入可能性までdry-run評価し、安全に実行できる候補を選ぶ」という安全方針は維持します。v4では、荷物の並び順だけでなく、同じ並び順の途中で発生する複数の有望な配置未来を残します。

## v4の探索構造

```text
64通りの荷物順序
        ↓
各順序ごとに3D-BPP配置Beam Search
        ↓
各荷物で上位3候補を分岐
        ↓
上位4状態を保持
        ↓
完成した複数Layout
        ↓
各LayoutをInsertion Feasibility Beamでdry-run
        ↓
safe_volume / safe_placed を優先して選択
```

## デフォルト探索規模

- `SC_SEARCH_TRIALS=64`
- `SC_SEARCH_MAX_SECONDS=180`
- `SC_BPP_BEAM=4`
- `SC_BPP_BRANCH=3`
- `SC_FEASIBILITY_BEAM=32`
- `SC_FEASIBILITY_BRANCH=16`

BPP Beamは、最初から大きくしすぎると候補数が急増するため、まず幅4・分岐3から始めます。

## v4で新たに分岐する部分

v3では各荷物について、

```text
EP候補を評価
↓
最良候補1個を選択
↓
その未来だけ続行
```

でした。

v4では、

```text
EP候補を評価
↓
上位3候補
├─ 配置A
├─ 配置B
└─ 配置C
↓
部分Layoutを評価
↓
上位4状態を残す
```

とします。

これにより、「局所scoreでは2番手だが、その後の荷物が多く入る配置」を探索できます。

## 維持する安全条件

候補の生成・分岐では、v3までの以下をそのまま使います。

- Extreme Point生成
- ordinary EP優先、必要時のみprojected EP
- 6 axis-aligned orientation
- container境界
- shelf plateとの干渉
- support判定
- final AABB clear
- static shelf path clear
- Deferred Retry
- C++ checker
- InsertionPlanner

したがって、Beam Searchは「今まで安全だった候補を緩める」のではなく、**安全な候補を複数残す**変更です。

## Multi-Start候補の評価

各荷物順序から最大 `SC_BPP_BEAM` 個の異なる幾何学Layoutを作り、それぞれに対して既存の搬入Feasibility Beamを実行します。

最終選択の優先順位は従来どおり、

1. `safe_volume`
2. `safe_placed`
3. 幾何学的BPP volume
4. 幾何学的BPP item count
5. CoGなど

です。

安全装置のため、最終的にはfeasibility dry-runで安全に順序付けできた荷物だけを実行候補にします。

## 実行例

`optimization_timeout` は180秒以上にしてください。

```bash
cd ~/simulator

FILE_NAME=search_cand_v5

mkdir -p results/$FILE_NAME

EP_CPP_MODE=cpp \
EP_NEED_3D_BPP=./results/$FILE_NAME/bpp_layout_$FILE_NAME.json \
SC_SEARCH_TRIALS=64 \
SC_SEARCH_MAX_SECONDS=180 \
SC_BPP_BEAM=4 \
SC_BPP_BRANCH=3 \
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

## 見るべきログ

```text
[search_cand_v4][SEARCH]
trial=.../64 beam=1/4 strategy=...
geom_placed=...
safe_placed=...

[search_cand_v4][SEARCH-BEST]
strategy=...
safe_placed=...
layouts=...
```

`layouts=` が、実際に搬入可能性まで評価した幾何学Layout候補数です。

v4の目的は単純です。**荷物順序を増やすだけではなく、同じ順序でも「どこに置くか」の別ルートを残し、搬入可能な高充填Layoutを探す**ことです。
