# v15: 小棚との搬入境界に正の余裕を確保

v14を継承し、小棚の右端から荷物左面までの距離が15 mm境界付近にあるときだけ、実行座標Xに1 µmの余裕を加えます。float32化後も余裕を維持する方向へ丸めます。
荷物番号には依存しません。公式安全マージンや評価関数、v14、既存結果は変更しません。
大きなマージン不足や幾何学的重なりを救済する処理ではありません。
v14の候補生成・スコア・探索順序を継承し、同じ共通クランプを通る計画の実行可能判定と実際の出力へ補正を適用します。
この版は同じリポジトリ内のv14を必要とします。

```bash
cd /workspace/nedo-baggage-loading-robot
bash scripts/cloud_setup.sh
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 \
/workspace/nedo-cloud/venv/bin/python -m scripts.test_shelf_clearance
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
EP_CPP_MODE=cpp SC_SEARCH_TRIALS=16 SC_SEARCH_MAX_SECONDS=60 \
SC_FEASIBILITY_BEAM=12 SC_FEASIBILITY_BRANCH=8 \
/workspace/nedo-cloud/venv/bin/python scripts/cloud_run.py \
  --module-path agents/search_cand_v15_shelf_clearance/ \
  --config-path configs/sample_config.json --verbose True
```

改善前の再現はmodule-pathを`agents/search_cand_v14_order_2716_diagnostic/`へ戻します。
結果は毎回`/workspace/nedo-cloud/runs/simulation-*`へ生成され、既存の`results/`へ書き込みません。
原因・比較値・制約は`docs/phase1_shelf_clearance.md`を参照してください。
