# search_cand_v8_geometry — 27個と21個の配置比較

v8の配置アルゴリズムと安全判定を変更せず、採用された探索レイアウトの
「搬入可否の絞り込み前」を同じJSONに追加保存します。

## インストール
`~/simulator2/agents/search_cand_v8_geometry/` に agent.py,
insertion_planner.py, __init__.py を配置します。

## 実行
cd ~/simulator2
FILE_NAME=search_cand_v8_geometry
mkdir -p results/$FILE_NAME
EP_CPP_MODE=cpp EP_NEED_3D_BPP=./results/$FILE_NAME/bpp_layout_$FILE_NAME.json \
SC_SEARCH_TRIALS=8 SC_SEARCH_MAX_SECONDS=60 SC_BPP_BEAM=4 SC_BPP_BRANCH=3 \
SC_FEASIBILITY_BEAM=32 SC_FEASIBILITY_BRANCH=16 SC_REAR_OPEN_PENALTY=1.5 \
python -m scripts.run_test --module-path agents/$FILE_NAME/ \
--config-path configs/sample_config.json --verbose True --render-mode None \
--result-dir results/$FILE_NAME/ --result-fname evaluation_results_$FILE_NAME.json

## JSONの項目
- `placements`: 搬入可能と判断された配置（Task 000では21個想定）
- `geometric_placements`: 同じ探索レイアウトの搬入判定前の配置（27個想定）
- `blocked_item_ids`: 上記2つの集合の差
- `num_planned_items` / `num_geometric_items`: 両者の個数

`ground_handling_visualizer_compare.html` をブラウザで開いて生成したJSONを選択し、
「表示する配置」で切り替えます。

注意：幾何学的に置けることは実際に搬入できることを意味しません。
この変更は可視化用であり、シミュレーションの配置結果は保証しません。
