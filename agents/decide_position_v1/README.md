# decide_position_v1

`search_cand_v6` をベースに、物理 settling による境界逸脱を切り分ける版です。

3D-BPP の `final_target`、orientation、Insertion Planner、inclusion の式、C++ AABB
checker、score、EP 優先順位は変更しません。

次の policy observation で simulator が返す settled pose を使い、各 action について
`[SETTLE-AUDIT]` を出力します。

- `final_delta`: BPP final target から settled local pose への差分
- `execution_delta`: simulator に送った execution target からの差分
- `max_dot`, `plane`: Evaluator と同じ world corner の最大 boundary dot
- `included`: `max_dot <= inclusion_margin` の報告値

この監査は診断専用であり、これを根拠に BPP target や orientation を自動補正しません。


```bash
EP_CPP_MODE=cpp EP_DEBUG_INSIDE=1 \
EP_NEED_3D_BPP=./results/decide_position_v1/bpp_layout.json \
python -m scripts.run_test \
  --module-path agents/decide_position_v1/ \
  --config-path configs/sample_config.json \
  --verbose True --render-mode None \
  --result-dir results/decide_position_v1/ \
  --result-fname evaluation_results.json
```
