# search_cand_v8 — next optimization workspace

This is an **unchanged code copy** of the successful `serch_cand_v7` agent, isolated for the next optimization iteration. The v7 implementation is preserved; no untested algorithm changes were silently introduced.

## Baseline result (user-supplied log, task 000)
- 41 items in scenario
- v7: 21 placement actions with `path_ok=True`, versus 3 actions in v6.
- Offline geometry: 27 items; feasibility-approved: 21 items.
- After item 21, next item 39 has no preplanned BPP target and produces `NO-BPP-PLAN` / `SAFE-ABORT`.
- No final evaluation score appears in the supplied v7 log; task 001 is not shown.

## Next optimization targets
1. Increase geometric candidate variety and the feasible subset without sacrificing the v7 path precheck.
2. Determine the simulator's legal behavior for excluded/unplanned items; do **not** assume an out-of-container position is a legal skip.
3. Revalidate with official validator, then evaluate both 000 and 001 before accepting a new change.
4. Address missing optional `cpp_aabb_checker` import separately from correctness.

## Run
Copy the folder to `~/simulator2/agents/search_cand_v8/`. From `~/simulator2`:

```bash
FILE_NAME=search_cand_v8
mkdir -p results/$FILE_NAME
EP_CPP_MODE=cpp EP_NEED_3D_BPP=./results/$FILE_NAME/bpp_layout_$FILE_NAME.json \
SC_SEARCH_TRIALS=8 SC_SEARCH_MAX_SECONDS=60 SC_BPP_BEAM=4 SC_BPP_BRANCH=3 \
SC_FEASIBILITY_BEAM=32 SC_FEASIBILITY_BRANCH=16 \
python -m scripts.run_test \
  --module-path agents/$FILE_NAME/ --config-path configs/sample_config.json \
  --verbose True --render-mode None --result-dir results/$FILE_NAME/ \
  --result-fname evaluation_results_$FILE_NAME.json \
  2>&1 | tee ./results/$FILE_NAME/$FILE_NAME.log
```

## v8.1 奥側優先（2026-10-10）

`agent.py` の `_score_candidate` に、**同じX方向のレーンで奥に空間を残す候補**へのソフトペナルティを追加しました。既存の `2.5 * front_n` に加えて、奥の未充填距離をコンテナ幅で正規化して減点します。支持・衝突・経路の安全判定は変更していません。

- `SC_REAR_OPEN_PENALTY=1.5`：追加ペナルティ係数（既定値）。大きいほど奥から優先。
- `SC_REAR_OPEN_PENALTY=0`：この新しいスコア補正を無効化し、v7と同じ候補スコアに戻す。
- `SC_REAR_LANE_OVERLAP=0.35`：同じXレーンを埋めていると判定する最小水平重複率。

**検証**：Python構文チェック、単純化したスコア単体テストは成功。PyBulletの `run_test` は未実行。変更が21個の搬入数を増やす保証はありません。v7の結果を同じ条件で比較してください。
