# search_cand_v10_runtime_replan (experimental)

Simulator contract checked against `src/ground_handling/env.py`:
- `set_item_order` requires a **permutation of all item IDs**, not a subset.
- `step` offers no skip/no-op; inclusion/path failure terminates the episode.
- Thus appending unplanned items in `optimize` is required, not necessarily a bug.

## Change from v9 diagnostic
When `policy` encounters a visible item with no precomputed safe BPP plan,
try `_extend_geometric_plan` on the current packed state and all remaining
reservations. If a placement can be planned, refresh precedence and let the
**existing InsertionPlanner and runtime path/inclusion checks** decide whether
the action is safe. If not, preserve previous deliberate safe abort behavior.

No forced assignment for item 38 or 32. No change to the original offline
beam-search logic. Successful 21 planned placements should remain the same.

Log markers: `[V10-UNPLANNED]`, `[V10-REPLAN]`, `[V10-REPLAN-FAILED]`,
`[PACK-PLAN] source=runtime_replan`, `[INSERT]`, `[ENTRY-REJECT]`.

## Install
Extract folder to `~/simulator2/agents/search_cand_v10_runtime_replan/`.

## Run from ~/simulator2
```bash
cd ~/simulator2
FILE_NAME=search_cand_v10_runtime_replan
mkdir -p results/$FILE_NAME
SC_DIAG_ITEMS=38 EP_CPP_MODE=cpp \
EP_NEED_3D_BPP=./results/$FILE_NAME/bpp_layout_$FILE_NAME.json \
SC_SEARCH_TRIALS=8 SC_SEARCH_MAX_SECONDS=60 SC_BPP_BEAM=4 SC_BPP_BRANCH=3 \
SC_FEASIBILITY_BEAM=32 SC_FEASIBILITY_BRANCH=16 SC_REAR_OPEN_PENALTY=1.5 \
python -m scripts.run_test --module-path agents/$FILE_NAME/ \
--config-path configs/sample_config.json --verbose True --render-mode None \
--result-dir results/$FILE_NAME/ --result-fname evaluation_results_$FILE_NAME.json \
2>&1 | tee results/$FILE_NAME/$FILE_NAME.log
```

This is a diagnostic/experimental patch: syntax checked only; no PyBullet test
has been run here. A runtime attempt for item 39 may fail because no safe
placement actually exists. Check log before concluding improved score.
