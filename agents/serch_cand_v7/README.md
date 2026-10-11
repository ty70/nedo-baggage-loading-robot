# serch_cand_v7

Based on user-provided `agent(1).py` and the existing `insertion_planner.py`.

## Changes
- Use the same simulator-aligned Y -> X transport precheck in offline feasibility, runtime selection and last-resort selection.
- Include supporting boxes as obstacles during the transport sweep, matching `PlacementValidator._move_item`.
- Fix double rotation of already-oriented item dimensions in `_clamp_to_safe_margin`.
- Try other remaining BPP plans rather than aborting at the first blocked last-resort plan.
- Preserve the existing 8-trial search settings and out-of-container abort behavior when no feasible action exists.

## Setup
Copy this directory to `simulator2/agents/serch_cand_v7/`.

Run from `simulator2`:

```bash
FILE_NAME=serch_cand_v7
mkdir -p results/$FILE_NAME
EP_CPP_MODE=cpp EP_NEED_3D_BPP=./results/$FILE_NAME/bpp_layout_$FILE_NAME.json \
SC_SEARCH_TRIALS=8 SC_SEARCH_MAX_SECONDS=60 SC_BPP_BEAM=4 SC_BPP_BRANCH=3 \
SC_FEASIBILITY_BEAM=32 SC_FEASIBILITY_BRANCH=16 \
python -m scripts.run_test --module-path agents/$FILE_NAME/ \
  --config-path configs/sample_config.json --verbose True --render-mode None \
  --result-dir results/$FILE_NAME/ --result-fname evaluation_results_$FILE_NAME.json \
  2>&1 | tee ./results/$FILE_NAME/$FILE_NAME.log
```

The simulator source `src/ground_handling/validator.py` is not changed here. The supplied modified validator should be kept separately for local testing only; contest evaluation may use the original simulator. No PyBullet test has been run here.
