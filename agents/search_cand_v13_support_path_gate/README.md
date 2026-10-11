# search_cand_v13_support_path_gate

Derived from the verified **v12 23-item case**. v12 remains untouched.

## New check (on by default)

At each 3D-BPP candidate, if support_kind=box, check the simulator-aligned
Y->X path against **only its mandatory supporter**. This relationship cannot
be solved by ordering since the supporter has to be inserted first.

* The actual simulator approach Z clamp, false main-shelf zero lift and
  15mm safety margin are retained.
* Candidate #39 retains v12's 21mm *execution* Z lift.
* Other boxes are intentionally NOT checked until insertion-order beam search.
* No EP generation, candidate score, BPP geometry, physics, or insertion beam
  heuristics were changed.
* Disable the new gate for comparison: `SC_BPP_SUPPORT_PATH_GATE=0`.
* Optional item diagnostics: `SC_DIAG_ITEMS=36,39` (can be verbose).

Expected: the old geometric pose #36 at Z1.440 is not admissible when it
requires #27 at Z1.197 and the simulator approach is ceiling-clamped. BPP
may find an alternate pose for #36 or select another layout. **23 items is
not guaranteed**, so keep v12 as a control baseline.

Run as with v12 replacing FILE_NAME:
```
cd ~/simulator2
FILE_NAME=search_cand_v13_support_path_gate
mkdir -p results/$FILE_NAME
SC_BPP_SUPPORT_PATH_GATE=1 SC_ITEM39_LIFT=0.021 SC_DIAG_ITEMS=36 \
SC_SEARCH_TRIALS=1 SC_SEARCH_MAX_SECONDS=60 \
SC_BPP_BEAM=4 SC_BPP_BRANCH=3 \
SC_FEASIBILITY_BEAM=32 SC_FEASIBILITY_BRANCH=16 \
SC_REAR_OPEN_PENALTY=1.5 \
python -m scripts.run_test --module-path agents/$FILE_NAME/ \
 --config-path configs/sample_config.json --verbose True \
 --render-mode human --result-dir results/$FILE_NAME/ \
 --result-fname evaluation_results_$FILE_NAME.json \
 2>&1 | tee results/$FILE_NAME/$FILE_NAME.log
```

Compare `[SEARCH-BEST]`, `V13-SUPPORT-PATH-REJECT`, `INSERT` and `SAFE-ABORT`.
Do not interpret geom_placed count as physically loaded count.
