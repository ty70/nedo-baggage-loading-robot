# search_cand_v12_item39_lift

Based on v11 diagnostic. Fixed BPP geometry / candidate scoring unchanged.

Item 39 only: temporary execution pose Z+0.021m (set `SC_ITEM39_LIFT`, use `0` as control).
This is **not** a different packing target; the simulator must settle the box onto item 22.
The physical simulator is NOT modified. All insertion/beam/runtime checks use the same execution_target.

Log checkpoints: `ORDER39-SUMMARY` (best_has39), `SEARCH-BEST` (safe_placed), runtime `[PACK] ... item=39`, `[INSERT] ... item=39`, and simulator status.

Run from `~/simulator2`; use the other settings of v11.
**Unverified in PyBullet**: lifting final action can fail settling or inclusion; compare with `SC_ITEM39_LIFT=0` and keep v8 as baseline.
