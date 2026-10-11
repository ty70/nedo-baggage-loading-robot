# v14: 2/7/16 insertion-order diagnostic

Based on v13. No geometry, scoring, precedence, motion, or pruning modifications.
Adds `SC_ORDER_DIAG_ITEMS` (default `2,7,16`), and logs:
- `[V14-SUMMARY]` totals: READY, support failure, fixed-shelf path, and route blockers
- `[V14-BRANCH]` count of expanded children containing each item
- `[V14-PRUNE]` count retained by Beam Search

Run with `SC_ORDER_DIAG_ITEMS=2,7,16 SC_DIAG_ITEMS=2,7,16 SC_SEARCH_TRIALS=1` along with the v13 command.
No item-specific overrides are applied.

## Visualizer
Open `ground_handling_visualizer_v14.html` in a browser and select the `bpp_layout_*.json` generated under `results/`. Compare geometric and safe placements; dimensions in the JSON are already oriented.
