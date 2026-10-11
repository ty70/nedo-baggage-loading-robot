# extreme_point_v4

`extreme_point_v3` から分岐した、3D-BPP と InsertionPlanner を分離したままの修正版です。

## v4 changes

1. **Simulator 座標系に合わせた world/local 変換修正**
   - `Container.local_to_global()` / `global_to_local()` と同じく、変換するのは X の `offset_x` だけ。
   - Y/Z に `container.center` を加減しない。
   - v3 ログで繰り返し出ていた `runtime-support z-gap ~= 0.82m` の原因を除去。

2. **Projected EP を第2段階候補へ降格**
   - 通常 EP で feasible candidate が1つでもあれば projected EP は使わない。
   - 通常 EP が全滅した場合だけ `*:proj:*` EP を探索。
   - 目先の projected pocket が既存の良い greedy packing sequence を崩すのを防ぐ。

3. **固定棚で永久に搬入不能な候補を BPP 段階で除外**
   - inclusion / support / final AABB が通っても static shelf path が通らない候補は予約しない。
   - small/main shelf support は InsertionPlanner と同じ temporary Z lift を使って static path を評価。

## Intentionally unchanged

- exact validator plane inclusion
- support ratio / mass / soft-item constraints
- BPP scoring weights
- support-before-upper precedence
- route precedence logic
- C++ AABB checker arithmetic order
- LAST-RESORT behavior (except that static-unreachable plans should now be much less likely to enter the plan)

## Run

```bash
cd ~/simulator
EP_CPP_MODE=cpp \
python -m scripts.run_test \
  --module-path agents/extreme_point_v4/ \
  --config-path configs/sample_config.json \
  --verbose True \
  --render-mode None \
  --result-fname evaluation_results_extreme_point_v4.json
```

## Log checkpoints

- `[extreme_point_v4]` tag is used so v3/v4 logs can be distinguished.
- `runtime-support ... z-gap=0.82xx` should disappear.
- Projected EP (`:proj:`) should appear only when ordinary EPs have no feasible target for that item.
- Targets that are statically blocked by a shelf should become `NO-GEOMETRIC-CANDIDATE` (or choose another reachable target) rather than reaching runtime as `STATIC_SHELF_BLOCKED`.


## 3D-BPP layout JSON output

`--need-3d-bpp <file.json>` を指定すると、InsertionPlanner の実行順ではなく、3D-BPP が決めた固定最終配置を JSON に保存します。

```bash
EP_CPP_MODE=cpp \
python -m scripts.run_test \
  --module-path agents/extreme_point_v4/ \
  --config-path configs/sample_config.json \
  --verbose True \
  --render-mode None \
  --result-fname evaluation_results_extreme_point_v4.json \
  --need-3d-bpp bpp_layout_extreme_point_v4.json
```

JSON は task ごとに `placements`, `container_list`, `bpp_order` を持ちます。`placements` は BPP の `final_target` そのもので、搬送に成功した荷物だけではなく、BPP が予約できた全荷物を含みます。001 のような rolling planning では計画追加のたびに同じファイルを更新します。

> 注意: `scripts.run_test` 側が未知の引数を拒否する版では、同じ機能を `EP_NEED_3D_BPP=bpp_layout_extreme_point_v4.json` でも有効化できます。これは runner 側の argparse に手を入れずに使うための互換経路です。


## v5 change
Only unsafe fallback behavior changed from v4:
- `NO-PLANNED-VISIBLE-ITEM` no longer calls the ordinary fallback placement.
- `NO-SAFE-EXECUTABLE-PLAN` no longer executes `INSERT-LAST-RESORT-BPP`.
- Because the simulator has no no-op/skip action, v5 returns an intentionally out-of-container target so inclusion validation terminates the episode before transport.
- BPP geometry, scoring, EP generation, precedence construction, runtime path/support checks, and BPP dump format are unchanged.
