# extreme_point_v3

Baseline: successful `extreme_point_v01` exact-plane + precedence version.

## v2 change

Rolling BPP now treats **all fixed `preplanned` targets as reserved occupied geometry**, regardless of whether the action has already been issued. Reservations are de-duplicated against simulator/internal packed geometry by item id. This prevents a newly visible item from receiving the same `final_target` as an earlier planned item during observation/action lag.

New diagnostic log:

```text
[extreme_point_v3][RESERVE] container=0 fixed_targets_added=N occupied=M
```

Packing targets remain owned by the BPP planner; insertion order remains owned by `InsertionPlanner`. No route heuristic is added to the BPP score.

# extreme_point_v3 — BPP / Insertion separated

This revision separates **where cargo should finally be packed** from **in which order those fixed poses can be inserted**.

## 1. `agent.py`: 3D-BPP / Extreme Point

The BPP side owns the final pose. It uses geometry only:

- six axis-aligned orientations,
- floor / shelf / packed-box Extreme Points,
- container bounds,
- shelf-plate exclusion,
- support ratio,
- final AABB overlap,
- mass / soft support rules,
- geometric compactness score.

It does **not** move a target to preserve a transport lane and it does **not** use Y->X path clearance when choosing the final target.

The fixed plan is stored as `preplanned[item_id]`.

### Full-plan mode

When simulator `optimize()` is enabled, all items are geometrically packed first. The insertion graph is then built from the complete fixed layout.

### Rolling-plan mode

When `optimize()` is not called, all newly visible lookahead items are assigned fixed targets together. Existing fixed targets are reserved and are not relocated when the horizon advances.

## 2. `insertion_planner.py`: insertion precedence

For every fixed BPP plan, the insertion planner checks pairwise blocking:

- If item **B**, once placed at its final pose, blocks item **A**'s Y->X insertion route, create `A -> B`.
- If item **A** supports item **B**, create `A -> B`.

`A -> B` means **A must be inserted before B**.

The graph is topologically sorted. BPP construction rank is used only as a deterministic tie-breaker.

If the graph contains a cycle, the cycle is logged explicitly. The planner does not silently relocate a BPP target to break it.

At runtime, the planner:

1. considers only visible items with fixed BPP plans,
2. respects unresolved predecessors,
3. checks the route against actual already-packed cargo,
4. chooses the earliest executable plan,
5. applies a temporary +21 mm Z lift for shelf insertion only.

The **final BPP target never changes**.

## Logs

Planning:

```text
[extreme_point_v3][PACK-PLAN] ... item=... final_target=(...)
[extreme_point_v3][INSERT-GRAPH] edges=['2->0:route', ...] order=[...]
[extreme_point_v3][INSERT-ORDER] mode=full order=[...] cycles=[]
```

Execution:

```text
[extreme_point_v3][PACK] ... final_target=(...)
[extreme_point_v3][INSERT] ... execution_target=(...) path_ok=True forced=False
```

This makes failures separable:

- `PACK-PLAN ... NO-GEOMETRIC-CANDIDATE` = BPP problem.
- `INSERT-GRAPH ... cycles=[...]` = fixed layout cannot be linearly inserted under the current route model.
- `INSERT ... NO-EXECUTABLE-VISIBLE-PLAN` = runtime insertion problem / insufficient visible horizon / settled-pose deviation.

## 2026-09-05 exact container inclusion

The BPP inclusion filter now mirrors `PlacementValidator.check_inclusion()` when
`container.n_vecs` and `container.points` are available:

`dot(n, target-point) + dot(abs(n), half_lwh) <= inclusion_margin`

This handles the real cut/sloped cargo boundary instead of approximating the
container as an axis-aligned rectangular box.  The default validator margin is
`-0.005 m` (sample_config).  It can be overridden with `EP_INCLUSION_MARGIN`.
Set `EP_DEBUG_INSIDE=1` to log the worst rejected plane.

This change is intentionally limited to the BPP geometric inclusion filter;
the precedence/insertion-order logic is unchanged.


## v3 delta
- Precedence route checks ignore fixed shelves; runtime route checks still include them.
- Removed forced execution of known path-invalid plans.
- If precedence is temporarily over-constrained, v3 may bypass precedence only for a runtime path-clear plan.

## Runtime support / hard precedence update

This v3 update keeps the 3D-BPP final targets unchanged and adds two execution-safety rules:

- Direct precedence edges are hard constraints. `precedence_bypass=True` is no longer used.
- Before a planned item is selected, its BPP support contract is re-checked at runtime:
  - floor: bottom Z must match the floor plane;
  - small/main shelf: footprint overlap must remain >= `min_support_ratio`;
  - box support: the exact `support_item_id` must be present first, the bottom/top Z gap must be within tolerance, and footprint overlap must remain >= `min_support_ratio`.

When the simulator has a settled `packed_items` pose it is preferred (`source=settled`). During lookahead action generation, when the simulator has not executed earlier actions yet, the fixed BPP reservation is used (`source=planned`). The source and measured support ratio are printed in `support_runtime=...` on each `[INSERT]` line.

## 2026-09-05 small revision
- Box support from an already-issued earlier action is accepted explicitly as `source=predecessor-plan` during get_init_states lookahead.
- Insertion precedence/runtime path checks add the simulator's low horizontal entry phase (`z = thickness + item_height/2 + 0.006`) followed by the vertical move at target XY. This catches blockers that the previous final-Z-only transport approximation missed.
- When no safe insertion candidate exists, the historical center-of-container fallback is no longer used. The last-resort action preserves the fixed 3D-BPP target and is logged as `INSERT-LAST-RESORT-BPP`.
- 3D-BPP final targets, orientations, scoring, and support construction are otherwise unchanged.

## 2026-09-05 small fix: support-authoritative route ordering

- A box supporter is authoritative in the insertion graph: `supporter -> upper`.
- The reverse route edge `upper -> supporter` is never generated for that pair.
- At runtime, the already-present box supporter is still required by the support check, but is excluded from the moving upper box's route-obstacle set. Other packed items and fixed shelves remain obstacles.
- This specifically removes artificial 2-cycles such as `5 -> 3 (support)` plus `3 -> 5 (route)`, `0 -> 8` plus `8 -> 0`, and `4 -> 12` plus `12 -> 4`.

## 2026-09-05 focused fix for case001 static shelf deadlock

- Fixed-shelf reachability is now classified separately from movable-item route blockage.
- A plan that fails the shelf-only path test is marked `STATIC_SHELF_BLOCKED`.
- Such a plan is never chosen by `INSERT-LAST-RESORT-BPP`.
- Route-only outgoing precedence edges from a static-blocked plan are ignored at runtime so that other items can continue.
- Support edges are never ignored by this mechanism.
- Verified against case001 geometry: item11 target `(0.415, 0.420, 0.681)` is detected as static-shelf blocked, while unrelated targets remain shelf-clear.


## 3D-BPP top-view projected EP update

The visualizer motivated one small BPP-only change: besides EPs derived from a
single box, v3 now generates projected X-Y corner EPs using the X side face of
one packed box and the front Y face of another box on the same support layer.
This lets the planner try rectangular pockets that are visible in a top view but
were unreachable from single-box EPs.  Candidate inclusion, support, overlap and
InsertionPlanner logic are unchanged.
