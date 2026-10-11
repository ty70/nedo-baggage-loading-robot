# extreme_point_v01 — BPP / Insertion separated

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
[extreme_point_v01][PACK-PLAN] ... item=... final_target=(...)
[extreme_point_v01][INSERT-GRAPH] edges=['2->0:route', ...] order=[...]
[extreme_point_v01][INSERT-ORDER] mode=full order=[...] cycles=[]
```

Execution:

```text
[extreme_point_v01][PACK] ... final_target=(...)
[extreme_point_v01][INSERT] ... execution_target=(...) path_ok=True forced=False
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
