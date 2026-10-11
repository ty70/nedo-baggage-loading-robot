import math
import os
import sys
from pathlib import Path
import numpy as np

try:
    from .insertion_planner import InsertionPlanner
except ImportError:
    from insertion_planner import InsertionPlanner


class Agent:
    """extreme_point_v01: first independent Extreme-Point 3D-BPP agent.

    Design goals for v01:
      - independent from safe_grid_* strategy code
      - generate geometric candidate points from floor/shelves/packed items
      - evaluate all 6 axis-aligned orientations
      - use the custom C++ checker for final AABB feasibility
      - delegate transport feasibility/order to a separate InsertionPlanner
      - keep a small, explicit heuristic score rather than hard-coded grid scans

    EP convention:
      Each EP is (x_edge, y_edge, z_bottom) where x_edge/y_edge are the
      positive-X / positive-Y outer faces of a candidate.  A box with dimensions
      (L,W,H) is placed at:
          center = (x_edge-L/2, y_edge-W/2, z_bottom+H/2)
      This makes the natural fill order back(+Y)->front(-Y), right(+X)->left(-X).
    """

    def __init__(self, module_path: str):
        self.module_path = module_path

        # Geometry / validator-aligned margins.
        self.safety_margin = 0.015
        self.extra_gap = 0.005
        self.xy_gap = self.safety_margin + self.extra_gap   # collision clearance = 0.020 m
        self.ep_gap = self.xy_gap  # dense contiguous EP spacing; currently 20 mm
        self.z_gap = 0.003
        self.start_margin = 0.01
        self.boundary_margin = 0.015
        # Exact validator inclusion threshold. sample_config.json uses -0.005 m.
        # Override only when the simulator config changes.
        self.inclusion_margin = float(os.environ.get('EP_INCLUSION_MARGIN', '-0.005'))

        # EP controls.
        self.ep_quant = 0.002      # 2 mm de-duplication cell
        self.max_eps_per_container = 400
        self.max_candidates = 6000
        self.min_support_ratio = 0.90
        self.support_z_tolerance = 0.035
        self.weight_tolerance = 0.05

        # State.
        self.optimize_enabled = False
        self.lookahead_k = 1
        self.container_init = []
        self.placed_orientation = {}
        self.soft_item_ids = set()
        self.global_max_mass = None
        self.step_no = 0
        # Per-container internal placement history.  The simulator's packed_items
        # can lag by one policy call, so we keep the action we just issued and
        # merge it with settled simulator poses on the next observation.
        self.internal_packed = []
        # Explicitly remember used EP locations. This prevents an initial seed
        # from being selected again even if simulator packed_items lags.
        self.used_ep_keys = []

        # Separation state: BPP owns fixed final targets; insertion owns order.
        self.preplanned = {}          # item_id -> fixed geometric plan
        self.precedence_graph = None  # built by InsertionPlanner
        self.executed_ids = set()
        self.plan_counter = 0

        # C++ checker.  Keep a dedicated env var so this agent does not depend
        # on SAFE_GRID_CPP_MODE naming.
        self.cpp_mode = os.environ.get('EP_CPP_MODE', 'cpp').strip().lower()
        self.cpp_checker = None
        self.cpp_checker_path = None
        self.cpp_stats = {
            'final_cpp': 0,
            'path_cpp': 0,
            'python_fallback_final': 0,
            'python_fallback_path': 0,
            'generated_eps': 0,
            'tested_candidates': 0,
            'feasible_candidates': 0,
        }
        self._load_cpp_checker(module_path)
        self.insertion_planner = InsertionPlanner(
            path_clear_fn=self._path_clear,
            shelf_lift=self.safety_margin + 0.006,
        )

    # ------------------------------------------------------------------
    # Simulator API
    # ------------------------------------------------------------------
    def get_init_states(self, init_states: dict):
        self.optimize_enabled = bool(init_states.get('optimize', False))
        self.lookahead_k = int(init_states.get('lookahead_k', 1))
        self.container_init = init_states.get('container_list', [])
        self.placed_orientation = {}
        self.soft_item_ids = set()
        self.global_max_mass = None
        self.step_no = 0
        self.internal_packed = [[] for _ in self.container_init]
        self.used_ep_keys = [set() for _ in self.container_init]
        self.preplanned = {}
        self.precedence_graph = None
        self.executed_ids = set()
        self.plan_counter = 0
        for k in self.cpp_stats:
            self.cpp_stats[k] = 0
        print(
            f'[extreme_point_v01][init] boundary_margin={self.boundary_margin:.3f} '
            f'xy_gap={self.xy_gap:.3f} ep_gap={self.ep_gap:.3f} '
            f'inclusion_margin={self.inclusion_margin:.3f} cpp_mode={self.cpp_mode}',
            flush=True
        )
        return True

    def optimize(self, item_list: list):
        """Plan the full packing layout first, then return insertion order.

        When the simulator enables optimize(), this is the cleanest separation:
        3D-BPP fixes every reachable final target using geometry only, then the
        InsertionPlanner derives a precedence graph from those fixed targets.
        """
        self._register_item_metadata(item_list)
        base_order = self._cargo_order_items(item_list)
        self._extend_geometric_plan(base_order, self.container_init, source='optimize')
        self._rebuild_precedence(self.container_init)

        planned_order = []
        if self.precedence_graph is not None:
            planned_order = [
                iid for iid in self.precedence_graph.get('order', [])
                if iid in self.preplanned
            ]
        # Keep simulator API total: any item that BPP could not place is appended
        # after the planned set instead of being silently dropped.
        seen = set(planned_order)
        planned_order.extend(int(it['index']) for it in base_order if int(it['index']) not in seen)
        print(
            f'[extreme_point_v01][INSERT-ORDER] mode=full order={planned_order} '
            f'cycles={self.precedence_graph.get("cyclic", []) if self.precedence_graph else []}',
            flush=True,
        )
        return planned_order

    def policy(self, observation: dict):
        pool = observation.get('pool_list', [])
        containers = observation.get('container_list', [])
        if not pool or not containers:
            return self._fallback()

        self.step_no += 1
        self._register_item_metadata(pool)

        # If optimize() was not called (e.g. lookahead mode), reserve fixed BPP
        # targets for every newly visible item as a rolling horizon. Existing
        # targets are never relocated by this extension.
        missing = [it for it in self._cargo_order_items(pool) if int(it['index']) not in self.preplanned]
        if missing:
            self._extend_geometric_plan(missing, containers, source='rolling')
            self._rebuild_precedence(containers)

        visible_plans = []
        pool_index_by_id = {}
        for pidx, raw in enumerate(pool):
            iid = int(raw['index'])
            pool_index_by_id[iid] = pidx
            plan = self.preplanned.get(iid)
            if plan is not None and iid not in self.executed_ids:
                visible_plans.append(plan)

        if not visible_plans:
            print(
                f'[extreme_point_v01][PACK] step={self.step_no} NO-PLANNED-VISIBLE-ITEM '
                f'pool={[int(x["index"]) for x in pool]}',
                flush=True,
            )
            return self._fallback(containers[0], pool[0])

        packed_by_container = {
            i: self._effective_packed(i, c) for i, c in enumerate(containers)
        }
        placed_ids = set(self.executed_ids)
        for packed in packed_by_container.values():
            for obj in packed:
                if obj.get('index') is not None:
                    placed_ids.add(int(obj['index']))

        selected = self.insertion_planner.choose_runtime(
            visible_plans,
            containers,
            packed_by_container,
            graph=self.precedence_graph,
            placed_ids=placed_ids,
        )

        if selected is None:
            order = self.precedence_graph.get('order', []) if self.precedence_graph else []
            print(
                f'[extreme_point_v01][INSERT] step={self.step_no} NO-EXECUTABLE-VISIBLE-PLAN '
                f'visible={[int(p["item_id"]) for p in visible_plans]} order={order}',
                flush=True,
            )
            # Diagnostic fallback: execute the highest-priority *fixed* plan,
            # never an unrelated center pose. This preserves problem separation.
            rank = {iid: i for i, iid in enumerate(order)}
            plan = min(visible_plans, key=lambda p: (rank.get(int(p['item_id']), 10**9), int(p['plan_rank'])))
            execution_pos, lift = self.insertion_planner.execution_target(plan)
            insert_meta = {'path_ok': False, 'shelf_lift': lift, 'forced': True}
        else:
            plan, execution_pos, insert_meta = selected

        iid = int(plan['item_id'])
        pidx = int(pool_index_by_id[iid])
        ori = int(plan['orientation'])
        final_pos = tuple(float(v) for v in plan['final_target'])
        item = plan['item']
        cidx = int(plan['container_idx'])
        self.placed_orientation[iid] = ori
        self.executed_ids.add(iid)

        ep = plan['ep']
        while len(self.used_ep_keys) <= cidx:
            self.used_ep_keys.append(set())
        self.used_ep_keys[cidx].add(self._ep_location_key(ep))
        # Internal geometry records the BPP final target, not temporary shelf lift.
        self._record_internal_placement(cidx, containers[cidx], item, final_pos, ori)

        print(
            f'[extreme_point_v01][PACK] step={self.step_no} item={iid} plan_rank={plan["plan_rank"]} '
            f'ori={ori} final_target=({final_pos[0]:.3f},{final_pos[1]:.3f},{final_pos[2]:.3f}) '
            f'ep=({ep[0]:.3f},{ep[1]:.3f},{ep[2]:.3f},{ep[3] if len(ep)>=4 else "?"}) '
            f'support={plan["support_kind"]} score={plan["score"]:.5f}',
            flush=True,
        )
        print(
            f'[extreme_point_v01][INSERT] step={self.step_no} item={iid} '
            f'execution_target=({execution_pos[0]:.3f},{execution_pos[1]:.3f},{execution_pos[2]:.3f}) '
            f'path_ok={insert_meta.get("path_ok")} shelf_lift={insert_meta.get("shelf_lift",0.0):.3f} '
            f'forced={insert_meta.get("forced", False)}',
            flush=True,
        )

        return {
            'item_idx': pidx,
            'container_idx': cidx,
            'place_pos': np.asarray(execution_pos, dtype=np.float32),
            'orientation': ori,
        }

    def _register_item_metadata(self, items):
        masses = [float(it.get('mass', 0.0)) for it in items]
        if masses:
            m = max(masses)
            self.global_max_mass = m if self.global_max_mass is None else max(self.global_max_mass, m)
        for it in items:
            if bool(it.get('is_soft', False)) and it.get('index') is not None:
                self.soft_item_ids.add(int(it['index']))

    def _cargo_order_items(self, item_list):
        """Deterministic BPP construction order; not the execution order."""
        def category(it):
            soft = bool(it.get('is_soft', False))
            pri = bool(it.get('is_prioritized', False))
            if not soft and not pri:
                return 0
            if not soft and pri:
                return 1
            if soft and not pri:
                return 2
            return 3
        return sorted(
            list(item_list),
            key=lambda it: (
                category(it),
                -float(it.get('mass', 0.0)),
                -(float(it['length']) * float(it['width'])),
                -float(it['height']),
                int(it['index']),
            ),
        )

    def _pending_plan_records(self, cidx):
        out = []
        for iid, p in self.preplanned.items():
            if int(p['container_idx']) != int(cidx) or iid in self.executed_ids:
                continue
            rec = p.get('packed_record')
            if rec is not None:
                out.append(dict(rec))
        return out

    def _extend_geometric_plan(self, items, containers, source='rolling'):
        """Reserve non-overlapping fixed final targets using geometry only."""
        # Start from actual/current geometry and reserve every already planned
        # but unexecuted final pose. This prevents rolling replanning overlap.
        planned_packed = {}
        local_used = {}
        for cidx, c in enumerate(containers):
            if source == 'optimize':
                base = self._oriented_packed(c.get('packed_items', []) or [])
            else:
                base = self._effective_packed(cidx, c)
            base_ids = {int(o['index']) for o in base if o.get('index') is not None}
            for rec in self._pending_plan_records(cidx):
                if rec.get('index') is not None and int(rec['index']) in base_ids:
                    continue
                base.append(rec)
            planned_packed[cidx] = base
            local_used[cidx] = set(self.used_ep_keys[cidx]) if cidx < len(self.used_ep_keys) else set()
            for p in self.preplanned.values():
                if int(p['container_idx']) == cidx:
                    local_used[cidx].add(self._ep_location_key(p['ep']))

        for raw_item in items:
            iid = int(raw_item['index'])
            if iid in self.preplanned:
                continue
            best = None
            tested = 0
            for cidx, c in enumerate(containers):
                packed = planned_packed[cidx]
                eps_all = self._generate_extreme_points(c, packed)
                eps = [ep for ep in eps_all if self._ep_location_key(ep) not in local_used[cidx]]
                for item in self._oriented_variants(raw_item):
                    for ep in eps:
                        tested += 1
                        cand = self._center_from_ep(ep, item)
                        if not self._inside_container(c, item, cand):
                            continue
                        if self._intersects_shelf_plate(c, item, cand):
                            continue
                        support = self._support_info(c, item, cand, packed)
                        if not support['ok']:
                            continue
                        if not self._final_clear(c, item, cand, packed, support_obj=support.get('support_obj')):
                            continue
                        score = self._score_candidate(c, raw_item, item, cand, ep, support, packed)
                        q = (score, cidx, cand, item, ep, support)
                        if best is None or (score, int(item['_orientation']), cidx) < (
                            best[0], int(best[3]['_orientation']), best[1]
                        ):
                            best = q
            if best is None:
                print(
                    f'[extreme_point_v01][PACK-PLAN] source={source} item={iid} NO-GEOMETRIC-CANDIDATE tested={tested}',
                    flush=True,
                )
                continue

            score, cidx, pos, item, ep, support = best
            ori = int(item['_orientation'])
            support_obj = support.get('support_obj')
            support_item_id = None
            if support_obj is not None and support_obj.get('index') is not None:
                support_item_id = int(support_obj['index'])

            rec = self._make_planned_packed_record(containers[cidx], item, pos, ori)
            plan = {
                'item_id': iid,
                'container_idx': int(cidx),
                'final_target': tuple(float(v) for v in pos),
                'orientation': ori,
                'item': dict(item),
                'ep': tuple(ep),
                'support_kind': support['kind'],
                'support_ratio': float(support['ratio']),
                'support_item_id': support_item_id,
                'score': float(score),
                'plan_rank': int(self.plan_counter),
                'packed_record': rec,
                'source': source,
            }
            self.plan_counter += 1
            self.preplanned[iid] = plan
            planned_packed[cidx].append(rec)
            local_used[cidx].add(self._ep_location_key(ep))
            print(
                f'[extreme_point_v01][PACK-PLAN] source={source} rank={plan["plan_rank"]} item={iid} '
                f'ori={ori} final_target=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) '
                f'support={support["kind"]} support_item={support_item_id} score={score:.5f}',
                flush=True,
            )

    def _make_planned_packed_record(self, c, item, local_pos, orientation):
        center = c.get('center', (0.0, 0.0, 0.0))
        rec = dict(item)
        rec['pos'] = (
            float(local_pos[0]) + float(center[0]),
            float(local_pos[1]) + float(center[1]),
            float(local_pos[2]) + float(center[2]),
        )
        rec['_orientation'] = int(orientation)
        rec['_already_oriented'] = True
        return rec

    def _rebuild_precedence(self, containers):
        pending = [p for iid, p in self.preplanned.items() if iid not in self.executed_ids]
        self.precedence_graph = self.insertion_planner.build_precedence(pending, containers)
        edges = []
        for a, dsts in self.precedence_graph.get('edges', {}).items():
            for b in sorted(dsts):
                rs = '+'.join(self.precedence_graph.get('reasons', {}).get((a, b), []))
                edges.append(f'{a}->{b}:{rs}')
        print(
            f'[extreme_point_v01][INSERT-GRAPH] edges={edges} '
            f'order={self.precedence_graph.get("order", [])} '
            f'cycles={self.precedence_graph.get("cyclic", [])}',
            flush=True,
        )

    def _ep_location_key(self, ep):
        """Quantized spatial key for an EP, intentionally ignoring source label.

        Two EPs at the same physical location are considered the same consumable
        placement anchor even if they were generated from different sources.
        """
        q = max(float(self.ep_quant), 1e-9)
        return (
            int(round(float(ep[0]) / q)),
            int(round(float(ep[1]) / q)),
            int(round(float(ep[2]) / q)),
        )

    # ------------------------------------------------------------------
    # Orientation / packed geometry
    # ------------------------------------------------------------------
    def _orientation_dims(self, item, orientation):
        L = float(item['length'])
        W = float(item['width'])
        H = float(item['height'])
        return {
            0: (L, W, H),
            1: (L, H, W),
            2: (H, W, L),
            3: (W, L, H),
            4: (W, H, L),
            5: (H, L, W),
        }[int(orientation)]

    def _oriented_variants(self, item):
        out = []
        seen = set()
        for ori in range(6):
            L, W, H = self._orientation_dims(item, ori)
            key = (round(L, 6), round(W, 6), round(H, 6))
            if key in seen:
                continue
            seen.add(key)
            v = dict(item)
            v['length'], v['width'], v['height'] = L, W, H
            v['_orientation'] = ori
            out.append(v)
        # Low orientation first is only a deterministic enumeration order; the
        # final choice is made by the score function.
        out.sort(key=lambda v: (
            float(v['height']),
            -(float(v['length']) * float(v['width'])),
            int(v['_orientation']),
        ))
        return out

    def _oriented_packed(self, packed):
        out = []
        for obj in packed:
            v = dict(obj)
            idx = v.get('index')
            if bool(v.get('_already_oriented', False)):
                ori = int(v.get('_orientation', 0))
            else:
                ori = self.placed_orientation.get(int(idx), 0) if idx is not None else 0
                L, W, H = self._orientation_dims(v, ori)
                v['length'], v['width'], v['height'] = L, W, H
                v['_orientation'] = ori
            out.append(v)
        return out

    def _record_internal_placement(self, cidx, c, item, local_pos, orientation):
        while len(self.internal_packed) <= cidx:
            self.internal_packed.append([])

        center = c.get('center', (0.0, 0.0, 0.0))
        world_pos = (
            float(local_pos[0]) + float(center[0]),
            float(local_pos[1]) + float(center[1]),
            float(local_pos[2]) + float(center[2]),
        )
        rec = dict(item)
        rec['pos'] = world_pos
        rec['_orientation'] = int(orientation)
        rec['_already_oriented'] = True

        idx = rec.get('index')
        if idx is not None:
            idx = int(idx)
            self.internal_packed[cidx] = [
                p for p in self.internal_packed[cidx]
                if p.get('index') is None or int(p.get('index')) != idx
            ]
        self.internal_packed[cidx].append(rec)

    def _effective_packed(self, cidx, c):
        """Merge settled simulator poses with just-issued internal poses.

        Simulator entries win for matching item ids because they contain the
        actual PyBullet-settled pose.  Internal entries only bridge the one-step
        observation lag immediately after returning an action.
        """
        sim_raw = c.get('packed_items', []) or []
        sim_packed = self._oriented_packed(sim_raw)
        sim_ids = {
            int(o['index']) for o in sim_packed
            if o.get('index') is not None
        }

        internal = self.internal_packed[cidx] if cidx < len(self.internal_packed) else []
        merged = list(sim_packed)
        for obj in internal:
            idx = obj.get('index')
            if idx is not None and int(idx) in sim_ids:
                continue
            merged.append(dict(obj))
        return merged

    # ------------------------------------------------------------------
    # Extreme Points
    # ------------------------------------------------------------------
    def _generate_extreme_points(self, c, packed):
        """Generate mirrored EPs: floor/shelf seeds + left/front/top of boxes.

        v01 intentionally keeps the generator compact.  Projected extreme points
        and dominance pruning beyond de-duplication are reserved for v02.
        """
        L, W, H = float(c['length']), float(c['width']), float(c['height'])
        t = float(c.get('thickness', 0.04))
        xmax = L/2 - t - self.boundary_margin
        ymax = W/2 - t - self.boundary_margin
        floor_z = t + self.boundary_margin
        cut_x = float(c.get('cut_x', 0.0))

        # The left opening/cut_x region is not a usable floor-entry seed.
        # Start the left-back floor seed just to the right of that region.
        xmin_inner = -L/2 + t + cut_x + self.boundary_margin
        # Only one floor seed.  After the first placement, floor expansion must
        # continue from right:/front: EPs generated by already placed boxes.
        # Keeping an independent floor-right-seed caused large unused gaps.
        raw = [
            (xmin_inner, ymax, floor_z, 'floor-left-seed'),
        ]

        # Small shelf is always present in this simulator family.
        buffer = float(c.get('buffer', 0.0))
        if cut_x > 0.0:
            ss_xmax = -L/2 + t + cut_x
            ss_ymax = W/2 - t
            ss_top = H/2 + buffer + t
            raw.append((ss_xmax - self.boundary_margin, ss_ymax - self.boundary_margin, ss_top + self.z_gap, 'small-shelf-seed'))

        # Optional main shelf, back half.
        if bool(c.get('shelf', c.get('require_shelf', False))):
            ms_xmax = (L - t) / 2.0
            ms_ymax = W/2 - t
            ms_top = H/2 + buffer + t
            raw.append((ms_xmax - self.boundary_margin, ms_ymax - self.boundary_margin, ms_top + self.z_gap, 'main-shelf-seed'))

        # Every packed item contributes three mirrored EPs.
        for obj in packed:
            if obj.get('pos') is None:
                continue
            ox, oy, oz = self._local_pos(c, obj['pos'])
            hx = float(obj['length']) / 2.0
            hy = float(obj['width']) / 2.0
            hz = float(obj['height']) / 2.0
            xmin, xmax_o = ox - hx, ox + hx
            ymin, ymax_o = oy - hy, oy + hy
            zmin, zmax_o = oz - hz, oz + hz

            # Lateral EPs on both sides. `right:*` stores the LEFT face of
            # the future candidate; `left:*` stores its RIGHT face.  These are
            # purely geometric packing anchors; insertion feasibility is handled
            # separately by InsertionPlanner.
            raw.append((xmin - self.ep_gap, ymax_o, zmin, f'left:{obj.get("index",-1)}'))
            raw.append((xmax_o + self.ep_gap, ymax_o, zmin, f'right:{obj.get("index",-1)}'))
            raw.append((xmax_o, ymin - self.ep_gap, zmin, f'front:{obj.get("index",-1)}'))
            raw.append((xmax_o, ymax_o, zmax_o + self.z_gap, f'top:{obj.get("index",-1)}'))

        # Quantized de-duplication.  Prefer lower z, then deeper y, then right x.
        dedup = {}
        for x, y, z, source in raw:
            key = (
                int(round(x / self.ep_quant)),
                int(round(y / self.ep_quant)),
                int(round(z / self.ep_quant)),
            )
            old = dedup.get(key)
            ep = (float(x), float(y), float(z), source)
            if old is None or (z, -y, -x) < (old[2], -old[1], -old[0]):
                dedup[key] = ep

        eps = list(dedup.values())
        eps.sort(key=lambda e: (e[2], -e[1], -e[0], e[3]))
        eps = eps[:self.max_eps_per_container]
        return eps

    def _center_from_ep(self, ep, item):
        x_edge, y_edge, z_bottom = ep[:3]
        source = ep[3] if len(ep) >= 4 else ''
        # Most EPs store the candidate's +X face.  The special left-seed and
        # right-expansion EPs store the candidate's -X face instead.
        if source == 'floor-left-seed' or source.startswith('right:'):
            cx = float(x_edge) + float(item['length']) / 2.0
        else:
            cx = float(x_edge) - float(item['length']) / 2.0
        return (
            cx,
            float(y_edge) - float(item['width']) / 2.0,
            float(z_bottom) + float(item['height']) / 2.0,
        )

    # ------------------------------------------------------------------
    # Feasibility
    # ------------------------------------------------------------------
    def _inside_container(self, c, item, cand):
        """Exact replica of PlacementValidator.check_inclusion().

        `cand` is the local action coordinate used by the agent.  The simulator
        stores container.points in world coordinates, while local_to_global()
        only adds the container X offset.  Therefore convert only X before
        evaluating the validator plane equations.

        Validator formula:
          dot(n, target-point) + dot(abs(n), half_lwh) <= inclusion_margin
        """
        n_vecs = c.get('n_vecs')
        points = c.get('points')
        if n_vecs is None or points is None:
            # Defensive fallback for non-simulator unit tests.
            L, W, H = float(c['length']), float(c['width']), float(c['height'])
            t = float(c.get('thickness', 0.04))
            x, y, z = map(float, cand)
            hx = float(item['length']) / 2.0
            hy = float(item['width']) / 2.0
            hz = float(item['height']) / 2.0
            m = self.boundary_margin
            return (
                x - hx >= -L/2 + t + m and
                x + hx <=  L/2 - t - m and
                y - hy >= -W/2 + t + m and
                y + hy <=  W/2 - t - m and
                z - hz >= t and
                z + hz <= H - t - 0.01
            )

        n = np.asarray(n_vecs, dtype=np.float64)
        p = np.asarray(points, dtype=np.float64)
        x, y, z = map(float, cand)
        # Container.local_to_global(local_pos) == (local_x + offset_x, y, z).
        # c['center'][0] equals offset_x for this simulator.
        offset_x = float(c.get('center', (0.0, 0.0, 0.0))[0])
        target_world = np.asarray((x + offset_x, y, z), dtype=np.float64)
        half_lwh = np.asarray((
            float(item['length']) / 2.0,
            float(item['width']) / 2.0,
            float(item['height']) / 2.0,
        ), dtype=np.float64)

        dots = (n * (target_world - p)).sum(axis=1) + np.dot(np.abs(n), half_lwh)
        ok = bool(np.all(dots <= float(self.inclusion_margin)))
        if not ok and os.environ.get('EP_DEBUG_INSIDE', '0') == '1':
            worst = int(np.argmax(dots))
            print(
                f'[extreme_point_v01][INSIDE-REJECT] plane={worst} '
                f'dot={float(dots[worst]):.6f} limit={self.inclusion_margin:.6f} '
                f'target_local=({x:.3f},{y:.3f},{z:.3f})',
                flush=True,
            )
        return ok

    def _support_info(self, c, item, cand, packed):
        """Require floor, shelf, or one-box support for every candidate."""
        x, y, z = map(float, cand)
        hx = float(item['length'])/2.0
        hy = float(item['width'])/2.0
        hz = float(item['height'])/2.0
        bottom = z - hz
        footprint = max(1e-12, 4.0 * hx * hy)

        L, W, H = float(c['length']), float(c['width']), float(c['height'])
        t = float(c.get('thickness', 0.04))
        buffer = float(c.get('buffer', 0.0))

        if abs(bottom - t) <= self.support_z_tolerance:
            return {'ok': True, 'kind': 'floor', 'ratio': 1.0, 'support_obj': None}

        # Small shelf support: narrow X strip, almost full Y depth.
        cut_x = float(c.get('cut_x', 0.0))
        shelf_top = H/2 + buffer + t
        if cut_x > 0 and abs(bottom - shelf_top) <= self.support_z_tolerance:
            sx0 = -L/2 + t
            sx1 = sx0 + cut_x
            sy0 = -W/2 + t
            sy1 = W/2 - t
            ratio = self._rect_overlap_ratio(x-hx, x+hx, y-hy, y+hy, sx0, sx1, sy0, sy1)
            # Outer wall side cannot overhang; retain a small inward clearance.
            outer_ok = (x-hx) >= sx0 + self.boundary_margin
            if outer_ok and ratio >= self.min_support_ratio:
                return {'ok': True, 'kind': 'small-shelf', 'ratio': ratio, 'support_obj': None}

        # Main shelf support: back half.
        if bool(c.get('shelf', c.get('require_shelf', False))) and abs(bottom - shelf_top) <= self.support_z_tolerance:
            ms_lx = L - t
            ms_ly = W/2 - 2*t
            sx0, sx1 = -ms_lx/2, ms_lx/2
            cy = W/4
            sy0, sy1 = cy - ms_ly/2, cy + ms_ly/2
            ratio = self._rect_overlap_ratio(x-hx, x+hx, y-hy, y+hy, sx0, sx1, sy0, sy1)
            if ratio >= self.min_support_ratio:
                return {'ok': True, 'kind': 'main-shelf', 'ratio': ratio, 'support_obj': None}

        # Single-box support.  v01 intentionally avoids multi-support unions.
        upper_mass = float(item.get('mass', 0.0))
        best = None
        for obj in packed:
            if obj.get('pos') is None:
                continue
            ox, oy, oz = self._local_pos(c, obj['pos'])
            ohx = float(obj['length'])/2.0
            ohy = float(obj['width'])/2.0
            ohz = float(obj['height'])/2.0
            top = oz + ohz
            if abs(bottom - top) > self.support_z_tolerance:
                continue
            ratio = self._rect_overlap_ratio(
                x-hx, x+hx, y-hy, y+hy,
                ox-ohx, ox+ohx, oy-ohy, oy+ohy,
            )
            if ratio < self.min_support_ratio:
                continue
            support_mass = float(obj.get('mass', 0.0))
            if upper_mass > 0 and support_mass > 0 and upper_mass > support_mass + self.weight_tolerance:
                continue
            if bool(obj.get('is_soft', False)) and not bool(item.get('is_soft', False)):
                continue
            rank = (-ratio, top)
            if best is None or rank < best[0]:
                best = (rank, obj, ratio)
        if best is not None:
            return {'ok': True, 'kind': 'box', 'ratio': best[2], 'support_obj': best[1]}

        return {'ok': False, 'kind': 'none', 'ratio': 0.0, 'support_obj': None}

    @staticmethod
    def _rect_overlap_ratio(ax0, ax1, ay0, ay1, bx0, bx1, by0, by1):
        ox = max(0.0, min(ax1, bx1) - max(ax0, bx0))
        oy = max(0.0, min(ay1, by1) - max(ay0, by0))
        area = max(1e-12, (ax1-ax0) * (ay1-ay0))
        return (ox * oy) / area

    def _intersects_shelf_plate(self, c, item, cand):
        """Reject final poses crossing through shelf plates themselves."""
        x, y, z = map(float, cand)
        hx, hy, hz = float(item['length'])/2, float(item['width'])/2, float(item['height'])/2
        L, W, H = float(c['length']), float(c['width']), float(c['height'])
        t = float(c.get('thickness', 0.04))
        buffer = float(c.get('buffer', 0.0))
        shelf_cz = H/2 + buffer + t/2

        def overlap_box(cx, cy, cz, lx, ly, lz):
            return (
                abs(x-cx) < hx + lx/2 and
                abs(y-cy) < hy + ly/2 and
                abs(z-cz) < hz + lz/2 - 1e-9
            )

        cut_x = float(c.get('cut_x', 0.0))
        if cut_x > 0:
            cx = -L/2 + t + cut_x/2
            if overlap_box(cx, 0.0, shelf_cz, cut_x, W-2*t, t):
                return True

        if bool(c.get('shelf', c.get('require_shelf', False))):
            if overlap_box(0.0, W/4, shelf_cz, L-t, W/2-2*t, t):
                return True
        return False

    # ------------------------------------------------------------------
    # Candidate scoring
    # ------------------------------------------------------------------
    def _score_candidate(self, c, raw_item, item, cand, ep, support, packed):
        """Lower is better.  v01 uses only transparent hand-tuned terms."""
        L, W, H = float(c['length']), float(c['width']), float(c['height'])
        x, y, z = map(float, cand)
        mass = float(raw_item.get('mass', 0.0))
        max_mass = self.global_max_mass if self.global_max_mass not in (None, 0.0) else max(mass, 1.0)

        # Normalize to approximately [0,1].
        height_n = z / max(H, 1e-9)
        front_n = (W/2 - y) / max(W, 1e-9)      # 0 at back, 1 at front
        left_n = (L/2 - x) / max(L, 1e-9)       # 0 at right, 1 at left
        mass_n = mass / max(max_mass, 1e-9)

        # Geometric compactness only.  Transport lanes and insertion order are
        # deliberately handled by InsertionPlanner, not by the BPP score.

        # Heavy cargo is more strongly rewarded for low Z.
        cog_term = height_n * (1.0 + 1.5 * mass_n)

        support_bonus = support['ratio']
        soft_penalty = 0.35 if bool(raw_item.get('is_soft', False)) and support['kind'] != 'box' else 0.0

        score = (
            3.0 * cog_term +
            2.5 * front_n +
            0.25 * left_n -
            1.5 * support_bonus +
            soft_penalty
        )
        return float(score)

    # ------------------------------------------------------------------
    # C++ checker integration
    # ------------------------------------------------------------------
    def _load_cpp_checker(self, module_path):
        if self.cpp_mode == 'off':
            print('[extreme_point_v01][cpp] mode=off')
            return

        candidates = []
        try:
            mp = Path(module_path).expanduser().resolve()
            project = mp.parent.parent if mp.name.startswith('extreme_point_') else mp
            candidates += [
                project / 'scripts' / 'cpp_aabb_checker_v05',
                project / 'scripts' / 'cpp_aabb_checker_v5',
            ]
        except Exception:
            pass
        cwd = Path.cwd().resolve()
        for base in (cwd, cwd.parent):
            candidates += [
                base / 'scripts' / 'cpp_aabb_checker_v05',
                base / 'scripts' / 'cpp_aabb_checker_v5',
            ]

        seen = set()
        last_error = None
        for p in candidates:
            ps = str(p)
            if ps in seen or not p.is_dir():
                continue
            seen.add(ps)
            try:
                if ps not in sys.path:
                    sys.path.insert(0, ps)
                import cpp_aabb_checker as checker
                if not hasattr(checker, 'aabb_overlap_python_order'):
                    raise RuntimeError(f'{p} lacks v05 final-overlap API')
                self.cpp_checker = checker
                self.cpp_checker_path = ps
                print(f'[extreme_point_v01][cpp] loaded mode={self.cpp_mode} path={p}')
                return
            except Exception as exc:
                last_error = exc
                sys.modules.pop('cpp_aabb_checker', None)

        if last_error is None:
            print('[extreme_point_v01][cpp] v05 checker not found; Python fallback')
        else:
            print(f'[extreme_point_v01][cpp] import failed: {last_error!r}; Python fallback')

    def _cpp_box(self, item, pos=(0.0, 0.0, 0.0), box_id=None):
        b = self.cpp_checker.Box()
        idx = item.get('index', -1) if box_id is None else box_id
        b.id = int(-1 if idx is None else idx)
        b.cx, b.cy, b.cz = map(float, pos)
        b.lx = float(item['length'])
        b.ly = float(item['width'])
        b.lz = float(item['height'])
        b.mass = float(item.get('mass', 0.0))
        b.is_soft = bool(item.get('is_soft', False))
        return b

    def _cpp_container_bounds(self, c):
        L, W, H = float(c['length']), float(c['width']), float(c['height'])
        t = float(c.get('thickness', 0.04))
        return self.cpp_checker.ContainerBounds(
            -L/2 + t, L/2 - t,
            -W/2 + t, W/2 - t,
            t, H - t,
        )

    def _cpp_packed_boxes(self, c, packed, skip_obj=None):
        boxes = []
        for obj in packed:
            if obj.get('pos') is None or (skip_obj is not None and obj is skip_obj):
                continue
            boxes.append(self._cpp_box(obj, self._local_pos(c, obj['pos'])))
        return boxes

    def _final_clear(self, c, item, cand, packed, support_obj=None):
        if self.cpp_checker is not None and self.cpp_mode != 'off':
            moving = self._cpp_box(item, cand)
            fn = self.cpp_checker.aabb_overlap_python_order
            for other in self._cpp_packed_boxes(c, packed, skip_obj=support_obj):
                if bool(fn(moving, other, float(self.xy_gap), float(self.z_gap))):
                    self.cpp_stats['final_cpp'] += 1
                    return False
            self.cpp_stats['final_cpp'] += 1
            return True

        self.cpp_stats['python_fallback_final'] += 1
        x, y, z = map(float, cand)
        hx, hy, hz = float(item['length'])/2, float(item['width'])/2, float(item['height'])/2
        for obj in packed:
            if obj.get('pos') is None or (support_obj is not None and obj is support_obj):
                continue
            ox, oy, oz = self._local_pos(c, obj['pos'])
            ohx, ohy, ohz = float(obj['length'])/2, float(obj['width'])/2, float(obj['height'])/2
            if (
                abs(x-ox) < hx+ohx+self.xy_gap and
                abs(y-oy) < hy+ohy+self.xy_gap and
                abs(z-oz) < hz+ohz+self.z_gap
            ):
                return False
        return True

    def _path_clear(self, c, item, target, packed):
        if self.cpp_checker is not None and self.cpp_mode != 'off':
            moving = self._cpp_box(item, (0.0, 0.0, 0.0))
            result = self.cpp_checker.check_transport_like_safe_grid(
                moving,
                self._cpp_container_bounds(c),
                self._cpp_packed_boxes(c, packed),
                float(target[0]), float(target[1]), float(target[2]),
                float(c.get('cut_x', 0.0)),
                float(c.get('thickness', 0.04)),
                float(self.start_margin),
                float(self.safety_margin),
                0.0,
                0.01,
                float(self.safety_margin),
            )
            self.cpp_stats['path_cpp'] += 1
            return bool(result['ok'])

        # Python fallback: same analytical two-segment approximation used in
        # safe_grid before C++ authority.
        self.cpp_stats['python_fallback_path'] += 1
        tx, ty, tz = map(float, target)
        L, W = float(c['length']), float(c['width'])
        t = float(c.get('thickness', 0.04))
        cut_x = float(c.get('cut_x', 0.0))
        hx, hy, hz = float(item['length'])/2, float(item['width'])/2, float(item['height'])/2
        x_min = -L/2 + t + cut_x + hx + max(self.start_margin, self.safety_margin + 0.003)
        x_max =  L/2 - t - hx - max(self.start_margin, self.safety_margin + 0.003)
        start_x = min(max(tx, x_min), x_max)
        start_y = -W/2

        floor_bottom = tz - hz
        path_z = tz if 0 <= floor_bottom-t <= 0.05 else min(
            tz + 0.08,
            float(c['height']) - t - hz - self.start_margin,
        )
        m = self.safety_margin
        for obj in packed:
            if obj.get('pos') is None:
                continue
            ox, oy, oz = self._local_pos(c, obj['pos'])
            ohx, ohy, ohz = float(obj['length'])/2, float(obj['width'])/2, float(obj['height'])/2
            if abs(path_z-oz) >= hz+ohz+m:
                continue
            ylo, yhi = sorted((start_y, ty))
            if abs(start_x-ox) < hx+ohx+m and not (yhi+hy <= oy-ohy-m or ylo-hy >= oy+ohy+m):
                return False
            xlo, xhi = sorted((start_x, tx))
            if abs(ty-oy) < hy+ohy+m and not (xhi+hx <= ox-ohx-m or xlo-hx >= ox+ohx+m):
                return False
        return True

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _local_pos(self, c, world_pos):
        center = c.get('center', (0.0, 0.0, 0.0))
        return (
            float(world_pos[0]) - float(center[0]),
            float(world_pos[1]) - float(center[1]),
            float(world_pos[2]) - float(center[2]),
        )

    def _fallback(self, container=None, item=None):
        z = 0.1
        if container is not None and item is not None:
            z = float(container.get('thickness', 0.04)) + float(item['height'])/2 + 0.006
        print(
            f'[extreme_point_v01] mode=NO-FEASIBLE-CANDIDATE '
            f'target=(0.000,0.000,{z:.3f}) stats={self.cpp_stats}'
        )
        return {
            'item_idx': 0,
            'container_idx': 0,
            'place_pos': np.asarray((0.0, 0.0, z), dtype=np.float32),
            'orientation': 0,
        }
