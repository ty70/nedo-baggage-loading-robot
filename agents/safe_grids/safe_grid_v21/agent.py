import math
import os
import sys
from pathlib import Path
import numpy as np


class Agent:
    """safe_grid_v21: v17 packing + Phase-5.1 hybrid C++ checker.

    - evaluate all 6 axis-aligned orientations
    - minimize oriented height when possible to lower center of gravity
    - enforce heavy-below-light stacking
    - use both the always-present small shelf and optional main shelf
    - deep (+Y) to front (-Y)
    - right (+X) to left (-X)
    - finish the deepest active tower before opening the next base
    - move tower-by-tower from back (+Y) to front (-Y)
    - conservative AABB approximation of validator Y->X transport path
    """

    def __init__(self, module_path: str):
        self.module_path = module_path
        self.safety_margin = 0.015
        self.extra_gap = 0.005
        self.scan_step = 0.02
        self.start_margin = 0.01
        self.max_stack_top = 1.48
        self.optimize_enabled = False
        self.lookahead_k = 1
        self.container_init = []
        self.last_target_xy = None
        self.stack_drop_clearance = 0.025
        self.stack_xy_inset = 0.003
        self.soft_item_ids = set()
        self.small_shelf_overhang = 0.006
        self.placed_orientation = {}
        self.weight_tolerance = 0.05
        self.global_max_mass = None

        # Phase 5.1: C++ checker integration.
        # - path_clear: C++ is authoritative whenever checker is available
        # - final_clear: Python remains authoritative in shadow mode
        # SAFE_GRID_CPP_MODE=off disables C++ entirely.
        # SAFE_GRID_CPP_MODE=cpp also makes final_clear C++ authoritative.
        self.cpp_mode = os.environ.get('SAFE_GRID_CPP_MODE', 'shadow').strip().lower()
        self.cpp_checker = None
        self.cpp_checker_path = None
        self.cpp_stats = {
            'final_total': 0, 'final_match': 0, 'final_mismatch': 0,
            'path_total': 0, 'path_match': 0, 'path_mismatch': 0, 'path_cpp_total': 0,
        }
        self.cpp_mismatch_print_limit = 30
        self.cpp_summary_interval = 5000
        self._load_cpp_checker(module_path)

    def get_init_states(self, init_states: dict):
        self.optimize_enabled = bool(init_states.get('optimize', False))
        self.lookahead_k = int(init_states.get('lookahead_k', 1))
        self.container_init = init_states.get('container_list', [])
        self.last_target_xy = None
        self.soft_item_ids = set()
        self.placed_orientation = {}
        self.global_max_mass = None
        # Reset per-episode comparison counters while keeping the imported module.
        for key in self.cpp_stats:
            self.cpp_stats[key] = 0
        if self.container_init:
            h = float(self.container_init[0].get('height', 1.6))
            t = float(self.container_init[0].get('thickness', 0.04))
            self.max_stack_top = h - t - 0.06
        return True

    def optimize(self, item_list: list):
        """v14 rules + heavy-first ordering inside each category.

        Global category order is unchanged:
          1) hard + non-priority
          2) hard + priority
          3) soft + non-priority
          4) soft + priority

        Within each category, mass is sorted descending.  Geometry is used only
        as a secondary key so heavier rigid cargo is exposed early enough to form
        the lowest layers of towers.
        """
        masses_all = [float(it.get('mass', 0.0)) for it in item_list]
        self.global_max_mass = max(masses_all) if masses_all else None

        for it in item_list:
            if bool(it.get('is_soft', False)):
                self.soft_item_ids.add(int(it['index']))

        def category(it):
            is_soft = bool(it.get('is_soft', False))
            is_priority = bool(it.get('is_prioritized', False))
            if not is_soft and not is_priority:
                return 0
            if not is_soft and is_priority:
                return 1
            if is_soft and not is_priority:
                return 2
            return 3

        buckets = {0: [], 1: [], 2: [], 3: []}
        for it in item_list:
            buckets[category(it)].append(it)

        order = []
        for bucket_id in (0, 1, 2, 3):
            group = sorted(
                buckets[bucket_id],
                key=lambda it: (
                    -float(it.get('mass', 0.0)),
                    -(float(it['length']) * float(it['width'])),
                    -float(it['height']),
                    int(it['index']),
                ),
            )
            order.extend(int(it['index']) for it in group)
        return order

    def policy(self, observation: dict):
        pool = observation.get('pool_list', [])
        containers = observation.get('container_list', [])
        if not pool or not containers:
            return self._fallback()

        for _it in pool:
            if bool(_it.get('is_soft', False)) and _it.get('index') is not None:
                self.soft_item_ids.add(int(_it['index']))

        masses = [float(it.get('mass', 0.0)) for it in pool]
        median_mass = float(np.median(masses)) if masses else 0.0
        max_visible_mass = max(masses) if masses else 0.0
        min_visible_mass = min(masses) if masses else 0.0

        def is_floor_heavy(mass):
            # optimize=True: we know the whole episode, so reserve strong floor
            # priority only for the true maximum-mass class.
            if self.global_max_mass is not None:
                return mass >= self.global_max_mass - self.weight_tolerance

            # optimize=False/look-ahead: only call something "floor-heavy" when
            # the visible pool actually contains a distinct heavy class.
            # If all visible items have almost the same mass, do not let that
            # suppress shelf usage.
            if max_visible_mass <= 0.0:
                return False
            if max_visible_mass - min_visible_mass <= self.weight_tolerance:
                return False
            heavy_cut = max(1.20 * median_mass, 0.90 * max_visible_mass)
            return mass >= heavy_cut - self.weight_tolerance

        # 1) Continue active tower.  Weight monotonicity is enforced inside
        # _continue_last_tower: an upper item cannot be materially heavier than
        # its support.  Among safe choices prefer rigid, then lighter/shorter
        # upper cargo, keeping tower mass concentrated low.
        if self.last_target_xy is not None:
            tower_choices = []
            for cidx, c in enumerate(containers):
                packed = self._oriented_packed(c.get('packed_items', []))
                for pidx, raw_item in enumerate(pool):
                    for item in self._oriented_variants(raw_item):
                        pos = self._continue_last_tower(c, item, packed)
                        if pos is None:
                            continue
                        mass = float(raw_item.get('mass', 0.0))
                        area = float(item['length']) * float(item['width'])
                        tower_choices.append((
                            bool(raw_item.get('is_soft', False)),
                            mass,
                            float(item['height']),
                            -area,
                            cidx, pidx, pos, item
                        ))
            if tower_choices:
                tower_choices.sort(key=lambda q: q[:4])
                _, _, _, _, cidx, pidx, pos, item = tower_choices[0]
                raw_item = pool[pidx]
                if bool(raw_item.get('is_soft', False)):
                    self.last_target_xy = None
                    mode = 'tower-stack-soft-cap'
                else:
                    self.last_target_xy = (float(pos[0]), float(pos[1]))
                    mode = 'tower-stack'
                return self._action(cidx, pidx, pos, mode=mode, item=item,
                                    orientation=int(item['_orientation']))

            self.last_target_xy = None

        # 2) Soft-only phase: cap an existing hard tower before creating a new
        # soft base.  Prefer the lowest reachable top, then deepest Y.
        visible_soft_only = bool(pool) and all(bool(it.get('is_soft', False)) for it in pool)
        if visible_soft_only:
            choices = []
            for cidx, c in enumerate(containers):
                packed = self._oriented_packed(c.get('packed_items', []))
                for pidx, raw_item in enumerate(pool):
                    for item in self._oriented_variants(raw_item):
                        pos = self._find_stack_position(c, item, packed)
                        if pos is None:
                            continue
                        choices.append((
                            float(pos[2]),
                            -float(pos[1]),
                            float(item['height']),
                            cidx, pidx, pos, item
                        ))
            if choices:
                choices.sort(key=lambda q: q[:3])
                _, _, _, cidx, pidx, pos, item = choices[0]
                self.last_target_xy = None
                return self._action(cidx, pidx, pos, mode='tower-soft-cap',
                                    item=item, orientation=int(item['_orientation']))

        # 3) Open a new base.
        # Only the true heaviest class strongly reserves FLOOR bases.
        # Medium/light cargo uses a safe shelf candidate before consuming floor.
        # This deliberately increases both small-shelf and main-shelf usage while
        # keeping the heavy-below-light stacking rule from v15.
        base_choices = []
        for cidx, c in enumerate(containers):
            packed = self._oriented_packed(c.get('packed_items', []))
            has_main_shelf = bool(c.get('shelf', c.get('require_shelf', False)))
            for pidx, raw_item in enumerate(pool):
                mass = float(raw_item.get('mass', 0.0))
                is_heavy = is_floor_heavy(mass)
                for item in self._oriented_variants(raw_item):
                    area = float(item['length']) * float(item['width'])

                    fpos = self._find_floor_position(c, item, packed)
                    if fpos is not None:
                        base_choices.append((
                            0 if is_heavy else 2,   # non-heavy floor yields to shelves
                            -mass,
                            -float(fpos[1]),
                            float(fpos[2]),
                            float(item['height']),
                            -area,
                            cidx, pidx, fpos, 'tower-base-floor', item
                        ))

                    sspos = self._find_small_shelf_position(c, item, packed)
                    if sspos is not None:
                        base_choices.append((
                            2 if is_heavy else 0,   # non-heavy small shelf first
                            -mass,
                            -float(sspos[1]),
                            float(sspos[2]),
                            float(item['height']),
                            -area,
                            cidx, pidx, sspos, 'tower-base-small-shelf', item
                        ))

                    if has_main_shelf:
                        spos = self._find_shelf_position(c, item, packed)
                        if spos is not None:
                            base_choices.append((
                                2 if is_heavy else 0,   # non-heavy main shelf first
                                -mass,
                                -float(spos[1]),
                                float(spos[2]),
                                float(item['height']),
                                -area,
                                cidx, pidx, spos, 'tower-base-main-shelf', item
                            ))

        if base_choices:
            # Preserve v14 soft rule: use a rigid base whenever any rigid base
            # candidate exists.
            hard_bases = [q for q in base_choices if not bool(pool[q[7]].get('is_soft', False))]
            selectable = hard_bases if hard_bases else base_choices
            selectable.sort(key=lambda q: q[:6])
            _, _, _, _, _, _, cidx, pidx, pos, mode, item = selectable[0]

            if bool(pool[pidx].get('is_soft', False)):
                self.last_target_xy = None
                mode += '-soft-cap'
            else:
                self.last_target_xy = (float(pos[0]), float(pos[1]))
            return self._action(cidx, pidx, pos, mode=mode, item=item,
                                orientation=int(item['_orientation']))

        # 4) Salvage stack.  Prefer low resulting Z and light upper items.
        stack_choices = []
        for cidx, c in enumerate(containers):
            packed = self._oriented_packed(c.get('packed_items', []))
            for pidx, raw_item in enumerate(pool):
                mass = float(raw_item.get('mass', 0.0))
                for item in self._oriented_variants(raw_item):
                    pos = self._find_stack_position(c, item, packed)
                    if pos is None:
                        continue
                    stack_choices.append((
                        bool(raw_item.get('is_soft', False)),
                        float(pos[2]),
                        mass,
                        -float(pos[1]),
                        float(item['height']),
                        cidx, pidx, pos, item
                    ))
        if stack_choices:
            stack_choices.sort(key=lambda q: q[:5])
            _, _, _, _, _, cidx, pidx, pos, item = stack_choices[0]
            if bool(pool[pidx].get('is_soft', False)):
                self.last_target_xy = None
                mode = 'tower-salvage-soft-cap'
            else:
                self.last_target_xy = (float(pos[0]), float(pos[1]))
                mode = 'tower-salvage'
            return self._action(cidx, pidx, pos, mode=mode, item=item,
                                orientation=int(item['_orientation']))

        # 5) Emergency stacking with the same weight constraint.
        emergency_choices = []
        for cidx, c in enumerate(containers):
            packed = self._oriented_packed(c.get('packed_items', []))
            for pidx, raw_item in enumerate(pool):
                mass = float(raw_item.get('mass', 0.0))
                for item in self._oriented_variants(raw_item):
                    pos = self._find_emergency_stack(c, item, packed)
                    if pos is not None:
                        emergency_choices.append((
                            bool(raw_item.get('is_soft', False)),
                            float(pos[2]),
                            mass,
                            float(item['height']),
                            cidx, pidx, pos, item
                        ))
        if emergency_choices:
            emergency_choices.sort(key=lambda q: q[:4])
            _, _, _, _, cidx, pidx, pos, item = emergency_choices[0]
            if bool(pool[pidx].get('is_soft', False)):
                self.last_target_xy = None
                mode = 'emergency-stack-soft-cap'
            else:
                self.last_target_xy = (float(pos[0]), float(pos[1]))
                mode = 'emergency-stack'
            return self._action(cidx, pidx, pos, mode=mode, item=item,
                                orientation=int(item['_orientation']))

        return self._fallback(containers[0], pool[0])

    def _orientation_dims(self, item, orientation):
        """Axis-aligned dimensions after one of the six allowed rotations."""
        L = float(item['length'])
        W = float(item['width'])
        H = float(item['height'])
        dims = {
            0: (L, W, H),
            1: (L, H, W),
            2: (H, W, L),
            3: (W, L, H),
            4: (W, H, L),
            5: (H, L, W),
        }
        return dims[int(orientation)]

    def _oriented_variants(self, item):
        """Unique orientations, low height first and broad footprint second."""
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
        out.sort(key=lambda v: (
            float(v['height']),
            -(float(v['length']) * float(v['width'])),
            int(v['_orientation']),
        ))
        return out

    def _oriented_packed(self, packed):
        """Apply remembered action orientation to packed-item geometry."""
        out = []
        for o in packed:
            v = dict(o)
            idx = v.get('index')
            ori = self.placed_orientation.get(int(idx), 0) if idx is not None else 0
            L, W, H = self._orientation_dims(v, ori)
            v['length'], v['width'], v['height'] = L, W, H
            v['_orientation'] = ori
            out.append(v)
        return out

    def _is_soft_obj(self, obj):
        if obj is None:
            return False
        if bool(obj.get('is_soft', False)):
            return True
        idx = obj.get('index')
        return idx is not None and int(idx) in self.soft_item_ids

    def _action(self, cidx, pidx, pos, mode='unknown', item=None, orientation=0):
        item_id = None if item is None else item.get('index')
        mass = 0.0 if item is None else float(item.get('mass', 0.0))
        if item_id is not None:
            self.placed_orientation[int(item_id)] = int(orientation)
        print(
            f'[safe_grid_v21] mode={mode} item={item_id} pool_idx={pidx} '
            f'mass={mass:.2f} ori={int(orientation)} '
            f'dims=({float(item["length"]):.3f},{float(item["width"]):.3f},{float(item["height"]):.3f}) '
            f'target=({float(pos[0]):.3f},{float(pos[1]):.3f},{float(pos[2]):.3f})'
            if item is not None else
            f'[safe_grid_v21] mode={mode} item=None pool_idx={pidx} '
            f'target=({float(pos[0]):.3f},{float(pos[1]):.3f},{float(pos[2]):.3f})'
        )
        return {
            'item_idx': int(pidx),
            'container_idx': int(cidx),
            'place_pos': np.asarray(pos, dtype=np.float32),
            'orientation': int(orientation),
        }

    def _continue_last_tower(self, c, item, packed):
        """Continue the physically settled tower nearest the previous target XY.

        v5 also prints a compact reject reason when the expected continuation fails.
        """
        if not packed or self.last_target_xy is None:
            return None

        ix, iy, ih = float(item['length']), float(item['width']), float(item['height'])
        lx, ly = self.last_target_xy
        supports = []
        drift_rejects = []
        footprint_rejects = []
        for o in packed:
            pos = o.get('pos')
            if pos is None:
                continue
            if self._is_soft_obj(o):
                continue
            if float(item.get('mass', 0.0)) > float(o.get('mass', 0.0)) + self.weight_tolerance:
                continue
            ox, oy, oz = self._local_pos(c, pos)
            dx, dy = abs(ox - lx), abs(oy - ly)
            if dx > 0.10 or dy > 0.10:
                drift_rejects.append((o.get('index'), ox, oy, dx, dy))
                continue
            # Soft cargo must remain the topmost item. Never load anything
            # (hard or soft) on a soft support.
            if self._is_soft_obj(o):
                footprint_rejects.append((o.get('index'), float(o['length']), float(o['width'])))
                continue

            sx, sy = float(o['length']), float(o['width'])
            if ix > sx + 0.005 or iy > sy + 0.005:
                footprint_rejects.append((o.get('index'), sx, sy))
                continue
            top = oz + float(o['height']) / 2.0
            supports.append((top, -dx-dy, ox, oy, o))

        if not supports:
            print(f'[safe_grid_v21][continue-reject] no-support last=({lx:.3f},{ly:.3f}) item=({ix:.3f},{iy:.3f},{ih:.3f}) packed={len(packed)}')
            if drift_rejects:
                print('[safe_grid_v21][continue-reject] drift=' + ', '.join(
                    f"id={r[0]} pos=({r[1]:.3f},{r[2]:.3f}) d=({r[3]:.3f},{r[4]:.3f})" for r in drift_rejects[:6]))
            if footprint_rejects:
                print('[safe_grid_v21][continue-reject] footprint=' + ', '.join(
                    f"id={r[0]} support=({r[1]:.3f},{r[2]:.3f})" for r in footprint_rejects[:6]))
            return None

        supports.sort(reverse=True, key=lambda q: (q[0], q[1]))
        rejects = []
        for top, _, ox, oy, support in supports:
            z = top + ih / 2.0 + self.stack_drop_clearance
            xy_candidates = self._stack_xy_candidates(c, item, ox, oy, support)
            if not xy_candidates:
                rejects.append((support.get('index'), 'support-overhang', (ox, oy, z), self.max_stack_top))
                continue
            limit = self.max_stack_top
            if bool(c.get('shelf', c.get('require_shelf', False))):
                shelf_front = float(c.get('thickness', 0.05))
                shelf_bottom = float(c['height'])/2.0 + float(c.get('buffer', 0.0))
                shelf_top = shelf_bottom + float(c.get('thickness', 0.05))
                support_half_y = float(support['width']) / 2.0
                # A floor support extending under the shelf must stay below its
                # underside. A support already resting on/above the shelf may grow
                # toward the container ceiling.
                if top < shelf_top - 0.02 and oy + support_half_y + self.safety_margin >= shelf_front:
                    limit = min(limit, shelf_bottom - self.safety_margin - 0.005)
            if z + ih/2.0 > limit:
                cx, cy = xy_candidates[0]
                rejects.append((support.get('index'), 'height', (cx, cy, z), limit))
                continue

            last_reason = 'no-xy'
            last_cand = (xy_candidates[0][0], xy_candidates[0][1], z)
            for cx, cy in xy_candidates:
                cand = (cx, cy, z)
                last_cand = cand
                if not self._inside_simple(c, item, cand):
                    last_reason = 'inside'
                    continue
                if not self._shelf_transport_clear(c, item, cand):
                    last_reason = 'shelf-path'
                    continue
                if not self._final_clear(c, item, cand, packed, support=support):
                    last_reason = 'final-clear'
                    continue
                if not self._path_clear(c, item, cand, packed):
                    last_reason = 'path-clear'
                    continue
                if abs(cx-ox) > 0.004 or abs(cy-oy) > 0.004:
                    print(f'[safe_grid_v21][stack-offset] support={support.get("index")} '
                          f'base=({ox:.3f},{oy:.3f}) chosen=({cx:.3f},{cy:.3f})')
                return cand
            rejects.append((support.get('index'), last_reason, last_cand, limit))

        print(f'[safe_grid_v21][continue-reject] candidates={len(supports)} last=({lx:.3f},{ly:.3f})')
        for sid, reason, cand, limit in rejects[:6]:
            print(f'[safe_grid_v21][continue-reject] support={sid} reason={reason} cand=({cand[0]:.3f},{cand[1]:.3f},{cand[2]:.3f}) limit={limit:.3f}')
        return None

    def _find_stack_position(self, c, item, packed):
        if not packed:
            return None
        ix = float(item['length'])
        iy = float(item['width'])
        ih = float(item['height'])

        # Build towers keyed by local XY; take the highest top at each XY.
        towers = []
        for o in packed:
            pos = o.get('pos')
            if pos is None:
                continue
            ox, oy, oz = self._local_pos(c, pos)
            top = oz + float(o['height']) / 2.0
            towers.append((oy, ox, top, o))

        # Deep towers first. Prefer exact same footprint; allow smaller on larger.
        if bool(c.get('shelf', c.get('require_shelf', False))):
            shelf_top = float(c['height'])/2.0 + float(c.get('thickness', 0.05))
            towers.sort(key=lambda q: (0 if q[2] > shelf_top + 0.02 else 1, -q[0], -q[1], q[2]))
        else:
            towers.sort(key=lambda q: (-q[0], -q[1], q[2]))
        for oy, ox, top, support in towers:
            if self._is_soft_obj(support):
                continue
            if float(item.get('mass', 0.0)) > float(support.get('mass', 0.0)) + self.weight_tolerance:
                continue
            sx = float(support['length'])
            sy = float(support['width'])
            if ix > sx + 0.005 or iy > sy + 0.005:
                continue

            # support must actually be the current top of this XY tower
            if any(abs(self._local_pos(c, p['pos'])[0] - ox) < 0.02 and
                   abs(self._local_pos(c, p['pos'])[1] - oy) < 0.02 and
                   self._local_pos(c, p['pos'])[2] + float(p['height'])/2.0 > top + 1e-4
                   for p in packed if p.get('pos') is not None):
                continue

            z = top + ih / 2.0 + self.stack_drop_clearance
            xy_candidates = self._stack_xy_candidates(c, item, ox, oy, support)
            if not xy_candidates:
                continue
            stack_top_limit = self.max_stack_top
            # In shelf containers the main shelf occupies the back half (local y >= thickness)
            # around height/2. Keep floor towers below its underside; do not try to use the shelf yet.
            if bool(c.get('shelf', c.get('require_shelf', False))):
                shelf_front = float(c.get('thickness', 0.05))
                shelf_bottom = float(c['height'])/2.0 + float(c.get('buffer', 0.0))
                shelf_top = shelf_bottom + float(c.get('thickness', 0.05))
                support_half_y = float(support['width']) / 2.0
                if top < shelf_top - 0.02 and oy + support_half_y + self.safety_margin >= shelf_front:
                    stack_top_limit = min(stack_top_limit, shelf_bottom - self.safety_margin - 0.005)
            if z + ih / 2.0 > stack_top_limit:
                continue
            for cx, cy in xy_candidates:
                cand = (cx, cy, z)
                if not self._inside_simple(c, item, cand):
                    continue
                if not self._shelf_transport_clear(c, item, cand):
                    continue
                if not self._final_clear(c, item, cand, packed, support=support):
                    continue
                if not self._path_clear(c, item, cand, packed):
                    continue
                if abs(cx-ox) > 0.004 or abs(cy-oy) > 0.004:
                    print(f'[safe_grid_v21][stack-offset] support={support.get("index")} '
                          f'base=({ox:.3f},{oy:.3f}) chosen=({cx:.3f},{cy:.3f})')
                return cand
        return None

    def _find_emergency_stack(self, c, item, packed):
        """Robustly continue on the highest physically settled support.

        Unlike normal tower detection, this does not require XY tower identity.
        It follows the actual settled support center, which tolerates PyBullet drift.
        """
        if not packed:
            return None
        ix, iy, ih = float(item['length']), float(item['width']), float(item['height'])
        supports = []
        for o in packed:
            if o.get('pos') is None:
                continue
            if self._is_soft_obj(o):
                continue
            if float(item.get('mass', 0.0)) > float(o.get('mass', 0.0)) + self.weight_tolerance:
                continue
            if ix > float(o['length']) + 0.005 or iy > float(o['width']) + 0.005:
                continue
            ox, oy, oz = self._local_pos(c, o['pos'])
            top = oz + float(o['height']) / 2.0
            supports.append((top, oy, ox, o))
        supports.sort(reverse=True, key=lambda q: q[0])
        for top, oy, ox, support in supports:
            # Keep a deliberate drop gap so validator's Y->X transport path passes
            # comfortably above the current support; PyBullet then settles onto it.
            z = top + ih / 2.0 + self.stack_drop_clearance
            cx, cy = self._clamp_stack_xy(c, item, ox, oy, support)
            if cx is None:
                continue
            if z + ih / 2.0 > self.max_stack_top:
                continue
            cand = (cx, cy, z)
            if not self._inside_simple(c, item, cand):
                continue
            if not self._shelf_transport_clear(c, item, cand):
                continue
            # Ignore the chosen support in final overlap, but still reject other boxes.
            if not self._final_clear(c, item, cand, packed, support=support):
                continue
            if not self._path_clear(c, item, cand, packed):
                continue
            return cand
        return None

    def _find_small_shelf_position(self, c, item, packed):
        """Use the always-present narrow shelf on the cut_x side.

        v17 fixes the v16 failure seen in case000:
        the previous code centered an item on the narrow shelf and allowed
        symmetric X overhang.  Because the shelf touches the outer container
        wall, overhang toward that wall is impossible: a 0.45 m item centered
        on a 0.44 m shelf protrudes 5 mm through the wall.

        Therefore:
          - no overhang is allowed toward the outer wall;
          - the item must stay slightly inside the physical container wall;
          - only a tiny overhang toward the container interior is allowed;
          - rotated dimensions are still supported.
        """
        L, W, H = float(c['length']), float(c['width']), float(c['height'])
        t = float(c.get('thickness', 0.04))
        cut_x = float(c.get('cut_x', 0.0))
        buffer = float(c.get('buffer', 0.0))
        if cut_x <= 0.0:
            return None

        ix = float(item['length'])
        iy = float(item['width'])
        ih = float(item['height'])
        hx, hy, hz = ix/2.0, iy/2.0, ih/2.0

        # Physical small-shelf interval.  Its outer edge is flush with the
        # container's inner wall, so the outer side may never overhang.
        shelf_x0 = -L/2.0 + t
        shelf_x1 = shelf_x0 + cut_x

        # Inclusion validator uses a -5 mm margin.  Keep one extra millimetre
        # of numerical/settling clearance rather than targeting the exact plane.
        wall_clear = 0.006

        # Left/outer edge must be inside the wall by wall_clear.
        x_min = shelf_x0 + wall_clear + hx

        # Right/interior edge may overhang the narrow shelf only by the small
        # amount already permitted by the agent.
        x_max = shelf_x1 + self.small_shelf_overhang - hx

        if x_min > x_max:
            return None

        # Prefer the position closest to the shelf centre, but clamp it into
        # the physically valid asymmetric interval.
        shelf_center = (shelf_x0 + shelf_x1) / 2.0
        x = min(max(shelf_center, x_min), x_max)

        gap = self.safety_margin + self.extra_gap
        y_min = -W/2.0 + t + hy + gap
        y_max =  W/2.0 - t - hy - gap
        if y_min > y_max:
            return None

        shelf_top = H/2.0 + buffer + t
        drop_clearance = self.safety_margin + 0.006
        z = shelf_top + drop_clearance + hz

        # Physical inclusion bounds independent of cut_x insertion geometry.
        if x - hx < -L/2.0 + t + wall_clear:
            return None
        if x + hx >  L/2.0 - t - wall_clear:
            return None
        if z - hz < t + wall_clear:
            return None
        if z + hz > H - t - wall_clear:
            return None
        if z + hz > self.max_stack_top:
            return None

        # Deepest free shelf position first.  Validate physical container
        # bounds before collision/path checks.
        y = y_max
        while y >= y_min - 1e-9:
            if (y - hy >= -W/2.0 + t + wall_clear and
                    y + hy <= W/2.0 - t - wall_clear):
                cand = (float(x), float(y), float(z))
                if (self._final_clear(c, item, cand, packed)
                        and self._path_clear(c, item, cand, packed)):
                    return cand
            y -= self.scan_step
        return None

    def _find_shelf_position(self, c, item, packed):
        if not bool(c.get('shelf', c.get('require_shelf', False))):
            return None
        L, W = float(c['length']), float(c['width'])
        t = float(c.get('thickness', 0.05))
        cut_x = float(c.get('cut_x', 0.0))
        hx = float(item['length']) / 2.0
        hy = float(item['width']) / 2.0
        hz = float(item['height']) / 2.0
        gap = self.safety_margin + self.extra_gap

        # Main shelf covers nearly the full X width.  Do not apply cut_x to
        # the final target: the validator clips only the insertion start X and
        # then performs the X motion after reaching target Y.
        x_min = -L/2 + t + hx + self.safety_margin + 0.003
        x_max =  L/2 - t - hx - self.safety_margin - 0.003
        y_min = t + hy + gap
        y_max = W/2 - t - hy - gap
        if x_min > x_max or y_min > y_max:
            return None

        shelf_top = float(c['height'])/2.0 + t
        # Keep > safety_margin above shelf during transport, but <=5cm so validator
        # disables the extra start_z lift. It then settles only ~2cm onto shelf.
        drop_clearance = self.safety_margin + 0.006
        z = shelf_top + drop_clearance + hz
        if z + hz > self.max_stack_top:
            return None

        y = y_max
        while y >= y_min - 1e-9:
            x = x_max
            while x >= x_min - 1e-9:
                cand = (float(x), float(y), float(z))
                if self._final_clear(c, item, cand, packed) and self._path_clear(c, item, cand, packed):
                    return cand
                x -= self.scan_step
            y -= self.scan_step
        return None

    def _find_floor_position(self, c, item, packed):
        L, W = float(c['length']), float(c['width'])
        t = float(c.get('thickness', 0.04))
        cut_x = float(c.get('cut_x', 0.0))
        hx = float(item['length']) / 2.0
        hy = float(item['width']) / 2.0
        hz = float(item['height']) / 2.0
        gap = self.safety_margin + self.extra_gap
        z = t + hz + 0.006

        # Same conservative X interval validator uses for transport start.
        x_min = -L/2 + t + cut_x + hx + max(self.start_margin, self.safety_margin + 0.003)
        x_max =  L/2 - t - hx - max(self.start_margin, self.safety_margin + 0.003)
        y_min = -W/2 + t + hy + gap
        y_max =  W/2 - t - hy - gap
        if x_min > x_max or y_min > y_max:
            return None

        # Candidate positions: deep->front.  Besides the regular scan grid, add
        # exact contact-aligned Y positions immediately in front of existing cargo.
        # This removes unnecessary 1-2 scan-step gaps without altering shelf priority.
        y_candidates = []
        y = y_max
        while y >= y_min - 1e-9:
            y_candidates.append(float(y))
            y -= self.scan_step

        # Opening/front is -Y; to pack a new item immediately in front of an
        # existing one, align its back edge just ahead of the existing front edge.
        for o in packed:
            if o.get('pos') is None:
                continue
            ox, oy, oz = self._local_pos(c, o['pos'])
            ohy = float(o['width']) / 2.0
            contact_y = oy - ohy - hy - gap
            if y_min - 1e-9 <= contact_y <= y_max + 1e-9:
                y_candidates.append(float(contact_y))

        # Preserve exact floating values; no rounding, so shelf/floor depth ties
        # remain governed by the existing z tie-breaker.
        y_candidates = sorted(set(y_candidates), reverse=True)
        for y in y_candidates:
            x = x_max
            while x >= x_min - 1e-9:
                cand = (float(x), float(y), float(z))
                if self._final_clear(c, item, cand, packed) and self._path_clear(c, item, cand, packed):
                    return cand
                x -= self.scan_step
        return None

    def _clamp_stack_xy(self, c, item, ox, oy, support):
        """Move a settled support center a few mm inward before stacking.

        PyBullet commonly shifts a floor box ~0.1-0.3 mm past our exact scan
        boundary. Reusing that XY verbatim makes our own inclusion precheck reject
        the next level. Clamp inward while keeping the upper footprint supported.
        """
        L, W = float(c['length']), float(c['width'])
        t = float(c.get('thickness', 0.04))
        cut_x = float(c.get('cut_x', 0.0))
        hx, hy = float(item['length'])/2.0, float(item['width'])/2.0
        x_min = -L/2 + t + cut_x + hx + max(self.start_margin, self.safety_margin + 0.003)
        x_max =  L/2 - t - hx - max(self.start_margin, self.safety_margin + 0.003)
        y_min = -W/2 + t + hy
        y_max =  W/2 - t - hy
        cx = min(max(float(ox), x_min + self.stack_xy_inset), x_max - self.stack_xy_inset)
        cy = min(max(float(oy), y_min + self.stack_xy_inset), y_max - self.stack_xy_inset)

        # Do not shift so far that the upper box loses practical support.
        sx, sy = float(support['length']), float(support['width'])
        overhang_tol = 0.006
        if abs(cx-float(ox)) + hx > sx/2.0 + overhang_tol:
            return None, None
        if abs(cy-float(oy)) + hy > sy/2.0 + overhang_tol:
            return None, None
        return cx, cy

    def _stack_xy_candidates(self, c, item, ox, oy, support):
        """Return supported XY candidates, preferring shifts toward the back (+Y).

        v8 tested only one almost-centered position on a support.  If that position
        was blocked by a neighbouring tower, the whole support was discarded even
        when the upper item was smaller and could safely slide a few centimetres
        within the support footprint.
        """
        base_x, base_y = self._clamp_stack_xy(c, item, ox, oy, support)
        if base_x is None:
            return []

        ix, iy = float(item['length']), float(item['width'])
        sx, sy = float(support['length']), float(support['width'])

        # Practical support tolerance is the same one used by _clamp_stack_xy.
        tol = 0.006
        slack_x = max(0.0, (sx - ix) / 2.0 + tol)
        slack_y = max(0.0, (sy - iy) / 2.0 + tol)

        # Prefer the back (+Y).  Try a modest 30 mm move first, then the full
        # support-safe shift.  X shifts are secondary escape routes.
        def mags(slack):
            vals = [0.0]
            if slack > 1e-6:
                vals.append(min(0.03, slack))
                if slack > 0.031:
                    vals.append(slack)
            return vals

        y_pos = mags(slack_y)
        x_pos = mags(slack_x)

        offsets = []
        # Center first preserves v8 behaviour when it already works.
        offsets.append((0.0, 0.0))
        # Then favour depth.
        for dy in y_pos[1:]:
            offsets.append((0.0, +dy))
        # Combine a back shift with lateral adjustment.
        for dy in y_pos[1:] or [0.0]:
            for dx in x_pos[1:]:
                offsets.append((-dx, +dy))
                offsets.append((+dx, +dy))
        # Lateral-only fallbacks.
        for dx in x_pos[1:]:
            offsets.append((-dx, 0.0))
            offsets.append((+dx, 0.0))
        # Front shifts are last resort.
        for dy in y_pos[1:]:
            offsets.append((0.0, -dy))

        L, W = float(c['length']), float(c['width'])
        t = float(c.get('thickness', 0.04))
        cut_x = float(c.get('cut_x', 0.0))
        hx, hy = ix/2.0, iy/2.0
        x_min = -L/2 + t + cut_x + hx + max(self.start_margin, self.safety_margin + 0.003)
        x_max =  L/2 - t - hx - max(self.start_margin, self.safety_margin + 0.003)
        y_min = -W/2 + t + hy
        y_max =  W/2 - t - hy

        out = []
        seen = set()
        for dx, dy in offsets:
            cx = min(max(base_x + dx, x_min + self.stack_xy_inset),
                     x_max - self.stack_xy_inset)
            cy = min(max(base_y + dy, y_min + self.stack_xy_inset),
                     y_max - self.stack_xy_inset)

            # Keep the upper footprint actually supported.
            if abs(cx - float(ox)) + hx > sx/2.0 + tol:
                continue
            if abs(cy - float(oy)) + hy > sy/2.0 + tol:
                continue
            key = (round(cx, 4), round(cy, 4))
            if key not in seen:
                seen.add(key)
                out.append((float(cx), float(cy)))
        return out

    def _shelf_transport_clear(self, c, item, target):
        """Conservative main-shelf check for validator's actual +0.08m transport."""
        if not bool(c.get('shelf', c.get('require_shelf', False))):
            return True
        t = float(c.get('thickness', 0.05))
        H = float(c['height'])
        buffer = float(c.get('buffer', 0.0))
        _, ty, tz = target
        hy, hz = float(item['width'])/2.0, float(item['height'])/2.0

        # Main shelf spans the back half: local y >= t, with underside H/2+buffer.
        shelf_front = t
        shelf_bottom = H/2.0 + buffer
        shelf_top = shelf_bottom + t
        floor_bottom = tz - hz
        path_z = tz if 0 <= floor_bottom-t <= 0.05 else min(
            tz + 0.08, H - t - hz - self.start_margin)
        m = self.safety_margin
        y_overlaps = ty + hy + m >= shelf_front
        z_overlaps = (path_z + hz + m > shelf_bottom and
                      path_z - hz - m < shelf_top)
        return not (y_overlaps and z_overlaps)

    def _inside_simple(self, c, item, cand):
        L, W, H = float(c['length']), float(c['width']), float(c['height'])
        t = float(c.get('thickness', 0.04))
        cut_x = float(c.get('cut_x', 0.0))
        x, y, z = cand
        hx = float(item['length'])/2
        hy = float(item['width'])/2
        hz = float(item['height'])/2
        # Conservative bounds; x_min mirrors validator safe entrance band.
        x_min = -L/2 + t + cut_x + hx + max(self.start_margin, self.safety_margin + 0.003)
        x_max =  L/2 - t - hx - max(self.start_margin, self.safety_margin + 0.003)
        return (x_min <= x <= x_max and
                -W/2 + t + hy <= y <= W/2 - t - hy and
                t + hz <= z <= H - t - hz - 0.01)

    def _local_pos(self, c, world_pos):
        center = c.get('center', (0.0, 0.0, 0.0))
        return (float(world_pos[0]) - float(center[0]), float(world_pos[1]), float(world_pos[2]))

    # ------------------------------------------------------------------
    # Phase 5 C++ shadow checker integration
    # ------------------------------------------------------------------
    def _load_cpp_checker(self, module_path):
        """Locate simulator/scripts/cpp_aabb_checker_v05/v5 and import the extension.

        Expected user layout:
          simulator/
            agents/safe_grid_v18/agent.py
            scripts/cpp_aabb_checker_v4/cpp_aabb_checker*.so

        We also accept cpp_aabb_checker_v04 for compatibility with the archive
        name used during development.
        """
        if self.cpp_mode == 'off':
            print('[safe_grid_v21][cpp] mode=off')
            return

        candidates = []
        try:
            mp = Path(module_path).expanduser().resolve()
            # module_path is normally simulator/agents/safe_grid_v18/
            project = mp.parent.parent if mp.name.startswith('safe_grid_') else mp
            candidates += [
                project / 'scripts' / 'cpp_aabb_checker_v05',
                project / 'scripts' / 'cpp_aabb_checker_v5',
                project / 'scripts' / 'cpp_aabb_checker_v4',
                project / 'scripts' / 'cpp_aabb_checker_v04',
            ]
        except Exception:
            pass

        cwd = Path.cwd().resolve()
        for base in (cwd, cwd.parent):
            candidates += [
                base / 'scripts' / 'cpp_aabb_checker_v05',
                base / 'scripts' / 'cpp_aabb_checker_v5',
                base / 'scripts' / 'cpp_aabb_checker_v4',
                base / 'scripts' / 'cpp_aabb_checker_v04',
            ]

        # De-duplicate while preserving order.
        seen = set()
        unique = []
        for p in candidates:
            ps = str(p)
            if ps not in seen:
                seen.add(ps)
                unique.append(p)

        last_error = None
        for p in unique:
            if not p.is_dir():
                continue
            try:
                if str(p) not in sys.path:
                    sys.path.insert(0, str(p))
                import cpp_aabb_checker as checker
                self.cpp_checker = checker
                self.cpp_checker_path = str(p)
                print(f'[safe_grid_v21][cpp] loaded mode={self.cpp_mode} path={p}')
                return
            except Exception as e:
                last_error = e

        self.cpp_checker = None
        if last_error is None:
            print('[safe_grid_v21][cpp] checker directory not found; Python-only fallback')
        else:
            print(f'[safe_grid_v21][cpp] import failed: {last_error!r}; Python-only fallback')

    def _cpp_box(self, item, pos=(0.0, 0.0, 0.0), box_id=None):
        if self.cpp_checker is None:
            return None
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
        for o in packed:
            if o.get('pos') is None or (skip_obj is not None and o is skip_obj):
                continue
            pos = self._local_pos(c, o['pos'])
            boxes.append(self._cpp_box(o, pos))
        return boxes

    def _cpp_record(self, kind, py_ok, cpp_ok, item=None, target=None, detail=None):
        total_key = f'{kind}_total'
        match_key = f'{kind}_match'
        mismatch_key = f'{kind}_mismatch'
        self.cpp_stats[total_key] += 1
        if bool(py_ok) == bool(cpp_ok):
            self.cpp_stats[match_key] += 1
        else:
            self.cpp_stats[mismatch_key] += 1
            n = self.cpp_stats[mismatch_key]
            if n <= self.cpp_mismatch_print_limit:
                iid = None if item is None else item.get('index')
                target_s = 'None' if target is None else '(' + ','.join(f'{float(v):.3f}' for v in target) + ')'
                print(
                    f'[safe_grid_v21][cpp-mismatch] kind={kind} n={n} item={iid} '
                    f'python={bool(py_ok)} cpp={bool(cpp_ok)} target={target_s} detail={detail}'
                )

        n_total = self.cpp_stats[total_key]
        if n_total and n_total % self.cpp_summary_interval == 0:
            self._print_cpp_summary(prefix=f'{kind}@{n_total}')

    def _print_cpp_summary(self, prefix='summary'):
        ft = self.cpp_stats['final_total']
        fm = self.cpp_stats['final_match']
        path_cpp_total = self.cpp_stats.get('path_cpp_total', 0)
        fpct = 100.0 * fm / ft if ft else 100.0
        print(
            f'[safe_grid_v21][cpp-{prefix}] '
            f'final={fm}/{ft} ({fpct:.3f}%) mismatch={self.cpp_stats["final_mismatch"]} '
            f'path_cpp_authority={path_cpp_total}'
        )

    def _cpp_final_clear(self, c, item, cand, packed, support=None, debug_id=None):
        """C++ equivalent of v17 _final_clear, with high-precision pair diagnostics."""
        if self.cpp_checker is None:
            return None, None
        m = self.safety_margin + self.extra_gap
        moving = self._cpp_box(item, cand, box_id=item.get('index', -1))

        # v21/v05: preserve the original dimensions and let C++ evaluate
        # exactly the Python expression order: half_a + half_b + margin.
        # This avoids the ~5e-17 m boundary drift caused by pre-expanding
        # moving.lx/ly/lz and then averaging the dimensions.
        overlap_fn = getattr(self.cpp_checker, 'aabb_overlap_python_order', None)
        debug_fn = getattr(self.cpp_checker, 'aabb_overlap_python_order_debug', None)
        if overlap_fn is None:
            return None, {'reason': 'cpp_checker_v05_function_missing'}

        hit_ids = []
        debug_pair = None
        for p in self._cpp_packed_boxes(c, packed, skip_obj=support):
            overlap = bool(overlap_fn(moving, p, float(m), 0.003))
            if overlap:
                hit_ids.append(int(p.id))
            if debug_id is not None and int(p.id) == int(debug_id):
                dbg = dict(debug_fn(moving, p, float(m), 0.003)) if debug_fn is not None else {}
                debug_pair = {
                    'id': int(p.id),
                    'moving_center': tuple(format(float(v), '.17g') for v in (moving.cx, moving.cy, moving.cz)),
                    'moving_dims': tuple(format(float(v), '.17g') for v in (moving.lx, moving.ly, moving.lz)),
                    'packed_center': tuple(format(float(v), '.17g') for v in (p.cx, p.cy, p.cz)),
                    'packed_dims': tuple(format(float(v), '.17g') for v in (p.lx, p.ly, p.lz)),
                    'delta': tuple(format(float(v), '.17g') for v in dbg.get('delta', ())),
                    'limits': tuple(format(float(v), '.17g') for v in dbg.get('limits', ())),
                    'delta_minus_limit': tuple(format(float(v), '.17g') for v in dbg.get('delta_minus_limit', ())),
                    'axis_lt': tuple(bool(v) for v in dbg.get('axis_lt', ())),
                    'aabb_overlap': overlap,
                    'method': 'half_a_plus_half_b_plus_margin',
                }
        return len(hit_ids) == 0, {'collision_ids': hit_ids, 'debug_pair': debug_pair}

    def _cpp_path_clear(self, c, item, target, packed, extra_lift=0.0):
        """C++ 1cm sweep equivalent of v17 Y->X transport check."""
        if self.cpp_checker is None:
            return None, None
        moving = self._cpp_box(item, (0.0, 0.0, 0.0), box_id=item.get('index', -1))
        boxes = self._cpp_packed_boxes(c, packed)
        bounds = self._cpp_container_bounds(c)
        result = self.cpp_checker.check_transport_like_safe_grid(
            moving,
            bounds,
            boxes,
            float(target[0]), float(target[1]), float(target[2]),
            float(c.get('cut_x', 0.0)),
            float(c.get('thickness', 0.04)),
            float(self.start_margin),
            float(self.safety_margin),
            float(extra_lift),
            0.01,
            float(self.safety_margin),
        )
        return bool(result['ok']), dict(result)

    def _final_clear(self, c, item, cand, packed, support=None):
        # Phase 5.1: Python remains authoritative for final placement in shadow mode.
        # On mismatch, log the Python collision item and geometry so the C++ AABB
        # conversion can be diagnosed directly.
        x, y, z = cand
        hx, hy, hz = float(item['length'])/2, float(item['width'])/2, float(item['height'])/2
        m = self.safety_margin + self.extra_gap
        py_ok = True
        py_hit = None
        for o in packed:
            if o.get('pos') is None:
                continue
            if support is not None and o is support:
                continue
            ox, oy, oz = self._local_pos(c, o['pos'])
            ohx, ohy, ohz = float(o['length'])/2, float(o['width'])/2, float(o['height'])/2
            if (abs(x-ox) < hx+ohx+m and
                abs(y-oy) < hy+ohy+m and
                abs(z-oz) < hz+ohz+0.003):
                py_ok = False
                dx = abs(float(x-ox)); dy = abs(float(y-oy)); dz = abs(float(z-oz))
                lx = float(hx+ohx+m); ly = float(hy+ohy+m); lz = float(hz+ohz+0.003)
                py_hit = {
                    'id': int(o.get('index', -1)),
                    'pos': tuple(format(float(v), '.17g') for v in (ox, oy, oz)),
                    'dims': tuple(format(float(v), '.17g') for v in (o['length'], o['width'], o['height'])),
                    'delta': tuple(format(v, '.17g') for v in (dx, dy, dz)),
                    'limits': tuple(format(v, '.17g') for v in (lx, ly, lz)),
                    'delta_minus_limit': tuple(format(a-b, '.17g') for a,b in ((dx,lx),(dy,ly),(dz,lz))),
                    'axis_lt': (dx < lx, dy < ly, dz < lz),
                }
                break

        if self.cpp_mode != 'off':
            debug_id = None if py_hit is None else py_hit['id']
            cpp_ok, cpp_detail = self._cpp_final_clear(c, item, cand, packed, support=support, debug_id=debug_id)
            if cpp_ok is not None:
                detail = {
                    'python_collision': py_hit,
                    'cpp_collision_ids': cpp_detail.get('collision_ids', []) if cpp_detail else [],
                    'cpp_pair': cpp_detail.get('debug_pair') if cpp_detail else None,
                }
                self._cpp_record('final', py_ok, cpp_ok, item=item, target=cand, detail=detail)
                if self.cpp_mode == 'cpp':
                    return cpp_ok
        return py_ok

    def _path_clear_python(self, c, item, target, packed, extra_lift=0.0):
        """Original v17 analytical path check, used only if C++ is unavailable/off."""
        tx, ty, tz = target
        L, W = float(c['length']), float(c['width'])
        t = float(c.get('thickness', 0.04))
        cut_x = float(c.get('cut_x', 0.0))
        hx, hy, hz = float(item['length'])/2, float(item['width'])/2, float(item['height'])/2
        x_min = -L/2 + t + cut_x + hx + max(self.start_margin, self.safety_margin + 0.003)
        x_max =  L/2 - t - hx - max(self.start_margin, self.safety_margin + 0.003)
        start_x = min(max(tx, x_min), x_max)
        start_y = -W/2

        floor_bottom = tz - hz
        floor_z = t
        path_z = tz if 0 <= floor_bottom-floor_z <= 0.05 else min(
            tz + 0.08 + extra_lift,
            float(c['height']) - t - hz - self.start_margin
        )
        m = self.safety_margin

        for o in packed:
            if o.get('pos') is None:
                continue
            ox, oy, oz = self._local_pos(c, o['pos'])
            ohx, ohy, ohz = float(o['length'])/2, float(o['width'])/2, float(o['height'])/2
            if abs(path_z-oz) >= hz+ohz+m:
                continue

            ylo, yhi = sorted((start_y, ty))
            if (abs(start_x-ox) < hx+ohx+m and
                not (yhi + hy <= oy-ohy-m or ylo - hy >= oy+ohy+m)):
                return False

            xlo, xhi = sorted((start_x, tx))
            if (abs(ty-oy) < hy+ohy+m and
                not (xhi + hx <= ox-ohx-m or xlo - hx >= ox+ohx+m)):
                return False
        return True

    def _path_clear(self, c, item, target, packed, extra_lift=0.0):
        # Phase 5.1: after 100% agreement in both sample cases, C++ path checking
        # is now authoritative.  Do not run the Python path checker in parallel.
        if self.cpp_checker is not None and self.cpp_mode != 'off':
            cpp_ok, _detail = self._cpp_path_clear(c, item, target, packed, extra_lift=extra_lift)
            if cpp_ok is not None:
                self.cpp_stats['path_cpp_total'] += 1
                return cpp_ok

        # Safe fallback for environments where the extension cannot be imported.
        return self._path_clear_python(c, item, target, packed, extra_lift=extra_lift)

    def _fallback(self, container=None, item=None):
        if self.cpp_checker is not None:
            self._print_cpp_summary(prefix='fallback')
        z = 0.1
        if container is not None and item is not None:
            z = float(container.get('thickness', 0.04)) + float(item['height'])/2 + 0.006
        print(f'[safe_grid_v21] mode=NO-SAFE-CANDIDATE target=(0.000,0.000,{z:.3f})')
        return {
            'item_idx': 0,
            'container_idx': 0,
            'place_pos': np.asarray([0.0, 0.0, z], dtype=np.float32),
            'orientation': 0,
        }
