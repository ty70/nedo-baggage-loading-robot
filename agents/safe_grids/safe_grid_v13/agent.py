import math
import numpy as np


class Agent:
    """Simple robust column-packing baseline.

    - orientation 0 only (no rotation yet)
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

    def get_init_states(self, init_states: dict):
        self.optimize_enabled = bool(init_states.get('optimize', False))
        self.lookahead_k = int(init_states.get('lookahead_k', 1))
        self.container_init = init_states.get('container_list', [])
        self.last_target_xy = None
        self.soft_item_ids = set()
        if self.container_init:
            h = float(self.container_init[0].get('height', 1.6))
            t = float(self.container_init[0].get('thickness', 0.04))
            self.max_stack_top = h - t - 0.06
        return True

    def optimize(self, item_list: list):
        """Order hard cargo first; reserve all soft cargo for tower caps.

        v11 only moved soft items to the end of each same-geometry group.  With
        lookahead=1 that still exposed soft cargo too early, so it became a new
        floor base instead of capping an existing rigid tower.
        """
        for it in item_list:
            if bool(it.get('is_soft', False)):
                self.soft_item_ids.add(int(it['index']))

        hard = [it for it in item_list if not bool(it.get('is_soft', False))]
        soft = [it for it in item_list if bool(it.get('is_soft', False))]

        def grouped_order(items):
            groups = {}
            for it in items:
                key = (round(float(it['length']), 4),
                       round(float(it['width']), 4),
                       round(float(it['height']), 4))
                groups.setdefault(key, []).append(it)
            keys = sorted(groups,
                          key=lambda k: (k[0] * k[1], k[2], k[0], k[1]),
                          reverse=True)
            order = []
            for key in keys:
                group = sorted(
                    groups[key],
                    key=lambda x: (
                        bool(x.get('is_prioritized', False)),
                        -int(x['index']),
                    ),
                    reverse=True,
                )
                order.extend(int(it['index']) for it in group)
            return order

        # Hard structure first.  Soft items come only after the rigid towers have
        # been established, so each can be assigned as a top cap.
        return grouped_order(hard) + grouped_order(soft)

    def policy(self, observation: dict):
        pool = observation.get('pool_list', [])
        containers = observation.get('container_list', [])
        if not pool or not containers:
            return self._fallback()

        # packed_items does not reliably preserve is_soft, so remember it by
        # original item index whenever the item is visible in the lookahead pool.
        for _it in pool:
            if bool(_it.get('is_soft', False)) and _it.get('index') is not None:
                self.soft_item_ids.add(int(_it['index']))

        # v8 invariant:
        #   1) finish the currently active back-row tower as high as safely possible;
        #   2) only then open the deepest remaining base position;
        #   3) move progressively toward the front (-Y).
        # This prevents a low floor carpet from making later back access difficult.

        # 1) Continue the active tower first.  Among lookahead items, use the
        # largest fitting footprint so the tower does not narrow prematurely.
        if self.last_target_xy is not None:
            tower_choices = []
            for cidx, c in enumerate(containers):
                packed = c.get('packed_items', [])
                for pidx, item in enumerate(pool):
                    pos = self._continue_last_tower(c, item, packed)
                    if pos is None:
                        continue
                    area = float(item['length']) * float(item['width'])
                    vol = area * float(item['height'])
                    tower_choices.append((-area, -vol, cidx, pidx, pos))
            if tower_choices:
                hard_choices = [q for q in tower_choices if not bool(pool[q[-2]].get('is_soft', False))]
                selectable = hard_choices if hard_choices else tower_choices
                selectable.sort(key=lambda x: x[:4])
                *_, cidx, pidx, pos = selectable[0]

                # A soft item is a tower cap: once placed, do not put anything
                # else on top of it.
                if bool(pool[pidx].get('is_soft', False)):
                    self.last_target_xy = None
                    mode = 'tower-stack-soft-cap'
                else:
                    self.last_target_xy = (float(pos[0]), float(pos[1]))
                    mode = 'tower-stack'
                return self._action(cidx, pidx, pos, mode=mode, item=pool[pidx])

            # No visible item can safely extend this tower: consider it complete
            # and open the next deepest base.
            self.last_target_xy = None

        # If the currently visible cargo is soft-only, it should cap an existing
        # rigid tower before we ever create a new soft floor/shelf base.
        visible_soft_only = bool(pool) and all(bool(it.get('is_soft', False)) for it in pool)
        if visible_soft_only:
            soft_stack_choices = []
            for cidx, c in enumerate(containers):
                packed = c.get('packed_items', [])
                for pidx, item in enumerate(pool):
                    pos = self._find_stack_position(c, item, packed)
                    if pos is None:
                        continue
                    # Highest usable position first, then deepest Y.
                    area = float(item['length']) * float(item['width'])
                    vol = area * float(item['height'])
                    soft_stack_choices.append((-float(pos[2]), -float(pos[1]),
                                               -area, -vol, cidx, pidx, pos))
            if soft_stack_choices:
                soft_stack_choices.sort(key=lambda x: x[:6])
                *_, cidx, pidx, pos = soft_stack_choices[0]
                self.last_target_xy = None
                return self._action(cidx, pidx, pos,
                                    mode='tower-soft-cap',
                                    item=pool[pidx])

        # 2) Open exactly one new deepest base.  Shelf bases are legitimate back
        # bases and, at the same Y, are preferred to floor-under-shelf because a
        # shelf tower can grow upward to the container ceiling.
        base_choices = []
        for cidx, c in enumerate(containers):
            packed = c.get('packed_items', [])
            has_main_shelf = bool(c.get('shelf', c.get('require_shelf', False)))
            for pidx, item in enumerate(pool):
                area = float(item['length']) * float(item['width'])
                vol = area * float(item['height'])

                # The narrow shelf created by cut_x exists in every container.
                sspos = self._find_small_shelf_position(c, item, packed)
                if sspos is not None:
                    base_choices.append((-float(sspos[1]), -float(sspos[2]), -area, -vol,
                                         cidx, pidx, sspos, 'tower-base-small-shelf'))

                if has_main_shelf:
                    spos = self._find_shelf_position(c, item, packed)
                    if spos is not None:
                        base_choices.append((-float(spos[1]), -float(spos[2]), -area, -vol,
                                             cidx, pidx, spos, 'tower-base-main-shelf'))

                fpos = self._find_floor_position(c, item, packed)
                if fpos is not None:
                    base_choices.append((-float(fpos[1]), -float(fpos[2]), -area, -vol,
                                         cidx, pidx, fpos, 'tower-base-floor'))

        if base_choices:
            hard_bases = [q for q in base_choices if not bool(pool[q[-3]].get('is_soft', False))]
            selectable = hard_bases if hard_bases else base_choices
            selectable.sort(key=lambda x: x[:6])
            *_, cidx, pidx, pos, mode = selectable[0]

            if bool(pool[pidx].get('is_soft', False)):
                # A soft base is allowed only when no rigid base candidate exists,
                # and it is immediately treated as complete (nothing stacked on it).
                self.last_target_xy = None
                mode += '-soft-cap'
            else:
                self.last_target_xy = (float(pos[0]), float(pos[1]))
            return self._action(cidx, pidx, pos, mode=mode, item=pool[pidx])

        # 3) Salvage: if no new base exists, use the deepest existing compatible
        # tower.  This is not the normal path, but avoids wasting a remaining gap.
        stack_choices = []
        for cidx, c in enumerate(containers):
            packed = c.get('packed_items', [])
            for pidx, item in enumerate(pool):
                pos = self._find_stack_position(c, item, packed)
                if pos is None:
                    continue
                area = float(item['length']) * float(item['width'])
                vol = area * float(item['height'])
                stack_choices.append((-float(pos[1]), float(pos[2]), -area, -vol, cidx, pidx, pos))
        if stack_choices:
            hard_choices = [q for q in stack_choices if not bool(pool[q[-2]].get('is_soft', False))]
            selectable = hard_choices if hard_choices else stack_choices
            selectable.sort(key=lambda x: x[:6])
            *_, cidx, pidx, pos = selectable[0]
            if bool(pool[pidx].get('is_soft', False)):
                self.last_target_xy = None
                mode = 'tower-salvage-soft-cap'
            else:
                self.last_target_xy = (float(pos[0]), float(pos[1]))
                mode = 'tower-salvage'
            return self._action(cidx, pidx, pos, mode=mode, item=pool[pidx])

        emergency_choices = []
        for cidx, c in enumerate(containers):
            packed = c.get('packed_items', [])
            for pidx, item in enumerate(pool):
                pos = self._find_emergency_stack(c, item, packed)
                if pos is not None:
                    emergency_choices.append((-float(pos[1]), float(pos[2]), cidx, pidx, pos))
        if emergency_choices:
            hard_choices = [q for q in emergency_choices if not bool(pool[q[-2]].get('is_soft', False))]
            selectable = hard_choices if hard_choices else emergency_choices
            selectable.sort(key=lambda x: x[:4])
            *_, cidx, pidx, pos = selectable[0]
            if bool(pool[pidx].get('is_soft', False)):
                self.last_target_xy = None
                mode = 'emergency-stack-soft-cap'
            else:
                self.last_target_xy = (float(pos[0]), float(pos[1]))
                mode = 'emergency-stack'
            return self._action(cidx, pidx, pos, mode=mode, item=pool[pidx])

        return self._fallback(containers[0], pool[0])

    def _is_soft_obj(self, obj):
        if obj is None:
            return False
        if bool(obj.get('is_soft', False)):
            return True
        idx = obj.get('index')
        return idx is not None and int(idx) in self.soft_item_ids

    def _action(self, cidx, pidx, pos, mode='unknown', item=None):
        item_id = None if item is None else item.get('index')
        print(f'[safe_grid_v13] mode={mode} item={item_id} pool_idx={pidx} target=({float(pos[0]):.3f},{float(pos[1]):.3f},{float(pos[2]):.3f})')
        return {
            'item_idx': int(pidx),
            'container_idx': int(cidx),
            'place_pos': np.asarray(pos, dtype=np.float32),
            'orientation': 0,
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
            print(f'[safe_grid_v13][continue-reject] no-support last=({lx:.3f},{ly:.3f}) item=({ix:.3f},{iy:.3f},{ih:.3f}) packed={len(packed)}')
            if drift_rejects:
                print('[safe_grid_v13][continue-reject] drift=' + ', '.join(
                    f"id={r[0]} pos=({r[1]:.3f},{r[2]:.3f}) d=({r[3]:.3f},{r[4]:.3f})" for r in drift_rejects[:6]))
            if footprint_rejects:
                print('[safe_grid_v13][continue-reject] footprint=' + ', '.join(
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
                    print(f'[safe_grid_v13][stack-offset] support={support.get("index")} '
                          f'base=({ox:.3f},{oy:.3f}) chosen=({cx:.3f},{cy:.3f})')
                return cand
            rejects.append((support.get('index'), last_reason, last_cand, limit))

        print(f'[safe_grid_v13][continue-reject] candidates={len(supports)} last=({lx:.3f},{ly:.3f})')
        for sid, reason, cand, limit in rejects[:6]:
            print(f'[safe_grid_v13][continue-reject] support={sid} reason={reason} cand=({cand[0]:.3f},{cand[1]:.3f},{cand[2]:.3f}) limit={limit:.3f}')
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
                    print(f'[safe_grid_v13][stack-offset] support={support.get("index")} '
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
        """Use the always-present narrow shelf at the cut_x side, orientation 0 only.

        The shelf footprint is approximately:
          X width = cut_x
          Y width = container width - 2*thickness
          Z top   = height/2 + buffer + thickness

        No rotation is attempted.  A tiny overhang (<=6 mm per side) is allowed
        so a 0.45 m item can use a 0.44 m shelf, but larger cargo is rejected.
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

        # orientation 0 only. Reject cargo wider than the narrow shelf except
        # for a few millimetres of symmetric overhang.
        if ix > cut_x + 2.0*self.small_shelf_overhang:
            return None

        # Left-side shelf interval inferred from the same cut_x geometry used by
        # the validator's insertion clamp.
        shelf_x0 = -L/2.0 + t
        shelf_x1 = shelf_x0 + cut_x
        x = (shelf_x0 + shelf_x1) / 2.0

        gap = self.safety_margin + self.extra_gap
        y_min = -W/2.0 + t + hy + gap
        y_max =  W/2.0 - t - hy - gap
        if y_min > y_max:
            return None

        shelf_top = H/2.0 + buffer + t
        drop_clearance = self.safety_margin + 0.006
        z = shelf_top + drop_clearance + hz
        if z + hz > self.max_stack_top:
            return None

        # Deepest free shelf position first.
        y = y_max
        while y >= y_min - 1e-9:
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

    def _final_clear(self, c, item, cand, packed, support=None):
        x, y, z = cand
        hx, hy, hz = float(item['length'])/2, float(item['width'])/2, float(item['height'])/2
        m = self.safety_margin + self.extra_gap
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
                return False
        return True

    def _path_clear(self, c, item, target, packed, extra_lift=0.0):
        tx, ty, tz = target
        L, W = float(c['length']), float(c['width'])
        t = float(c.get('thickness', 0.04))
        cut_x = float(c.get('cut_x', 0.0))
        hx, hy, hz = float(item['length'])/2, float(item['width'])/2, float(item['height'])/2
        x_min = -L/2 + t + cut_x + hx + max(self.start_margin, self.safety_margin + 0.003)
        x_max =  L/2 - t - hx - max(self.start_margin, self.safety_margin + 0.003)
        start_x = min(max(tx, x_min), x_max)
        start_y = -W/2

        # floor targets get effective_start_z=0; stacked targets normally +0.08.
        floor_bottom = tz - hz
        floor_z = t
        path_z = tz if 0 <= floor_bottom-floor_z <= 0.05 else min(tz + 0.08 + extra_lift, float(c['height']) - t - hz - self.start_margin)
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

    def _fallback(self, container=None, item=None):
        z = 0.1
        if container is not None and item is not None:
            z = float(container.get('thickness', 0.04)) + float(item['height'])/2 + 0.006
        print(f'[safe_grid_v13] mode=NO-SAFE-CANDIDATE target=(0.000,0.000,{z:.3f})')
        return {
            'item_idx': 0,
            'container_idx': 0,
            'place_pos': np.asarray([0.0, 0.0, z], dtype=np.float32),
            'orientation': 0,
        }
