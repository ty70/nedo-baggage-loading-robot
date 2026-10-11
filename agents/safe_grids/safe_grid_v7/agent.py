import math
import numpy as np


class Agent:
    """Simple robust column-packing baseline.

    - orientation 0 only
    - deep (+Y) to front (-Y)
    - right (+X) to left (-X)
    - stack vertically on an existing footprint when safely supported
    - otherwise create a new floor tower
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

    def get_init_states(self, init_states: dict):
        self.optimize_enabled = bool(init_states.get('optimize', False))
        self.lookahead_k = int(init_states.get('lookahead_k', 1))
        self.container_init = init_states.get('container_list', [])
        self.last_target_xy = None
        if self.container_init:
            h = float(self.container_init[0].get('height', 1.6))
            t = float(self.container_init[0].get('thickness', 0.04))
            self.max_stack_top = h - t - 0.06
        return True

    def optimize(self, item_list: list):
        """Expose mixed footprints instead of exhausting one geometry group.

        With lookahead=1, grouping every identical item consecutively forces the
        policy to build a tall tower even when a different-size item could still
        fill a deeper floor gap.  Round-robin the geometry groups so deeper floor
        holes can be consumed before vertical growth.
        """
        groups = {}
        for it in item_list:
            key = (round(float(it['length']), 4), round(float(it['width']), 4), round(float(it['height']), 4))
            groups.setdefault(key, []).append(it)

        # Larger footprints are offered first in each round, but only one item per
        # geometry group is emitted before moving to the next group.
        keys = sorted(groups, key=lambda k: (k[0] * k[1], k[2]), reverse=True)
        queues = {}
        for key in keys:
            queues[key] = sorted(
                groups[key],
                key=lambda x: (bool(x.get('is_prioritized', False)), x['index']),
                reverse=True,
            )

        order = []
        while any(queues[k] for k in keys):
            for key in keys:
                if queues[key]:
                    order.append(int(queues[key].pop(0)['index']))
        return order

    def policy(self, observation: dict):
        pool = observation.get('pool_list', [])
        containers = observation.get('container_list', [])
        if not pool or not containers:
            return self._fallback()

        # 1) Use the main shelf first when available.  It is already in the back
        # half and does not create a front-side vertical wall on the floor path.
        shelf_choices = []
        for cidx, c in enumerate(containers):
            if not bool(c.get('shelf', c.get('require_shelf', False))):
                continue
            packed = c.get('packed_items', [])
            for pidx, item in enumerate(pool):
                pos = self._find_shelf_position(c, item, packed)
                if pos is None:
                    continue
                vol = float(item['length']) * float(item['width']) * float(item['height'])
                area = float(item['length']) * float(item['width'])
                # Back/deep (+Y) is the primary objective; size is only a tie-breaker.
                shelf_choices.append((-float(pos[1]), -area, -vol, -float(pos[0]), cidx, pidx, pos))
        if shelf_choices:
            shelf_choices.sort(key=lambda x: x[:6])
            *_, cidx, pidx, pos = shelf_choices[0]
            self.last_target_xy = (float(pos[0]), float(pos[1]))
            return self._action(cidx, pidx, pos, mode='shelf', item=pool[pidx])

        # 2) Fill every reachable floor gap from back (+Y) to front (-Y) BEFORE
        # increasing tower height.  v6 did the reverse and could build a tall
        # front tower while a smaller visible item still fit farther back.
        floor_choices = []
        for cidx, c in enumerate(containers):
            packed = c.get('packed_items', [])
            for pidx, item in enumerate(pool):
                pos = self._find_floor_position(c, item, packed)
                if pos is None:
                    continue
                vol = float(item['length']) * float(item['width']) * float(item['height'])
                area = float(item['length']) * float(item['width'])
                floor_choices.append((-float(pos[1]), -area, -vol, -float(pos[0]), cidx, pidx, pos))

        if floor_choices:
            floor_choices.sort(key=lambda x: x[:6])
            *_, cidx, pidx, pos = floor_choices[0]
            self.last_target_xy = (float(pos[0]), float(pos[1]))
            return self._action(cidx, pidx, pos, mode='floor-backfill', item=pool[pidx])

        # 3) Only after the floor has no safe candidate do we grow vertically.
        # Prefer the deepest available tower and, at equal depth, the lower target.
        forced_choices = []
        if self.last_target_xy is not None:
            for cidx, c in enumerate(containers):
                packed = c.get('packed_items', [])
                for pidx, item in enumerate(pool):
                    pos = self._continue_last_tower(c, item, packed)
                    if pos is None:
                        continue
                    vol = float(item['length']) * float(item['width']) * float(item['height'])
                    area = float(item['length']) * float(item['width'])
                    forced_choices.append((-float(pos[1]), float(pos[2]), -area, -vol, cidx, pidx, pos))
        if forced_choices:
            forced_choices.sort(key=lambda x: x[:6])
            *_, cidx, pidx, pos = forced_choices[0]
            self.last_target_xy = (float(pos[0]), float(pos[1]))
            return self._action(cidx, pidx, pos, mode='continue-stack', item=pool[pidx])

        stack_choices = []
        for cidx, c in enumerate(containers):
            packed = c.get('packed_items', [])
            for pidx, item in enumerate(pool):
                pos = self._find_stack_position(c, item, packed)
                if pos is None:
                    continue
                vol = float(item['length']) * float(item['width']) * float(item['height'])
                area = float(item['length']) * float(item['width'])
                stack_choices.append((-float(pos[1]), float(pos[2]), -area, -vol, cidx, pidx, pos))
        if stack_choices:
            stack_choices.sort(key=lambda x: x[:6])
            *_, cidx, pidx, pos = stack_choices[0]
            self.last_target_xy = (float(pos[0]), float(pos[1]))
            return self._action(cidx, pidx, pos, mode='stack-after-floor', item=pool[pidx])

        emergency = self._find_emergency_stack(containers[0], pool[0], containers[0].get('packed_items', []))
        if emergency is not None:
            return self._action(0, 0, emergency, mode='emergency-stack', item=pool[0])
        return self._fallback(containers[0], pool[0])

    def _action(self, cidx, pidx, pos, mode='unknown', item=None):
        item_id = None if item is None else item.get('index')
        print(f'[safe_grid_v7] mode={mode} item={item_id} pool_idx={pidx} target=({float(pos[0]):.3f},{float(pos[1]):.3f},{float(pos[2]):.3f})')
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
            ox, oy, oz = self._local_pos(c, pos)
            dx, dy = abs(ox - lx), abs(oy - ly)
            if dx > 0.10 or dy > 0.10:
                drift_rejects.append((o.get('index'), ox, oy, dx, dy))
                continue
            sx, sy = float(o['length']), float(o['width'])
            if ix > sx + 0.005 or iy > sy + 0.005:
                footprint_rejects.append((o.get('index'), sx, sy))
                continue
            top = oz + float(o['height']) / 2.0
            supports.append((top, -dx-dy, ox, oy, o))

        if not supports:
            print(f'[safe_grid_v7][continue-reject] no-support last=({lx:.3f},{ly:.3f}) item=({ix:.3f},{iy:.3f},{ih:.3f}) packed={len(packed)}')
            if drift_rejects:
                print('[safe_grid_v7][continue-reject] drift=' + ', '.join(
                    f"id={r[0]} pos=({r[1]:.3f},{r[2]:.3f}) d=({r[3]:.3f},{r[4]:.3f})" for r in drift_rejects[:6]))
            if footprint_rejects:
                print('[safe_grid_v7][continue-reject] footprint=' + ', '.join(
                    f"id={r[0]} support=({r[1]:.3f},{r[2]:.3f})" for r in footprint_rejects[:6]))
            return None

        supports.sort(reverse=True, key=lambda q: (q[0], q[1]))
        rejects = []
        for top, _, ox, oy, support in supports:
            z = top + ih / 2.0 + self.stack_drop_clearance
            cx, cy = self._clamp_stack_xy(c, item, ox, oy, support)
            if cx is None:
                rejects.append((support.get('index'), 'support-overhang', (ox, oy, z), self.max_stack_top))
                continue
            limit = self.max_stack_top
            if bool(c.get('shelf', c.get('require_shelf', False))):
                shelf_front = float(c.get('thickness', 0.05))
                support_half_y = float(support['width']) / 2.0
                if oy + support_half_y + self.safety_margin >= shelf_front:
                    limit = min(limit, float(c['height'])/2.0 - self.safety_margin - 0.005)
            cand = (cx, cy, z)
            if z + ih/2.0 > limit:
                rejects.append((support.get('index'), 'height', cand, limit))
                continue
            if not self._inside_simple(c, item, cand):
                rejects.append((support.get('index'), 'inside', cand, limit))
                continue
            if not self._shelf_transport_clear(c, item, cand):
                rejects.append((support.get('index'), 'shelf-path', cand, limit))
                continue
            if not self._final_clear(c, item, cand, packed, support=support):
                rejects.append((support.get('index'), 'final-clear', cand, limit))
                continue
            if not self._path_clear(c, item, cand, packed):
                rejects.append((support.get('index'), 'path-clear', cand, limit))
                continue
            return cand

        print(f'[safe_grid_v7][continue-reject] candidates={len(supports)} last=({lx:.3f},{ly:.3f})')
        for sid, reason, cand, limit in rejects[:6]:
            print(f'[safe_grid_v7][continue-reject] support={sid} reason={reason} cand=({cand[0]:.3f},{cand[1]:.3f},{cand[2]:.3f}) limit={limit:.3f}')
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
            cx, cy = self._clamp_stack_xy(c, item, ox, oy, support)
            if cx is None:
                continue
            stack_top_limit = self.max_stack_top
            # In shelf containers the main shelf occupies the back half (local y >= thickness)
            # around height/2. Keep floor towers below its underside; do not try to use the shelf yet.
            if bool(c.get('shelf', c.get('require_shelf', False))):
                shelf_front = float(c.get('thickness', 0.05))
                support_half_y = float(support['width']) / 2.0
                if oy + support_half_y + self.safety_margin >= shelf_front:
                    stack_top_limit = min(stack_top_limit, float(c['height'])/2.0 - self.safety_margin - 0.005)
            if z + ih / 2.0 > stack_top_limit:
                continue
            cand = (cx, cy, z)
            if not self._inside_simple(c, item, cand):
                continue
            if not self._shelf_transport_clear(c, item, cand):
                continue
            if not self._final_clear(c, item, cand, packed, support=support):
                continue
            if not self._path_clear(c, item, cand, packed):
                continue
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

        # Main shelf covers local y roughly [t, W/2-t].
        x_min = -L/2 + t + cut_x + hx + max(self.start_margin, self.safety_margin + 0.003)
        x_max =  L/2 - t - hx - max(self.start_margin, self.safety_margin + 0.003)
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

        # Candidate positions: deep->front, right->left. Use exact back/right boundary
        # plus scan grid. This retains packing density without a fixed occupancy grid.
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
        print(f'[safe_grid_v7] mode=NO-SAFE-CANDIDATE target=(0.000,0.000,{z:.3f})')
        return {
            'item_idx': 0,
            'container_idx': 0,
            'place_pos': np.asarray([0.0, 0.0, z], dtype=np.float32),
            'orientation': 0,
        }
