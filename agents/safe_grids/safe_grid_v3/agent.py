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
        """Group identical footprints/heights so each tower can be completed before moving forward."""
        groups = {}
        for it in item_list:
            key = (round(float(it['length']), 4), round(float(it['width']), 4), round(float(it['height']), 4))
            groups.setdefault(key, []).append(it)

        # Larger footprints first; for equal footprint, taller first.
        keys = sorted(groups, key=lambda k: (k[0] * k[1], k[2]), reverse=True)
        order = []
        for key in keys:
            # prioritized first inside an otherwise identical geometry group
            vals = sorted(groups[key], key=lambda x: (bool(x.get('is_prioritized', False)), x['index']), reverse=True)
            order.extend(int(x['index']) for x in vals)
        return order

    def policy(self, observation: dict):
        pool = observation.get('pool_list', [])
        containers = observation.get('container_list', [])
        if not pool or not containers:
            return self._fallback()

        # Score all visible items by whether they can continue an existing tower first,
        # then by volume. This is useful for online lookahead case.
        choices = []
        for cidx, c in enumerate(containers):
            packed = c.get('packed_items', [])
            for pidx, item in enumerate(pool):
                stack_pos = self._find_stack_position(c, item, packed)
                if stack_pos is not None:
                    vol = float(item['length']) * float(item['width']) * float(item['height'])
                    choices.append((0, -vol, cidx, pidx, stack_pos))

        if choices:
            choices.sort(key=lambda x: x[:4])
            _, _, cidx, pidx, pos = choices[0]
            self.last_target_xy = (float(pos[0]), float(pos[1]))
            return self._action(cidx, pidx, pos, mode='stack', item=pool[pidx])

        # In shelf-equipped containers, use the upper shelf before filling/tall-stacking
        # the floor region. Target a point slightly above the shelf and let settling
        # drop the item a few centimeters onto it.
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
                shelf_choices.append((-area, -vol, -float(pos[1]), -float(pos[0]), cidx, pidx, pos))
        if shelf_choices:
            shelf_choices.sort(key=lambda x: x[:6])
            *_, cidx, pidx, pos = shelf_choices[0]
            self.last_target_xy = (float(pos[0]), float(pos[1]))
            return self._action(cidx, pidx, pos, mode='shelf', item=pool[pidx])

        # No safe stack/shelf continuation: create a new floor tower.
        floor_choices = []
        for cidx, c in enumerate(containers):
            packed = c.get('packed_items', [])
            for pidx, item in enumerate(pool):
                pos = self._find_floor_position(c, item, packed)
                if pos is None:
                    continue
                # Prefer deeper Y, then larger volume, then right X.
                vol = float(item['length']) * float(item['width']) * float(item['height'])
                area = float(item['length']) * float(item['width'])
                floor_choices.append((-area, -vol, -float(pos[1]), -float(pos[0]), cidx, pidx, pos))

        if floor_choices:
            floor_choices.sort(key=lambda x: x[:6])
            *_, cidx, pidx, pos = floor_choices[0]
            self.last_target_xy = (float(pos[0]), float(pos[1]))
            return self._action(cidx, pidx, pos, mode='floor', item=pool[pidx])

        # Last chance: continue from the physically highest support instead of
        # returning an unsafe center fallback. This is especially important
        # when PyBullet settling shifts a tower by a few centimeters.
        emergency = self._find_emergency_stack(containers[0], pool[0], containers[0].get('packed_items', []))
        if emergency is not None:
            return self._action(0, 0, emergency, mode='emergency-stack', item=pool[0])
        return self._fallback(containers[0], pool[0])

    def _action(self, cidx, pidx, pos, mode='unknown', item=None):
        item_id = None if item is None else item.get('index')
        print(f'[safe_grid_v2] mode={mode} item={item_id} pool_idx={pidx} target=({float(pos[0]):.3f},{float(pos[1]):.3f},{float(pos[2]):.3f})')
        return {
            'item_idx': int(pidx),
            'container_idx': int(cidx),
            'place_pos': np.asarray(pos, dtype=np.float32),
            'orientation': 0,
        }

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

            z = top + ih / 2.0 + 0.001
            stack_top_limit = self.max_stack_top
            # In shelf containers the main shelf occupies the back half (local y >= thickness)
            # around height/2. Keep floor towers below its underside; do not try to use the shelf yet.
            if bool(c.get('shelf', c.get('require_shelf', False))) and oy >= float(c.get('thickness', 0.05)):
                stack_top_limit = min(stack_top_limit, float(c['height'])/2.0 - self.safety_margin - 0.005)
            if z + ih / 2.0 > stack_top_limit:
                continue
            cand = (ox, oy, z)
            if not self._inside_simple(c, item, cand):
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
            # 6 mm initial gap: enough to avoid numerical overlap, then settle.
            z = top + ih / 2.0 + 0.006
            if z + ih / 2.0 > self.max_stack_top:
                continue
            cand = (ox, oy, z)
            if not self._inside_simple(c, item, cand):
                continue
            # Ignore the chosen support in final overlap, but still reject other boxes.
            if not self._final_clear(c, item, cand, packed, support=support):
                continue
            # Use a more conservative path check with extra vertical clearance.
            if not self._path_clear(c, item, cand, packed, extra_lift=0.04):
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
        print(f'[safe_grid_v2] mode=FALLBACK-UNSAFE target=(0.000,0.000,{z:.3f})')
        return {
            'item_idx': 0,
            'container_idx': 0,
            'place_pos': np.asarray([0.0, 0.0, z], dtype=np.float32),
            'orientation': 0,
        }
