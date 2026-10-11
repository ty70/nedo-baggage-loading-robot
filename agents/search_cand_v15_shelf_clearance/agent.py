"""v14の探索を維持し、小棚から15 mmちょうどの実行座標に正の余裕を加える。"""
import os
import numpy as np

from agents.search_cand_v14_order_2716_diagnostic.agent import Agent as V14Agent


class Agent(V14Agent):
    # 公式マージンの緩和ではなく、経路を棚から1 µm遠ざける。
    clearance_padding = 1e-6

    def _clamp_to_safe_margin(self, c, item, pos, orientation):
        x, y, z = super()._clamp_to_safe_margin(c, item, pos, orientation)
        L, W, H = (float(c[k]) for k in ('length', 'width', 'height'))
        t, cut = float(c.get('thickness', .04)), float(c.get('cut_x', 0))
        buf = float(c.get('buffer', 0))
        hx, hy, hz = (float(item[k]) / 2 for k in ('length', 'width', 'height'))
        right = -L / 2 + t + cut
        gap = float(x) - hx - right
        pad = self.clearance_padding
        # 実際の重なりや大きな不足は補正せず、境界付近だけを扱う。
        if cut <= 0 or not self.safety_margin - pad <= gap <= self.safety_margin + pad:
            return x, y, z
        lift = float(os.environ.get('EP_SIM_START_Z', '.08'))
        if any(0 <= z - hz - s <= .05 for s in (t, H / 2 + t + buf)):
            lift = 0.0
        if lift > 0:
            ceiling_margin = float(os.environ.get('EP_SIM_CEILING_MARGIN', '.018'))
            for ceiling in (H / 2 + buf, H + buf - t):
                clearance = ceiling - (z + hz)
                if 0 <= clearance < lift + ceiling_margin:
                    lift = max(0.0, clearance - ceiling_margin - .0005)
                    break
        path_z = min(H + buf - t - hz - self.start_margin, z + lift)
        # Y区間と小棚のY/Z投影が離れていれば補正不要。
        if (path_z + hz <= H / 2 + buf - self.safety_margin or
                path_z - hz >= H / 2 + t + buf + self.safety_margin or
                max(-W / 2, y) + hy <= -W / 2 + t - self.safety_margin):
            return x, y, z
        desired = right + hx + self.safety_margin + pad
        corrected = np.float32(desired)
        # float32化で余裕を失わないよう、外側へ丸める。
        if float(corrected) < desired:
            corrected = np.nextafter(corrected, np.float32(np.inf))
        candidate = (float(corrected), y, z)
        if not self._inside_container(c, item, candidate):
            return x, y, z
        return candidate

    def policy(self, observation):
        action = super().policy(observation)
        pool = observation.get('pool_list', [])
        if 0 <= action['item_idx'] < len(pool):
            print(f'[V15-ISSUED] item={pool[action["item_idx"]]["index"]} '
                  f'position={tuple(map(float, action["place_pos"]))}', flush=True)
        return action
