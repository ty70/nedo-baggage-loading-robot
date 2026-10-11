import numpy as np
import sys

class Agent:
    def __init__(self, module_path: str):
        self.module_path = module_path
        self.num_containers = 0
        self.container_info = None  # will store dict of first container
        self.placed_items_local = []  # list of (x, y, z, dx, dy, dz) in local coordinates
        self.item_scores = {}  # item index -> score
        self.optimized_order = []  # list of item indices sorted by score
        self.debug = True

    def _log(self, msg):
        if self.debug:
            print(f"[DEBUG] {msg}", file=sys.stderr)
            sys.stderr.flush()

    def get_init_states(self, init_states: dict):
        container_list = init_states.get('container_list', [])
        self.num_containers = len(container_list)
        if container_list:
            self.container_info = container_list[0]  # assume single container
        self.placed_items_local = []
        self.item_scores = {}
        self.optimized_order = []
        self._log(f"get_init_states: {self.num_containers} containers")
        return True

    def _compute_item_score(self, item: dict) -> float:
        # Heavier items (larger mass) should be placed earlier (lower in container) -> give negative contribution
        mass = item.get('mass', 1.0)
        length = item.get('length', 0.1)
        width = item.get('width', 0.1)
        height = item.get('height', 0.1)
        footprint = length * width
        # Softer items are worse to place at bottom -> penalize soft (increase score)
        soft = item.get('soft', False)
        priority = item.get('priority', False)  # priority items we want to place higher (later) -> increase score
        # Score: lower is better (placed earlier)
        score = -(mass * 1000) - (footprint * 100) + (500 if soft else 0) - (500 if priority else 0)
        return score

    def optimize(self, item_list: list):
        # Score each item
        scored = []
        for item in item_list:
            score = self._compute_item_score(item)
            self.item_scores[item['index']] = score
            scored.append((score, item['index']))
        # Sort by score ascending (lower score first)
        scored.sort(key=lambda x: x[0])
        self.optimized_order = [idx for _, idx in scored]
        self._log(f"optimize: order length {len(self.optimized_order)}")
        # Return the order indices
        return self.optimized_order

    def _get_inner_bounds(self, container: dict):
        """Return inner bounds (x_min, x_max, y_min, y_max, z_min, z_max) in local coordinates."""
        length = container['length']
        width = container['width']
        height = container['height']
        thickness = container.get('thickness', 0.02)
        buffer = container.get('buffer', 0.0)
        cut_x = container.get('cut_x', 0.0)
        cut_y = container.get('cut_y', 0.0)
        # Inner dimensions after thickness, buffer, cut
        x_min = -length/2 + thickness + cut_x + buffer
        x_max = length/2 - thickness - buffer
        y_min = -width/2 + thickness + buffer
        y_max = width/2 - thickness - buffer
        z_min = thickness + buffer  # bottom inner
        z_max = height - thickness - buffer  # top inner
        return x_min, x_max, y_min, y_max, z_min, z_max

    def _is_valid_placement(self, container: dict, item: dict, pos_local: list, margin: float = 0.02) -> bool:
        """Check if item placed at pos_local (center) fits inside inner bounds and does not collide with placed items.
        pos_local: [x, y, z] center of item in local coordinates.
        margin: extra separation to avoid collision."""
        x_min, x_max, y_min, y_max, z_min, z_max = self._get_inner_bounds(container)
        dx = item.get('length', 0.1) / 2.0
        dy = item.get('width', 0.1) / 2.0
        dz = item.get('height', 0.1) / 2.0
        # Check bounds with margin
        if not (x_min + dx + margin <= pos_local[0] <= x_max - dx - margin):
            return False
        if not (y_min + dy + margin <= pos_local[1] <= y_max - dy - margin):
            return False
        if not (z_min + dz + margin <= pos_local[2] <= z_max - dz - margin):
            return False
        # Check against placed items
        for (px, py, pz, pdx, pdy, pdz) in self.placed_items_local:
            if (abs(pos_local[0] - px) < (dx + pdx + margin) or
                abs(pos_local[1] - py) < (dy + pdy + margin) or
                abs(pos_local[2] - pz) < (dz + pdz + margin)):
                return False
        return True

    def _find_valid_position(self, container: dict, item: dict, grid_step: float = 0.02) -> list:
        """Search for a valid position in local coordinates using a grid.
        Returns position [x, y, z] if found, else None."""
        x_min, x_max, y_min, y_max, z_min, z_max = self._get_inner_bounds(container)
        dx = item.get('length', 0.1) / 2.0
        dy = item.get('width', 0.1) / 2.0
        dz = item.get('height', 0.1) / 2.0
        # Define search ranges with margin
        min_x = x_min + dx + 0.02
        max_x = x_max - dx - 0.02
        min_y = y_min + dy + 0.02
        max_y = y_max - dy - 0.02
        min_z = z_min + dz + 0.02
        max_z = z_max - dz - 0.02
        if min_x > max_x or min_y > max_y or min_z > max_z:
            return None
        # Start from lower layers (smaller z) to fill bottom first
        z = min_z
        while z <= max_z:
            y = min_y
            while y <= max_y:
                x = min_x
                while x <= max_x:
                    pos = [x, y, z]
                    if self._is_valid_placement(container, item, pos):
                        return pos
                    x += grid_step
                y += grid_step
            z += grid_step
        return None

    def policy(self, observation: dict):
        pool_list = observation.get('pool_list', [])
        container_list = observation.get('container_list', [])
        if not pool_list or not container_list:
            # fallback
            container_idx = 0
            return {
                'item_idx': 0,
                'container_idx': container_idx,
                'place_pos': np.array([0.0, 0.0, 0.4], dtype=np.float32),
                'orientation': 0
            }
        # Use first container
        container = container_list[0]
        container_idx = container.get('index', 0)
        offset_x = container.get('offset_x', 0.0)
        offset_y = container.get('offset_y', 0.0)
        offset_z = container.get('offset_z', 0.0)
        # If we haven't optimized yet (should have been called by optimize), do a simple sort by volume
        if not self.optimized_order:
            self._log("optimize not called, computing order on the fly")
            scored = []
            for item in pool_list:
                score = self._compute_item_score(item)
                scored.append((score, item['index']))
            scored.sort(key=lambda x: x[0])
            self.optimized_order = [idx for _, idx in scored]
        # Try to place items in optimized order
        for item_idx in self.optimized_order:
            # Find the item dict in pool_list
            item = None
            for it in pool_list:
                if it['index'] == item_idx:
                    item = it
                    break
            if item is None:
                continue
            # Skip if already placed? We don't have a placed flag; we rely on the simulator's belongs_to.
            # For simplicity, we assume each item appears only once per call.
            # Try to find a valid position
            pos_local = self._find_valid_position(container, item, grid_step=0.02)
            if pos_local is not None:
                # Convert to global coordinates
                pos_global = [pos_local[0] + offset_x, pos_local[1] + offset_y, pos_local[2] + offset_z]
                # Record placed item (local) for future collision checks
                dx = item.get('length', 0.1) / 2.0
                dy = item.get('width', 0.1) / 2.0
                dz = item.get('height', 0.1) / 2.0
                self.placed_items_local.append((pos_local[0], pos_local[1], pos_local[2], dx*2, dy*2, dz*2))
                self._log(f"Placed item {item['index']} at local {pos_local}, global {pos_global}")
                return {
                    'item_idx': item_idx,
                    'container_idx': container_idx,
                    'place_pos': np.array(pos_global, dtype=np.float32),
                    'orientation': 0
                }
        # If no item could be placed, return a default action (try to place first item at a safe default)
        # We'll attempt to place the first item at the center bottom of action space (local) and hope it's valid
        if pool_list:
            item = pool_list[0]
            # Try a few default positions
            defaults = [
                [0.0, 0.0, 0.4],
                [0.0, 0.0, 0.6],
                [0.1, 0.1, 0.5],
                [-0.1, -0.1, 0.5]
            ]
            for d in defaults:
                if self._is_valid_placement(container, item, d):
                    pos_global = [d[0] + offset_x, d[1] + offset_y, d[2] + offset_z]
                    dx = item.get('length', 0.1) / 2.0
                    dy = item.get('width', 0.1) / 2.0
                    dz = item.get('height', 0.1) / 2.0
                    self.placed_items_local.append((d[0], d[1], d[2], dx*2, dy*2, dz*2))
                    self._log(f"Default placement item {item['index']} at local {d}, global {pos_global}")
                    return {
                        'item_idx': item['index'],
                        'container_idx': container_idx,
                        'place_pos': np.array(pos_global, dtype=np.float32),
                        'orientation': 0
                    }
        # Last resort: return center action (may be invalid but we have to return something)
        self._log("No valid position found, returning center action")
        return {
            'item_idx': pool_list[0]['index'] if pool_list else 0,
            'container_idx': container_idx,
            'place_pos': np.array([0.0, 0.0, 0.4], dtype=np.float32),
            'orientation': 0
        }