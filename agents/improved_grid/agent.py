import numpy as np
import sys
import json
import os


class Agent:
    def __init__(self, module_path: str):
        self.module_path = module_path
        self.occupancy = {}  # container_idx -> 2D list (ny, nx) bool
        self.grid_info = {}  # container_idx -> dict with cell dimensions and origin
        self.placed_items = {}  # container_idx -> list of placed items for collision checking
        self.debug = True
        self.validator_config = {}  # validator parameters for current container
        self.container_dims = {}  # container_idx -> dict of container dimensions

    def _log(self, msg):
        if self.debug:
            print(f"[DEBUG] {msg}", file=sys.stderr)
            sys.stderr.flush()

    def _read_validator_config(self):
        """Read validator config from the sample_config.json file."""
        config_path = '/home/ty70/simulator/configs/sample_config.json'
        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
            # Find the validator config that matches the current container dimensions
            # We'll match by container index? Actually the config has keys "000" and "001"
            # We don't know which one we are in yet. We'll store all and later match by dimensions.
            self.full_config = config
        except Exception as e:
            self._log(f"Failed to read config: {e}")
            self.full_config = {}

    def get_init_states(self, init_states: dict):
        self._log(f"get_init_states called")
        self._read_validator_config()
        self.num_containers = len(init_states.get('container_list', []))
        self._log(f"get_init_states: {self.num_containers} containers")
        # Store container dimensions for each container index
        for idx, cont in enumerate(init_states.get('container_list', [])):
            self.container_dims[idx] = {
                'length': cont['length'],
                'width': cont['width'],
                'height': cont['height'],
                'thickness': cont.get('thickness', 0.02),
                'offset_x': cont.get('offset_x', 0.0),
                'offset_y': cont.get('offset_y', 0.0),
                'offset_z': cont.get('offset_z', 0.0),
            }
            # Find matching validator config from full_config by matching dimensions
            if hasattr(self, 'full_config'):
                for task_key, task_config in self.full_config.items():
                    if isinstance(task_config, dict) and 'containers' in task_config:
                        cont_list = task_config['containers'].get('container_list', [])
                        if len(cont_list) > 0:
                            c = cont_list[0]
                            # Check if dimensions match (allow small tolerance for floating point)
                            if (abs(c['length'] - cont['length']) < 0.001 and
                                abs(c['width'] - cont['width']) < 0.001 and
                                abs(c['height'] - cont['height']) < 0.001):
                                self.validator_config[idx] = task_config.get('validator', {})
                                self._log(f"Matched validator config for container {idx} from task {task_key}")
                                break
            # If not found, use defaults
            if idx not in self.validator_config:
                self.validator_config[idx] = {
                    'inclusion_margin': -0.005,
                    'start_z': 0.08,
                    'safety_margin': 0.015,
                    'ceiling_margin': 0.018,
                    'displacement_threshold': 0.3,
                    'angle_displacement_threshold': 45,
                    'settle_wait_step': 300
                }
                self._log(f"Using default validator config for container {idx}")
        return True

    def optimize(self, item_list: list):
        # Simple volume-based ordering (larger volume first)
        def volume(item):
            return item['length'] * item['width'] * item['height']
        sorted_items = sorted(item_list, key=volume, reverse=True)
        return [item['index'] for item in sorted_items]

    def policy(self, observation: dict):
        self._log("policy entered")
        depth_maps = observation.get('depth_map')
        pool_list = observation.get('pool_list', [])
        container_list = observation.get('container_list', [])

        if not pool_list or not container_list:
            self._log("empty pool or container")
            return {
                'item_idx': 0,
                'container_idx': 0,
                'place_pos': np.array([0.0, 0.0, 0.0], dtype=np.float32),
                'orientation': 0
            }

        # Use first container (assuming single container setup)
        container_idx = 0
        container = container_list[container_idx]

        # Get container properties
        length = container['length']
        width = container['width']
        height = container['height']
        thickness = container.get('thickness', 0.02)

        # Get validator config for this container
        validator = self.validator_config.get(container_idx, {
            'safety_margin': 0.015,
            'ceiling_margin': 0.018,
            'start_z': 0.08,
            'inclusion_margin': -0.005
        })
        safety_margin = validator.get('safety_margin', 0.015)
        ceiling_margin = validator.get('ceiling_margin', 0.018)
        start_z = validator.get('start_z', 0.08)
        inclusion_margin = validator.get('inclusion_margin', -0.005)

        # Inner dimensions (subtract thickness from all sides)
        inner_length = length - 2 * thickness
        inner_width = width - 2 * thickness
        inner_height = height - 2 * thickness

        self._log(f"Container inner dimensions: {inner_length:.3f}x{inner_width:.3f}x{inner_height:.3f}")
        self._log(f"Pool list length: {len(pool_list)}")
        self._log(f"Validator: safety_margin={safety_margin}, ceiling_margin={ceiling_margin}, start_z={start_z}")

        # Initialize grid for this container if not done
        if container_idx not in self.grid_info:
            self._initialize_grid(container_idx, pool_list, inner_length, inner_width, inner_height, safety_margin)

        info = self.grid_info[container_idx]
        occ = self.occupancy[container_idx]

        # Initialize placed items tracking for collision checking
        if container_idx not in self.placed_items:
            self.placed_items[container_idx] = []

        # Create a mapping from item index to pool_list index for quick lookup
        item_index_to_pool_index = {item['index']: idx for idx, item in enumerate(pool_list)}

        # Try to place items in optimized order
        optimized_order = self.optimize(pool_list)
        self._log(f"Optimized order: {optimized_order}")

        for item_idx in optimized_order:
            # Find the item in pool_list by its index
            pool_idx = None
            for idx, item in enumerate(pool_list):
                if item['index'] == item_idx:
                    pool_idx = idx
                    item = item
                    break
            if pool_idx is None:
                self._log(f"Item {item_idx} not found in pool_list")
                continue

            self._log(f"Trying to place item {item_idx} (pool index {pool_idx}): {item}")

            # Try to find a valid position
            pos = self._find_valid_position(container, item, info, occ, container_idx, safety_margin, ceiling_margin)
            if pos is not None:
                place_x, place_y, place_z = pos

                # Mark occupancy (simple point-based for now)
                gx, gy = self._world_to_grid(place_x, place_y, info)
                if 0 <= gx < info['nx'] and 0 <= gy < info['ny']:
                    occ[gy][gx] = True

                # Record placed item for collision checking
                dx = item['length'] / 2.0
                dy = item['width'] / 2.0
                dz = item['height'] / 2.0
                self.placed_items[container_idx].append((place_x, place_y, place_z, dx*2, dy*2, dz*2))

                self._log(f"Placed item {item['index']} at ({place_x:.3f}, {place_y:.3f}, {place_z:.3f})")

                # Return the pool_list index, not the item's permanent index
                return {
                    'item_idx': pool_idx,
                    'container_idx': container_idx,
                    'place_pos': np.array([place_x, place_y, place_z], dtype=np.float32),
                    'orientation': 0
                }
            else:
                self._log(f"Could not find valid position for item {item_idx}")

        # If no item could be placed, return fallback action
        self._log("No valid placement found, using fallback")
        if pool_list:
            item = pool_list[0]
            # Try a few default positions
            defaults = [
                [0.0, 0.0, start_z],
                [0.0, 0.0, start_z + 0.2],
                [0.1, 0.1, start_z + 0.1],
                [-0.1, -0.1, start_z + 0.1]
            ]
            for d in defaults:
                if self._is_valid_placement_simple(container, item, d, self.placed_items.get(container_idx, []), safety_margin, ceiling_margin):
                    self._log(f"Fallback placement at {d}")
                    return {
                        'item_idx': 0,  # First item in pool_list
                        'container_idx': container_idx,
                        'place_pos': np.array([d[0], d[1], d[2]], dtype=np.float32),
                        'orientation': 0
                    }

        # Last resort
        self._log("Last resort: returning center action")
        return {
            'item_idx': 0 if pool_list else 0,  # First item in pool_list or 0 if empty
            'container_idx': container_idx,
            'place_pos': np.array([0.0, 0.0, start_z], dtype=np.float32),
            'orientation': 0
        }

    def _initialize_grid(self, container_idx, pool_list, inner_length, inner_width, inner_height, safety_margin):
        self._log(f"Initializing grid for container {container_idx}")

        # Use average item size for grid cells
        if pool_list:
            avg_length = np.mean([item['length'] for item in pool_list])
            avg_width = np.mean([item['width'] for item in pool_list])
        else:
            avg_length = avg_width = 0.1

        # Add margin to prevent overlap (item size + safety margin)
        cell_x = avg_length + safety_margin
        cell_y = avg_width + safety_margin

        # Calculate grid dimensions
        nx = max(1, int((inner_length - safety_margin) // cell_x))  # leave margin at walls
        ny = max(1, int((inner_width - safety_margin) // cell_y))

        # Adjust cell size to fit exactly (with margins)
        actual_cell_x = (inner_length - 2 * safety_margin) / nx if nx > 0 else inner_length
        actual_cell_y = (inner_width - 2 * safety_margin) / ny if ny > 0 else inner_width

        # Origin at bottom-left-front (minimum corner) plus safety margin
        origin_x = -inner_length / 2.0 + safety_margin
        origin_y = -inner_width / 2.0 + safety_margin
        origin_z = -inner_height / 2.0  # Will be adjusted in placement (we'll use start_z later)

        self.grid_info[container_idx] = {
            'nx': nx,
            'ny': ny,
            'cell_x': actual_cell_x,
            'cell_y': actual_cell_y,
            'origin_x': origin_x,
            'origin_y': origin_y,
            'origin_z': origin_z
        }

        # Initialize occupancy map
        self.occupancy[container_idx] = [[False for _ in range(nx)] for _ in range(ny)]

        self._log(f"Grid initialized: {nx}x{ny}, cell size {actual_cell_x:.3f}x{actual_cell_y:.3f}")
        self._log(f"Origin: ({origin_x:.3f}, {origin_y:.3f}, {origin_z:.3f})")

    def _world_to_grid(self, x, y, info):
        """Convert world coordinates to grid indices"""
        gx = int((x - info['origin_x']) / info['cell_x'])
        gy = int((y - info['origin_y']) / info['cell_y'])
        return gx, gy

    def _find_valid_position(self, container, item, info, occ, container_idx, safety_margin, ceiling_margin):
        """Find a valid position for the item using grid-based search"""
        dx = item['length'] / 2.0
        dy = item['width'] / 2.0
        dz = item['height'] / 2.0

        # Get validator config for this container
        validator = self.validator_config.get(container_idx, {
            'safety_margin': 0.015,
            'ceiling_margin': 0.018,
            'start_z': 0.08
        })
        safety_margin = validator.get('safety_margin', 0.015)
        ceiling_margin = validator.get('ceiling_margin', 0.018)
        start_z = validator.get('start_z', 0.08)

        # Search from bottom to top, back to front, left to right
        # We'll limit Z search to a reasonable range
        max_z_layers = 20  # limit to prevent infinite loop
        for iz in range(max_z_layers):  # Z layers
            iy_start = info['ny'] - 1  # Start from back (high Y)
            iy_end = -1
            iy_step = -1

            for iy in range(iy_start, iy_end, iy_step):
                for ix in range(info['nx']):  # Left to right
                    if not occ[iy][ix]:
                        # Calculate world position at cell center
                        wx = info['origin_x'] + (ix + 0.5) * info['cell_x']
                        wy = info['origin_y'] + (iy + 0.5) * info['cell_y']
                        # Z position: start from start_z and stack up
                        wz = start_z + dz + iz * 0.1  # Small increment for stacking

                        # Check if this position is valid
                        if self._is_valid_position(container, item, [wx, wy, wz], dz, container_idx, safety_margin, ceiling_margin):
                            return [wx, wy, wz]

        return None

    def _is_valid_position(self, container, item, pos, dz, container_idx, safety_margin, ceiling_margin):
        """Check if item at position is valid (within bounds and no collision)"""
        x, y, z = pos
        dx = item['length'] / 2.0
        dy = item['width'] / 2.0
        # Use actual inner dimensions from container
        thickness = container.get('thickness', 0.02)
        inner_length = container['length'] - 2 * thickness
        inner_width = container['width'] - 2 * thickness
        inner_height = container['height'] - 2 * thickness

        # Get validator config for this container
        validator = self.validator_config.get(container_idx, {
            'safety_margin': 0.015,
            'ceiling_margin': 0.018,
            'inclusion_margin': -0.005
        })
        safety_margin = validator.get('safety_margin', 0.015)
        ceiling_margin = validator.get('ceiling_margin', 0.018)
        inclusion_margin = validator.get('inclusion_margin', -0.005)

        # Check bounds with safety margin and inclusion margin
        # The validator likely expects the item to be within inner bounds reduced by safety margin
        # and also above start_z (which we already set in origin)
        min_x = -inner_length/2 + dx + safety_margin + inclusion_margin
        max_x = inner_length/2 - dx - safety_margin - inclusion_margin
        min_y = -inner_width/2 + dy + safety_margin + inclusion_margin
        max_y = inner_width/2 - dy - safety_margin - inclusion_margin
        min_z = -inner_height/2 + dz + safety_margin + inclusion_margin  # bottom
        max_z = inner_height/2 - dz - safety_margin - ceiling_margin - inclusion_margin  # top, subtract ceiling_margin

        if not (min_x <= x <= max_x):
            self._log(f"X bounds fail: {x} not in [{min_x}, {max_x}]")
            return False
        if not (min_y <= y <= max_y):
            self._log(f"Y bounds fail: {y} not in [{min_y}, {max_y}]")
            return False
        if not (min_z <= z <= max_z):
            self._log(f"Z bounds fail: {z} not in [{min_z}, {max_z}]")
            return False

        # Check against placed items with safety margin
        for (px, py, pz, pdx, pdy, pdz) in self.placed_items.get(container_idx, []):
            if (abs(x - px) < (dx + pdx/2 + safety_margin) or
                abs(y - py) < (dy + pdy/2 + safety_margin) or
                abs(z - pz) < (dz + pdz/2 + safety_margin)):
                self._log(f"Collision with item at ({px},{py},{pz}) size ({pdx},{pdy},{pdz})")
                return False

        return True

    def _is_valid_placement_simple(self, container, item, pos, placed_items, safety_margin, ceiling_margin):
        """Simple collision check for fallback positions"""
        x, y, z = pos
        dx = item['length'] / 2.0
        dy = item['width'] / 2.0
        dz = item['height'] / 2.0

        inner_length = container['length'] - 2 * container.get('thickness', 0.02)
        inner_width = container['width'] - 2 * container.get('thickness', 0.02)
        inner_height = container['height'] - 2 * container.get('thickness', 0.02)

        # Get validator config for this container
        validator = self.validator_config.get(0, {
            'safety_margin': 0.015,
            'ceiling_margin': 0.018,
            'inclusion_margin': -0.005
        })  # Assume container_idx 0 for fallback
        safety_margin = validator.get('safety_margin', 0.015)
        ceiling_margin = validator.get('ceiling_margin', 0.018)
        inclusion_margin = validator.get('inclusion_margin', -0.005)

        # Check bounds with safety margin
        min_x = -inner_length/2 + dx + safety_margin + inclusion_margin
        max_x = inner_length/2 - dx - safety_margin - inclusion_margin
        min_y = -inner_width/2 + dy + safety_margin + inclusion_margin
        max_y = inner_width/2 - dy - safety_margin - inclusion_margin
        min_z = -inner_height/2 + dz + safety_margin + inclusion_margin
        max_z = inner_height/2 - dz - safety_margin - ceiling_margin - inclusion_margin

        if not (min_x <= x <= max_x):
            return False
        if not (min_y <= y <= max_y):
            return False
        if not (min_z <= z <= max_z):
            return False

        # Check against placed items
        for (px, py, pz, pdx, pdy, pdz) in placed_items:
            if (abs(x - px) < (dx + pdx/2 + safety_margin) or
                abs(y - py) < (dy + pdy/2 + safety_margin) or
                abs(z - pz) < (dz + pdz/2 + safety_margin)):
                return False

        return True