"""v15に積載後の実姿勢を使う搬入経路判定を追加する。"""
import math

from agents.search_cand_v15_shelf_clearance.agent import Agent as V15Agent


def observed_box_extents(item):
    """Fixed box's world AABB dimensions from the observed quaternion.

    These are bounds, not a claim that the cargo has physically expanded.
    Contact compliance is already reflected in its observed center/rotation.
    """
    quaternion = item.get('orn')
    if quaternion is None or len(quaternion) != 4:
        return None
    x, y, z, w = map(float, quaternion)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm <= 1e-12:
        return None
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    rotation = (
        (1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
        (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
        (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)),
    )
    dimensions = tuple(float(item[k]) for k in ('length', 'width', 'height'))
    return tuple(sum(abs(a)*d for a, d in zip(row, dimensions)) for row in rotation)


class Agent(V15Agent):
    def _oriented_packed(self, packed):
        result = super()._oriented_packed(packed)
        for raw, oriented in zip(packed, result):
            # Only observations retain physical, pre-orientation dimensions.
            # Internal action records have no observed quaternion and fall back.
            if not raw.get('_already_oriented', False):
                extents = observed_box_extents(raw)
                if extents is not None:
                    oriented['_observed_transport_extents'] = extents
        return result

    def _actual_simulator_entry_clear(self, c, item, target, packed, verbose=True):
        obstacles = []
        for obj in packed:
            copy = dict(obj)
            extents = obj.get('_observed_transport_extents')
            if extents is not None:
                copy['length'], copy['width'], copy['height'] = extents
            obstacles.append(copy)
        # Only transport bounds change. Support areas/nominal volumes retain
        # their original meanings; a rotated AABB is not a flat support surface.
        return super()._actual_simulator_entry_clear(c, item, target, obstacles, verbose)
