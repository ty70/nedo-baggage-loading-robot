"""Independent DIRECT prediction scene, using the simulator's unchanged model.

Branch restoration reconstructs bodies with their poses and velocities. Contact
solver warm-start caches are not exposed by the observation and are not cloned.
"""
import contextlib
from dataclasses import fields
import io

import numpy as np
import pybullet as p
import pybullet_data
from pybullet_utils.bullet_client import BulletClient

from src.ground_handling.containers import Container
from src.ground_handling.items import Item
from src.ground_handling.validator import PlacementValidator
from src.ground_handling.evaluator import Evaluator


ITEM_KEYS = {f.name for f in fields(Item)}


class PhysicsWorld:
    def __init__(self, container_infos, validator_config):
        self.infos = container_infos
        self.client = BulletClient(connection_mode=p.DIRECT)
        self.client.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.validator = PlacementValidator(self.client, dict(validator_config))
        self.evaluator = Evaluator(self.client, {'inclusion_margin': validator_config['inclusion_margin']})
        self.last_report = {}

    def close(self):
        self.client.disconnect()

    def restore(self, snapshot):
        self.client.resetSimulation()
        self.client.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
        self.client.setGravity(0, 0, -9.8)
        self.client.loadURDF('plane.urdf')
        self.containers = []
        for cidx, info in enumerate(self.infos):
            center = info.get('center', (0, 0, 0))
            buffer = float(center[2]) - float(info['height'])/2
            container = Container(index=int(info.get('index', cidx)), offset_x=float(center[0]),
                                  thickness=float(info['thickness']), length=float(info['length']),
                                  width=float(info['width']), height=float(info['height']), packed_items=[],
                                  require_shelf=bool(info.get('shelf', False)), buffer=buffer,
                                  cut_x=float(info.get('cut_x', 0)), cut_y=float(info.get('cut_y', 0)),
                                  is_prioritized=bool(info.get('is_prioritized', False)))
            container.create(self.client)
            self.containers.append(container)
        for cidx, records in snapshot.items():
            for raw in records:
                item = Item(**{k: v for k, v in raw.items() if k in ITEM_KEYS})
                item.spawn(self.client, initial_pos=raw['pos'], initial_orn=raw['orn'])
                self.client.resetBaseVelocity(item.pybullet_id, raw.get('_linear_velocity', (0, 0, 0)),
                                             raw.get('_angular_velocity', (0, 0, 0)))
                item._prediction_orientation = raw.get('_prediction_orientation', 0)
                self.containers[cidx].packed_items.append(item)

    def transition(self, snapshot, raw_item, cidx, local_pos, orientation):
        """Return a settled child snapshot only if inclusion/transport/settle pass."""
        self.restore(snapshot)
        item = Item(**{k: v for k, v in raw_item.items() if k in ITEM_KEYS})
        item._prediction_orientation = int(orientation)
        container = self.containers[cidx]
        issued = np.asarray(local_pos, dtype=np.float32)
        target = container.local_to_global(issued)
        captured = io.StringIO()
        with contextlib.redirect_stdout(captured):
            included = self.validator.check_inclusion(container, item, target, orientation)
            transported = included and self.validator.check_transport_path(container, item, target, orientation)
            settled = transported and self.validator.place_item(item, target, orientation)
        self.last_report = {'item_id': int(raw_item['index']), 'included': bool(included),
                            'transported': bool(transported), 'settled': bool(settled),
                            'messages': captured.getvalue().splitlines()}
        if not settled:
            return None
        container.packed_items.append(item)
        child = {}
        for idx, c in enumerate(self.containers):
            c.update_packed_items(self.client)
            child[idx] = []
            for obj in c.packed_items:
                record = obj.get_info()
                linear, angular = self.client.getBaseVelocity(obj.pybullet_id)
                record['_linear_velocity'] = linear
                record['_angular_velocity'] = angular
                record['_prediction_orientation'] = obj._prediction_orientation
                child[idx].append(record)
        with contextlib.redirect_stdout(io.StringIO()):
            fill, _ = self.evaluator.calculate_fill_rate(self.containers)
        self.last_report['fill_score'] = float(fill)
        return child
