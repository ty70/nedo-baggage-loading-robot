"""Physical branches must inherit settled state and isolate sibling attempts."""
import copy
import json
from pathlib import Path
import unittest

from agents.search_cand_v17_physics_beam.physics_world import PhysicsWorld
from agents.search_cand_v17_physics_beam.agent import Agent
from src.ground_handling.items import Item


class PhysicsBeamTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads(Path('configs/sample_config.json').read_text())['000']
        raw = self.config['containers']['container_list'][0]
        self.info = dict(raw, center=(0, 0, raw['height']/2 + raw['buffer']), shelf=raw['require_shelf'])
        self.item = dict(index=777, length=.4, width=.3, height=.25, mass=4, is_soft=True)
        self.world = PhysicsWorld([self.info], self.config['validator'])

    def tearDown(self):
        self.world.close()

    def test_transition_matches_official_validator(self):
        target = (.2, .3, .18)
        predicted = self.world.transition({0: []}, self.item, 0, target, 0)
        self.assertIsNotNone(predicted)
        self.world.restore({0: []})
        original = Item(**self.item)
        container = self.world.containers[0]
        import numpy as np
        global_target = container.local_to_global(np.asarray(target, dtype=np.float32))
        self.assertTrue(self.world.validator.check_inclusion(container, original, global_target, 0))
        self.assertTrue(self.world.validator.check_transport_path(container, original, global_target, 0))
        self.assertTrue(self.world.validator.place_item(original, global_target, 0))
        pos, orn = original.get_pose(self.world.client)
        for a, b in zip(pos, predicted[0][0]['pos']):
            self.assertAlmostEqual(a, b, places=10)
        for a, b in zip(orn, predicted[0][0]['orn']):
            self.assertAlmostEqual(a, b, places=10)

    def test_settled_parent_and_sibling_isolation(self):
        parent = {0: []}
        saved = copy.deepcopy(parent)
        first = self.world.transition(parent, self.item, 0, (.2, .3, .18), 0)
        sibling = self.world.transition(parent, self.item, 0, (.6, .3, .18), 0)
        self.assertEqual(parent, saved)
        self.assertNotEqual(first[0][0]['pos'], sibling[0][0]['pos'])
        self.assertLess(first[0][0]['pos'][2], .18)
        self.assertIn('_linear_velocity', first[0][0])
        self.assertIn('_angular_velocity', first[0][0])
        colliding = dict(self.item, index=778)
        self.assertIsNone(self.world.transition(first, colliding, 0, (.2, .3, .18), 0))
        self.assertFalse(self.world.last_report['transported'])

    def test_settling_failure_does_not_become_child(self):
        self.world.validator.config['displacement_threshold'] = .001
        self.assertIsNone(self.world.transition({0: []}, self.item, 0, (.2, .3, .18), 0))
        self.assertTrue(self.world.last_report['transported'])
        self.assertFalse(self.world.last_report['settled'])

    def test_new_load_updates_supporters_settled_pose(self):
        parent = self.world.transition({0: []}, self.item, 0, (.2, .3, .18), 0)
        supporter = parent[0][0]
        payload = dict(index=778, length=.3, width=.25, height=.2, mass=2, is_soft=True)
        target_z = supporter['pos'][2] + self.item['height']/2 + payload['height']/2 + .003
        child = self.world.transition(parent, payload, 0, (.2, .3, target_z), 0)
        self.assertIsNotNone(child)
        changed = next(obj for obj in child[0] if obj['index'] == 777)
        self.assertGreater(abs(changed['pos'][2] - supporter['pos'][2]), 1e-6)
        self.assertEqual(len(parent[0]), 1)
        self.assertEqual(len(child[0]), 2)

    def test_branch_geometry_uses_that_branches_orientation(self):
        agent = Agent.__new__(Agent)
        agent.placed_orientation = {777: 0}
        import pybullet as p
        raw = dict(self.item, pos=(.2, .3, .18), orn=p.getQuaternionFromEuler((0, 0, 1.5707963267948966)),
                   _prediction_orientation=3)
        geometry = agent._geometry({0: [raw]})[0][0]
        self.assertEqual((geometry['length'], geometry['width'], geometry['height']), (.3, .4, .25))
        self.assertEqual(agent.placed_orientation, {777: 0})
        self.assertEqual(raw['length'], .4)

    def test_beam_can_change_insertion_order(self):
        agent = Agent.__new__(Agent)
        agent.placed_orientation = {}
        agent.physics_item_branch = 2
        agent.physics_beam = 2
        # Cargo 1 can only enter after cargo 2; no failed attempt makes a child.
        def children(state, raw, containers, world, deadline):
            iid = raw['index']
            if iid == 1 and 2 not in state['plans']:
                return []
            plan = dict(item=dict(length=.2, width=.2, height=.2, mass=1), final_target=(0, 0, .2), score=0)
            return [dict(snapshot={0: state['snapshot'][0]+[dict(index=iid, pos=(iid, 0, .2), orn=(0, 0, 0, 1))]},
                         plans={**state['plans'], iid: plan}, order=state['order']+[iid], score_sum=0,
                         fill_score=len(state['plans'])+1)]
        agent._physical_children = children
        import time
        result = agent._physics_search([{'index': 1}, {'index': 2}], [dict(packed_items=[])], None, time.perf_counter()+1)
        self.assertEqual(result['order'], [2, 1])
        self.assertEqual(len(result['snapshot'][0]), 2)


if __name__ == '__main__':
    unittest.main()
