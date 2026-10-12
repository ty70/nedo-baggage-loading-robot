"""Verify observed transport bounds independently against PyBullet."""
import math
import unittest

import pybullet as p

from agents.search_cand_v16_observed_pose.agent import Agent, observed_box_extents


class ObservedPoseTests(unittest.TestCase):
    def test_rotated_bounds_match_bullet(self):
        client = p.connect(p.DIRECT)
        try:
            dimensions = (.65, .45, .25)
            shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[v/2 for v in dimensions], physicsClientId=client)
            for euler in ((0, 0, 0), (.00002, -.00003, .00004), (.2, -.3, .4), (0, 0, math.pi/2)):
                orn = p.getQuaternionFromEuler(euler)
                body = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=shape, basePosition=(.2, -.1, .7), baseOrientation=orn, physicsClientId=client)
                lower, upper = p.getAABB(body, physicsClientId=client)
                result = observed_box_extents(dict(length=.65, width=.45, height=.25, orn=orn))
                for measured, actual in zip(result, (b-a for a, b in zip(lower, upper))):
                    self.assertAlmostEqual(measured, actual, places=10)
                p.removeBody(body, physicsClientId=client)
        finally:
            p.disconnect(client)

    def test_tilt_blocks_route_previously_accepted(self):
        agent = Agent.__new__(Agent)
        agent.placed_orientation = {15: 0}
        agent.start_margin = .015
        agent.safety_margin = .015
        container = dict(length=2, width=2, height=2, thickness=.04, cut_x=0, center=(0, 0, 0))
        moving = dict(index=38, length=.4, width=.4, height=.2)
        obstacle = dict(index=15, length=.4, width=.4, height=.2, pos=(.61501, 0, .58), orn=p.getQuaternionFromEuler((0, 0, .0001)))
        packed = agent._oriented_packed([obstacle])
        from agents.search_cand_v15_shelf_clearance.agent import Agent as Previous
        self.assertTrue(Previous._actual_simulator_entry_clear(agent, container, moving, (.2, 0, .5), packed, False))
        self.assertFalse(agent._actual_simulator_entry_clear(container, moving, (.2, 0, .5), packed, False))
        self.assertEqual(packed[0]['length'], .4)
        self.assertEqual(obstacle['length'], .4)

    def test_invalid_or_missing_observation_falls_back(self):
        self.assertIsNone(observed_box_extents({'orn': None}))
        self.assertIsNone(observed_box_extents({'orn': (0, 0, 0, 0)}))
        self.assertIsNone(observed_box_extents({'orn': (0, 0, 0, float('nan'))}))


if __name__ == '__main__':
    unittest.main()
