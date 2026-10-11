"""公式validatorを変更せず、境界衝突とv15の経路補正を再現する。"""
import json
import os
import sys
from pathlib import Path

import numpy as np
import pybullet as p

checker_path = Path(os.environ.get('NEDO_CLOUD_STATE', '/workspace/nedo-cloud')) / 'checker'
sys.path.insert(0, str(checker_path))
import cpp_aabb_checker
assert Path(cpp_aabb_checker.__file__).resolve().parent == checker_path.resolve()
from agents.search_cand_v15_shelf_clearance.agent import Agent
from agents.search_cand_v14_order_2716_diagnostic.agent import Agent as V14
from src.ground_handling.env import GroundHandlingEnv
from src.ground_handling.items import Item
from src.ground_handling.utils import ORNS


def main():
    config = json.loads(Path('configs/sample_config.json').read_text())['000']
    env = GroundHandlingEnv(config, False)
    env.reset_settings()
    container = env.container_manager.containers[0]
    c = config['containers']['container_list'][0]
    raw = next(i for i in config['item_stream']['item_list'] if i['index'] == 39)
    oriented = dict(raw, length=.45, width=.65, height=.25, _orientation=3)
    old, new = V14('agents/search_cand_v14_order_2716_diagnostic/'), Agent('agents/search_cand_v15_shelf_clearance/')
    nominal = (-.28, .345, .687)
    issued = tuple(np.float32(nominal))
    corrected = tuple(np.float32(new._clamp_to_safe_margin(c, oriented, nominal, 3)))
    try:
        item = Item(**raw)
        item.spawn(env.client, (issued[0], -.725, issued[2] + .08), p.getQuaternionFromEuler(ORNS[3]))
        points = env.client.getClosestPoints(item.pybullet_id, container.small_shelf_bullet_id, .02)
        distance = points[0][8]
        assert abs(distance - .014999998807907153) < 1e-15
        print('元の距離:', distance, '不足(m):', .015 - distance)
        print('荷物AABB:', env.client.getAABB(item.pybullet_id))
        print('小棚AABB:', env.client.getAABB(container.small_shelf_bullet_id))
        item.remove(env.client)
        assert old._path_clear_packed_only(c, oriented, issued, [])
        assert old._static_shelf_path_clear(c, oriented, issued)
        shelf = cpp_aabb_checker.make_small_shelf(c['length'], c['width'], c['height'], c['thickness'], c['cut_x'], c['buffer'])
        moving = old._cpp_box(oriented)
        shelf_results = []
        for target in (issued, corrected):
            shelf_results.append(cpp_aabb_checker.check_transport_path_with_shelves(
                moving, [], [shelf], float(target[0]), -.725,
                float(np.float32(target[2] + .08)), float(target[0]), float(target[1]),
                step_len=.01, collision_gap=.015))
        assert not shelf_results[0]['ok'] and shelf_results[1]['ok']
        print('小棚を含むC++（実validator経路）:', shelf_results)
        assert not env.validator.check_transport_path(container, Item(**raw), issued, 3)
        # float64だけに変えても、経路途中で境界を割る。
        assert not env.validator.check_transport_path(container, Item(**raw), nominal, 3)
        assert corrected[0] > issued[0]
        assert float(corrected[0]) - .225 - (-.52) > .015 + 1e-6
        assert new._path_clear_packed_only(c, oriented, corrected, [])
        assert env.validator.check_transport_path(container, Item(**raw), corrected, 3)
        # 最初の位置以外も、両区間すべてで公式マージンを超える。
        probe = Item(**raw)
        probe.spawn(env.client, (0, 0, 0), p.getQuaternionFromEuler(ORNS[3]))
        minimum = float('inf')
        for y in np.linspace(-.725, float(corrected[1]), 109):
            probe.set_pose(env.client, (corrected[0], y, corrected[2] + .08), p.getQuaternionFromEuler(ORNS[3]))
            pts = env.client.getClosestPoints(probe.pybullet_id, container.small_shelf_bullet_id, 10)
            minimum = min(minimum, pts[0][8])
        assert minimum > config['validator']['safety_margin']
        probe.remove(env.client)
        for position in ((0, .345, .687), (-.281, .345, .687), (-.28, .345, .17)):
            assert new._clamp_to_safe_margin(c, oriented, position, 3) == old._clamp_to_safe_margin(c, oriented, position, 3)
        assert env.validator.safety_margin == .015
        print('補正後:', tuple(map(float, corrected)), '経路最小距離:', minimum)
        print('境界衝突、float64対照、C++比較、補正後経路、不変条件: PASS')
    finally:
        env.close()


if __name__ == '__main__':
    main()
