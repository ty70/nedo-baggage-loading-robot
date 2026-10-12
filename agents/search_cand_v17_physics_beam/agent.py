"""Beam states contain physically transported and settled cargo, not reservations."""
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

from agents.search_cand_v16_observed_pose.agent import Agent as V16Agent
from .physics_world import PhysicsWorld


class Agent(V16Agent):
    def __init__(self, module_path):
        super().__init__(module_path)
        self.physics_beam = max(1, int(os.environ.get('SC_PHYSICS_BEAM', '8')))
        self.physics_branch = max(1, int(os.environ.get('SC_PHYSICS_BRANCH', '2')))
        self.physics_item_branch = max(1, int(os.environ.get('SC_PHYSICS_ITEM_BRANCH', '8')))
        self.physics_candidate_pool = max(self.physics_branch, int(os.environ.get('SC_PHYSICS_CANDIDATES', '16')))
        self.physics_trials = max(1, int(os.environ.get('SC_PHYSICS_TRIALS', '3')))
        self.physics_budget = min(150.0, max(1.0, float(os.environ.get('SC_PHYSICS_SECONDS', '60'))))
        self.policy_budget = min(5.0, max(.2, float(os.environ.get('SC_PHYSICS_POLICY_SECONDS', '2'))))
        self.physics_stats = {'attempted': 0, 'transport_rejected': 0, 'settling_rejected': 0, 'accepted': 0}
        self.physics_trace = []

    def get_init_states(self, init_states):
        result = super().get_init_states(init_states)
        # Observations do not contain validator settings. Local runners provide
        # the exact input config via CLI; fail rather than silently invent it.
        config_path = os.environ.get('SC_PHYSICS_CONFIG')
        if config_path is None and '--config-path' in sys.argv:
            config_path = sys.argv[sys.argv.index('--config-path') + 1]
        if config_path is None:
            raise RuntimeError('Physics beam requires --config-path or SC_PHYSICS_CONFIG.')
        configs = json.loads(Path(config_path).expanduser().read_text())
        matches = []
        for config in configs.values():
            if bool(config['agent']['optimize']) != self.optimize_enabled:
                continue
            if int(config['item_stream']['look_ahead']) != self.lookahead_k:
                continue
            cs = config['containers']['container_list']
            if len(cs) != len(self.container_init):
                continue
            if all(all(abs(float(c[k])-float(obs[k])) < 1e-8 for k in ('length', 'width', 'height', 'thickness', 'cut_x', 'cut_y'))
                   and bool(c.get('require_shelf', False)) == bool(obs.get('shelf', False))
                   for c, obs in zip(cs, self.container_init)):
                matches.append(config['validator'])
        unique = {json.dumps(v, sort_keys=True) for v in matches}
        if len(unique) != 1:
            raise RuntimeError('Cannot resolve exact validator settings for the observed scene.')
        self.physics_validator = json.loads(unique.pop())
        print(f'[V17-PHYSICS] beam={self.physics_beam} branch={self.physics_branch} trials={self.physics_trials} '
              f'validator={self.physics_validator}', flush=True)
        return result

    def _geometry(self, snapshot):
        previous = self.placed_orientation
        self.placed_orientation = dict(previous)
        for records in snapshot.values():
            for raw in records:
                self.placed_orientation[int(raw['index'])] = int(raw.get('_prediction_orientation', previous.get(int(raw['index']), 0)))
        try:
            return {cidx: self._oriented_packed(records) for cidx, records in snapshot.items()}
        finally:
            self.placed_orientation = previous

    def _state_metrics(self, state):
        geometric = self._layout_metrics(state['plans'])
        return (float(state.get('fill_score', 0)), len(state['plans']), geometric[0], geometric[2], geometric[3])

    def _initial_snapshot(self, containers):
        # Observations do not supply velocities: initialize those to zero.
        return {idx: [dict(raw, _prediction_orientation=self.placed_orientation.get(int(raw['index']), 0))
                      for raw in c.get('packed_items', [])] for idx, c in enumerate(containers)}

    def _generate_extreme_points(self, c, packed):
        eps = list(super()._generate_extreme_points(c, packed))
        # Physics must inspect both ordinary and projected EPs; a geometrically
        # feasible ordinary EP can still have no executable transport path.
        eps = [tuple(ep[:3]) + (str(ep[3]).replace(':proj:', ':physics-proj:'),) for ep in eps]
        t = float(c.get('thickness', .04))
        eps.append((float(c['length'])/2-t-self.boundary_margin,
                    float(c['width'])/2-t-self.boundary_margin,
                    t+self.boundary_margin, 'physics-floor-right'))
        return eps

    def _physical_children(self, state, raw, containers, world, deadline):
        geometry = self._geometry(state['snapshot'])
        candidates, _, _ = self._find_top_candidates_local(
            raw, containers, geometry, {i: set() for i in geometry}, limit=self.physics_candidate_pool)
        children = []
        for score, cidx, pos, item, ep, support in candidates:
            if time.perf_counter() >= deadline:
                break
            iid, ori = int(raw['index']), int(item['_orientation'])
            support_obj = support.get('support_obj')
            plan = {'item_id': iid, 'container_idx': cidx, 'final_target': pos, 'orientation': ori,
                    'item': dict(item), 'ep': ep, 'support_kind': support['kind'],
                    'support_ratio': float(support['ratio']),
                    'support_item_id': int(support_obj['index']) if support_obj else None,
                    'score': float(score), 'plan_rank': len(state['order']), 'source': 'physics_beam'}
            execution, _ = self.insertion_planner.execution_target(plan)
            execution = tuple(map(float, np.asarray(self._clamp_to_safe_margin(containers[cidx], item, execution, ori), dtype=np.float32)))
            child_snapshot = world.transition(state['snapshot'], raw, cidx, execution, ori)
            self.physics_stats['attempted'] += 1
            if child_snapshot is None:
                key = 'settling_rejected' if world.last_report['transported'] else 'transport_rejected'
                self.physics_stats[key] += 1
                if len(self.physics_trace) < 60:
                    self.physics_trace.append(dict(world.last_report, phase='search', execution=execution))
                continue
            self.physics_stats['accepted'] += 1
            settled = next(obj for obj in child_snapshot[cidx] if int(obj['index']) == iid)
            plan['execution_target'] = execution
            plan['predicted_settled'] = dict(settled)
            plan['packed_record'] = self._geometry({cidx: [settled]})[cidx][0]
            child = {'snapshot': child_snapshot, 'plans': dict(state['plans']),
                     'order': state['order'] + [iid], 'score_sum': state['score_sum'] + float(score),
                     'fill_score': world.last_report['fill_score']}
            child['plans'][iid] = plan
            children.append(child)
            if len(children) >= self.physics_branch:
                break
        return children

    def _physics_search(self, items, containers, world, deadline, depth=None):
        frontier = [{'snapshot': self._initial_snapshot(containers), 'plans': {}, 'order': [], 'score_sum': 0.0}]
        best = frontier[0]
        for level in range(min(len(items), depth) if depth is not None else len(items)):
            if time.perf_counter() >= deadline:
                break
            expanded = []
            for state in frontier:
                pending = [raw for raw in items if int(raw['index']) not in state['plans']]
                # Branch on insertion order as well as position/orientation.
                # If this window is entirely blocked, advance to other cargo.
                for offset in range(0, len(pending), self.physics_item_branch):
                    window_children = []
                    for raw in pending[offset:offset+self.physics_item_branch]:
                        if time.perf_counter() >= deadline:
                            break
                        window_children.extend(self._physical_children(state, raw, containers, world, deadline))
                    expanded.extend(window_children)
                    if window_children or time.perf_counter() >= deadline:
                        break
            if not expanded:
                break
            expanded.sort(key=self._state_metrics, reverse=True)
            if self._state_metrics(expanded[0]) > self._state_metrics(best):
                best = expanded[0]
            frontier, signatures, diversity = [], set(), {}
            for state in expanded:
                signature = tuple((idx, int(obj['index']), tuple(round(v, 6) for v in obj['pos']), tuple(round(v, 6) for v in obj['orn']))
                                  for idx, records in sorted(state['snapshot'].items()) for obj in records)
                # Avoid spending the entire beam on similar placements of the
                # same next cargo from the same parent order.
                key = tuple(state['order'])
                if signature in signatures or diversity.get(key, 0) >= (1 if level == 0 else 2):
                    continue
                signatures.add(signature)
                diversity[key] = diversity.get(key, 0) + 1
                frontier.append(state)
                if len(frontier) >= self.physics_beam:
                    break
            if not frontier:
                break
        return best

    def optimize(self, item_list):
        self._register_item_metadata(item_list)
        started = time.perf_counter()
        deadline = started + self.physics_budget
        base = self._cargo_order_items(item_list)
        orders = [base, sorted(base, key=lambda it: float(it['length'])*float(it['width'])*float(it['height']), reverse=True),
                  sorted(base, key=lambda it: max(float(it[k]) for k in ('length','width','height')), reverse=True)]
        best = None
        world = PhysicsWorld(self.container_init, self.physics_validator)
        try:
            for trial, order in enumerate(orders[:self.physics_trials]):
                self.diag_trial = f'physics-{trial+1}'
                state = self._physics_search(order, self.container_init, world, deadline)
                if best is None or self._state_metrics(state) > self._state_metrics(best):
                    best = state
                print(f'[V17-BEAM] trial={trial+1} packed={len(state["plans"])} elapsed={time.perf_counter()-started:.3f}s', flush=True)
                if time.perf_counter() >= deadline:
                    break
        finally:
            world.close()
        self.preplanned = dict(best['plans'])
        self.geometric_preplanned = dict(best['plans'])
        self.feasibility_order = list(best['order'])
        self.plan_counter = len(self.preplanned)
        self._dump_bpp_layout(self.container_init, stage='optimize')
        self._write_physics_report()
        seen = set(best['order'])
        order = best['order'] + [int(it['index']) for it in base if int(it['index']) not in seen]
        print(f'[V17-ORDER] order={order} stats={self.physics_stats}', flush=True)
        return order

    def policy(self, observation):
        self.step_no += 1
        containers, pool = observation.get('container_list', []), observation.get('pool_list', [])
        if not containers or not pool:
            return self._safe_abort_no_transport(containers[0] if containers else None, pool, 'MISSING-POOL-OR-CONTAINER')
        self._register_item_metadata(pool)
        world = PhysicsWorld(containers, self.physics_validator)
        chosen = None
        deadline = time.perf_counter() + self.policy_budget
        try:
            snapshot = self._initial_snapshot(containers)
            by_id = {int(it['index']): (idx, it) for idx, it in enumerate(pool)}
            for iid in self.feasibility_order:
                if iid not in by_id or iid in self.executed_ids:
                    continue
                plan = self.preplanned[iid]
                _, raw = by_id[iid]
                child = world.transition(snapshot, raw, plan['container_idx'], plan['execution_target'], plan['orientation'])
                if child is not None:
                    chosen = plan
                    break
                print(f'[V17-RUNTIME-REJECT] item={iid} report={world.last_report}', flush=True)
                self.physics_trace.append(dict(world.last_report, phase='runtime'))
                if time.perf_counter() >= deadline:
                    break
            if chosen is None and time.perf_counter() < deadline:
                state = self._physics_search(self._cargo_order_items(pool), containers, world, deadline, depth=3)
                if state['order']:
                    chosen = state['plans'][state['order'][0]]
                    self.preplanned[int(chosen['item_id'])] = chosen
                    self.geometric_preplanned[int(chosen['item_id'])] = chosen
                    print(f'[V17-RUNTIME-REPLAN] item={chosen["item_id"]} target={chosen["execution_target"]}', flush=True)
        finally:
            world.close()
        self._write_physics_report()
        if chosen is None:
            return self._safe_abort_no_transport(containers[0], pool, 'NO-PHYSICS-VALID-CANDIDATE')
        iid, cidx, ori = int(chosen['item_id']), int(chosen['container_idx']), int(chosen['orientation'])
        self.executed_ids.add(iid)
        self.placed_orientation[iid] = ori
        self._record_internal_placement(cidx, containers[cidx], chosen['item'], chosen['final_target'], ori)
        self._dump_bpp_layout(containers, stage='physics_runtime')
        pidx = next(i for i, raw in enumerate(pool) if int(raw['index']) == iid)
        print(f'[V17-ISSUED] item={iid} target={chosen["execution_target"]} physical_check=True', flush=True)
        return {'item_idx': pidx, 'container_idx': cidx, 'place_pos': np.asarray(chosen['execution_target'], dtype=np.float32), 'orientation': ori}

    def _write_physics_report(self):
        if self.bpp_dump_path is not None:
            path = self.bpp_dump_path.with_name('physics_prediction.json')
            path.write_text(json.dumps({'stats': self.physics_stats, 'trace': self.physics_trace,
                                       'validator': self.physics_validator, 'limitations': 'Reconstructed pose/velocity scene; no contact warm-start cache, observations lack velocity.'}, indent=2)+'\n')
