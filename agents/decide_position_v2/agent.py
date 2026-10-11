"""
グランドハンドリングシミュレータ用エージェント (decide_position_v2)

設計方針:
    depth_map(前面からの距離画像)は値の解釈に曖昧さが残るため使用しない。
    代わりに observation['container_list'][i]['packed_items'] に含まれる
    「既配置荷物の実座標(pos)・姿勢(orn, クォータニオン)」を直接使い、
    軸並行境界ボックス(AABB)を計算して衝突しない配置を探索する。
    向き(orientation)は0〜5の6通りすべて90度単位の回転なので、
    クォータニオンからでも厳密にAABBの半径(half-extent)を計算できる。

座標系メモ:
    - packed_items / pool_list の pos は世界座標系(world)。
    - action の place_pos は「(offset_x, 0, 0) を原点とした相対座標」。
    - container_list[i]['center'] は世界座標系でのコンテナ中心。
      コンテナはX軸方向にのみオフセットして並ぶ(同じ姿勢)ため、
      offset_x == center[0]、かつ Y, Z は world と local で一致すると仮定する。
      => local = (world_x - center_x, world_y, world_z)
"""

import numpy as np

# ORNS (utils.py) と同じ並びに対応させた「向きインデックス -> (元のl,w,hをどう並べ替えるか)」
ORN_DIM_MAP = {
    0: (0, 1, 2),  # [l, w, h]
    1: (0, 2, 1),  # [l, h, w]
    2: (2, 1, 0),  # [h, w, l]
    3: (1, 0, 2),  # [w, l, h]
    4: (1, 2, 0),  # [w, h, l]
    5: (2, 0, 1),  # [h, l, w]
}

# 評価基盤の実パラメータ(README記載)に合わせたデフォルトマージン
INCLUSION_MARGIN = -0.005
SAFETY_MARGIN = 0.06   # 物理定着時のズレ・沈み込みを見込んで余裕を持たせる(元は0.015)
CEILING_MARGIN = 0.018

GRID_STEP = 0.03  # 候補位置グリッドの刻み幅[m]

# v2: PlacementValidator の inclusion_margin 自体は変更しない。
# 候補生成時に「境界面ぴったり」を避けるための追加クリアランス。
# validator は dots <= INCLUSION_MARGIN のまま、candidate はさらに内側を目指す。
INCLUSION_CANDIDATE_CLEARANCE = 0.010
INCLUSION_PROJECTION_ITERS = 4



def get_dims_for_orn(lwh: tuple[float, float, float], orn_idx: int) -> tuple[float, float, float]:
    """向きorn_idxを適用した後の(X方向, Y方向, Z方向)の寸法を返す。"""
    i, j, k = ORN_DIM_MAP[orn_idx]
    return lwh[i], lwh[j], lwh[k]


def quat_to_aabb_half_extent(half_extent: np.ndarray, quat: tuple[float, float, float, float]) -> np.ndarray:
    """
    クォータニオン(x,y,z,w)で回転した直方体のAABB半径を計算する。
    90度単位の回転であればこの式で厳密に一致する(一般の回転でも外接AABBとして有効)。
    """
    x, y, z, w = quat
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return np.abs(R) @ half_extent


class Agent:
    def __init__(self, module_path: str):
        self.module_path = module_path
        self.num_containers = 0
        self.lookahead_k = 0
        self.optimize_enabled = False

    # ------------------------------------------------------------------
    def get_init_states(self, init_states: dict) -> None:
        self.optimize_enabled = init_states.get('optimize', False)
        self.lookahead_k = init_states.get('lookahead_k', 1)
        container_list = init_states.get('container_list', [])
        self.num_containers = len(container_list)
        # README仕様上、get_init_statesの戻り値はNone(実装テンプレートもNoneを想定)

    # ------------------------------------------------------------------
    def optimize(self, item_list: list) -> list:
        """
        オフライン最適化: 体積の大きい順を基本としつつ、優先荷物・ソフト貨物を
        属性ごとにまとめる(placement_score/soft_item_scoreがグルーピングで安定しやすいため)。
        """
        def sort_key(item):
            vol = item['length'] * item['width'] * item['height']
            group = (item.get('is_prioritized', False), item.get('is_soft', False))
            return (group, -vol)

        sorted_items = sorted(item_list, key=sort_key)
        return [item['index'] for item in sorted_items]

    def _stack_depth(self, x: float, y: float, existing: list, half_extent: np.ndarray) -> int:
        """(x,y)の位置で、下に何段積まれているかを数える(footprint重なりで判定)。"""
        depth = 0
        for (ocenter, ohalf) in existing:
            if (abs(x - ocenter[0]) < (half_extent[0] + ohalf[0]) and
                    abs(y - ocenter[1]) < (half_extent[1] + ohalf[1])):
                depth += 1
        return depth

    # ------------------------------------------------------------------
    def _safe_bounds(self, container: dict, half_extent: np.ndarray):
        length = container['length']
        width = container['width']
        height = container['height']
        thickness = container.get('thickness', 0.02)
        cut_x = container.get('cut_x', 0.0)
        buf = 0.03

        # 斜めカットは低X側のみ。x_minでfootprint全体をcut_xより内側に押し込めば
        # cut_yを別途z側で考慮する必要はない(x>=cut_xの範囲は矩形のまま)。
        x_min = -length / 2 + thickness + cut_x + buf + half_extent[0]
        x_max =  length / 2 - thickness - buf - half_extent[0]

        y_min = -width / 2 + thickness + buf + half_extent[1]
        y_max =  width / 2 - thickness - buf - half_extent[1]

        z_min = thickness + buf + half_extent[2]
        z_max = height - thickness - buf - half_extent[2]   # 天井は矩形なのでcut_y不要

        if x_min > x_max or y_min > y_max or z_min > z_max:
            return None
        return x_min, x_max, y_min, y_max, z_min, z_max
    
    def _packed_aabbs(self, container: dict, container_center: np.ndarray):
        """既配置荷物それぞれについて (local_center, half_extent) のリストを返す。"""
        aabbs = []
        for packed in container.get('packed_items', []):
            if packed.get('pos') is None:
                continue
            world_pos = np.array(packed['pos'], dtype=np.float64)
            local_center = world_pos - np.array([container_center[0], 0.0, 0.0])
            half_extent = np.array([packed['length'], packed['width'], packed['height']], dtype=np.float64) / 2.0
            orn = packed.get('orn')
            if orn is not None:
                half_extent = quat_to_aabb_half_extent(half_extent, orn)
            aabbs.append((local_center, half_extent))
        return aabbs

    def _overlaps(self, center: np.ndarray, half_extent: np.ndarray,
                  other_center: np.ndarray, other_half: np.ndarray) -> bool:
        diff = np.abs(center - other_center)
        return bool(np.all(diff < (half_extent + other_half + SAFETY_MARGIN)))

    # def _find_placement(self, container: dict, container_idx: int, half_extent: np.ndarray):
    #     """
    #     与えられたhalf_extentの荷物について、このコンテナ内で衝突しない
    #     最も低いz位置の候補を探す。見つからなければNoneを返す。
    #     """
    #     container_center = np.array(container['center'], dtype=np.float64)
    #     bounds = self._safe_bounds(container, half_extent)
    #     if bounds is None:
    #         return None
    #     x_min, x_max, y_min, y_max, z_min, z_max = bounds

    #     existing = self._packed_aabbs(container, container_center)

    #     xs = np.arange(x_min, x_max, GRID_STEP)
    #     ys = np.arange(y_min, y_max, GRID_STEP)
    #     if len(xs) == 0:
    #         xs = np.array([(x_min + x_max) / 2])
    #     if len(ys) == 0:
    #         ys = np.array([(y_min + y_max) / 2])

    #     best = None  # (score, z, x, y)  score = (-y, z) を最小化 = yはできるだけ奥、zはできるだけ低い
    #     for x in xs:
    #         for y in ys:
    #             # 搬送経路は「入口(y=-width/2)から目標yまでスライド」する動きなので、
    #             # Xがかぶっている(=搬入経路上にある)別の荷物より奥には置けない。
    #             # ただし「同じ場所に積み重ねる」場合(yもかぶっている)はスライド経路上の
    #             # 障害物ではない(その場でz方向に降ろすだけ)ので、この制約の対象外とする。
    #             blocked = False
    #             for (ocenter, ohalf) in existing:
    #                 x_overlaps = abs(x - ocenter[0]) < (half_extent[0] + ohalf[0] + SAFETY_MARGIN)
    #                 y_overlaps = abs(y - ocenter[1]) < (half_extent[1] + ohalf[1] + SAFETY_MARGIN)
    #                 if x_overlaps and not y_overlaps:
    #                     obstacle_front_edge = ocenter[1] - ohalf[1]
    #                     if y + half_extent[1] + SAFETY_MARGIN > obstacle_front_edge:
    #                         blocked = True
    #                         break
    #             if blocked:
    #                 continue

    #             # このxyの真下にある既配置荷物のうち、footprintが重なるものの最大天面を探す
    #             z = z_min
    #             for (ocenter, ohalf) in existing:
    #                 if (abs(x - ocenter[0]) < (half_extent[0] + ohalf[0] + SAFETY_MARGIN) and
    #                         abs(y - ocenter[1]) < (half_extent[1] + ohalf[1] + SAFETY_MARGIN)):
    #                     top = ocenter[2] + ohalf[2] + half_extent[2] + SAFETY_MARGIN
    #                     z = max(z, top)

    #             if z > z_max:
    #                 continue

    #             cand_center = np.array([x, y, z])

    #             # 既配置荷物との衝突チェック(自前のAABB同士)
    #             collided = False
    #             for (ocenter, ohalf) in existing:
    #                 if self._overlaps(cand_center, half_extent, ocenter, ohalf):
    #                     collided = True
    #                     break
    #             if collided:
    #                 continue

    #             score = (-y, z)
    #             if best is None or score < best[0]:
    #                 best = (score, x, y, z)

    #     if best is None:
    #         return None
    #     _, x, y, z = best
    #     return np.array([x, y, z], dtype=np.float32)

    # ------------------------------------------------------------------
    def _is_stable(self, x: float, y: float, z: float, half_extent: np.ndarray,
                   existing: list, floor_z: float, tol: float = 0.02) -> bool:
        """
        底面の四隅それぞれが、床 or 既配置荷物の天面によって
        ほぼ同じ高さ(bottom_z)で支えられているかを確認する。
        1つでも支えのない隅があれば不安定とみなし False を返す。
        """
        corners = [
            (x - half_extent[0], y - half_extent[1]),
            (x - half_extent[0], y + half_extent[1]),
            (x + half_extent[0], y - half_extent[1]),
            (x + half_extent[0], y + half_extent[1]),
        ]
        bottom_z = z - half_extent[2]

        for cx, cy in corners:
            supported = False

            # 床に直接接している隅はOK
            if abs(bottom_z - floor_z) < tol:
                supported = True
            else:
                for (ocenter, ohalf) in existing:
                    # その隅が既存荷物のfootprint範囲内にあるか
                    within_x = (ocenter[0] - ohalf[0] - tol) <= cx <= (ocenter[0] + ohalf[0] + tol)
                    within_y = (ocenter[1] - ohalf[1] - tol) <= cy <= (ocenter[1] + ohalf[1] + tol)
                    if within_x and within_y:
                        top = ocenter[2] + ohalf[2]
                        if abs(top - bottom_z) < tol:
                            supported = True
                            break

            if not supported:
                return False

        return True

    # ------------------------------------------------------------------
    def _inclusion_dots(self, container: dict, target_local: np.ndarray, half_extent: np.ndarray):
        """PlacementValidator.check_inclusion() と同じ plane 値を返す。"""
        n_vecs = container.get('n_vecs')
        points = container.get('points')
        if n_vecs is None or points is None:
            return None
        target_world = np.asarray(target_local, dtype=np.float64).copy()
        center = np.asarray(container.get('center', [0.0, 0.0, 0.0]), dtype=np.float64)
        target_world[0] += center[0]
        n = np.asarray(n_vecs, dtype=np.float64)
        p = np.asarray(points, dtype=np.float64)
        return (n * (target_world - p)).sum(axis=1) + np.dot(np.abs(n), half_extent)

    def _project_inside_container(self, container: dict, target_local: np.ndarray,
                                  half_extent: np.ndarray):
        """
        v2 の中心変更点。

        既存の矩形 _safe_bounds だけでは拾えない斜め/実境界面を、
        PlacementValidator と同じ plane 式で再確認する。
        外側に出ている場合は worst plane の法線方向へ中心を少し内側へ
        投影する。validator の inclusion_margin は一切変更しない。

        戻り値: (projected_local, diagnostics) / 失敗時 (None, diagnostics)
        """
        n_vecs = container.get('n_vecs')
        points = container.get('points')
        if n_vecs is None or points is None:
            return np.asarray(target_local, dtype=np.float64), {
                'available': False,
                'reason': 'no-plane-data',
            }

        n = np.asarray(n_vecs, dtype=np.float64)
        p = np.asarray(points, dtype=np.float64)
        center = np.asarray(container.get('center', [0.0, 0.0, 0.0]), dtype=np.float64)
        q = np.asarray(target_local, dtype=np.float64).copy()

        before = self._inclusion_dots(container, q, half_extent)
        if before is None:
            return q, {'available': False, 'reason': 'no-plane-data'}

        target_limit = float(INCLUSION_MARGIN - INCLUSION_CANDIDATE_CLEARANCE)
        applied = []

        for _ in range(INCLUSION_PROJECTION_ITERS):
            world = q.copy()
            world[0] += center[0]
            dots = (n * (world - p)).sum(axis=1) + np.dot(np.abs(n), half_extent)
            worst = int(np.argmax(dots))
            violation = float(dots[worst] - target_limit)
            if violation <= 0.0:
                break

            normal = n[worst]
            norm2 = float(np.dot(normal, normal))
            if norm2 <= 1e-12:
                return None, {
                    'available': True,
                    'reason': 'zero-normal',
                    'before_max_dot': float(np.max(before)),
                    'after_max_dot': float(np.max(dots)),
                    'worst_plane': worst,
                }

            # dot が violation 分だけ減るように target を -n 方向へ移動。
            delta = normal * (violation / norm2)
            q -= delta
            applied.append({
                'plane': worst,
                'violation': violation,
                'delta': delta.tolist(),
            })

        after = self._inclusion_dots(container, q, half_extent)
        if after is None:
            return None, {'available': False, 'reason': 'no-plane-data'}

        ok = bool(np.all(after <= INCLUSION_MARGIN))
        diag = {
            'available': True,
            'before_max_dot': float(np.max(before)),
            'after_max_dot': float(np.max(after)),
            'worst_plane_before': int(np.argmax(before)),
            'worst_plane_after': int(np.argmax(after)),
            'target_limit': target_limit,
            'projected': bool(applied),
            'iterations': len(applied),
            'deltas': applied,
        }
        if not ok:
            diag['reason'] = 'projection-failed'
            return None, diag
        return q, diag

    def _passes_exact_inclusion(self, container: dict, target_local: np.ndarray,
                                half_extent: np.ndarray, *, label='candidate') -> bool:
        dots = self._inclusion_dots(container, target_local, half_extent)
        if dots is None:
            return True
        ok = bool(np.all(dots <= INCLUSION_MARGIN))
        if not ok:
            worst = int(np.argmax(dots))
            print(
                f'[decide_position_v2][INCLUSION-REJECT] {label} '
                f'plane={worst} max_dot={float(dots[worst]):.9f} '
                f'limit={INCLUSION_MARGIN:.6f} '
                f'target=({target_local[0]:.6f},{target_local[1]:.6f},{target_local[2]:.6f})',
                flush=True,
            )
        return ok

    def _find_placement(self, container: dict, container_idx: int, half_extent: np.ndarray):
        container_center = np.array(container['center'], dtype=np.float64)
        bounds = self._safe_bounds(container, half_extent)
        if bounds is None:
            return None
        x_min, x_max, y_min, y_max, z_min, z_max = bounds

        # 床面そのものの高さ(荷物の半分の厚みは含めない生の値)
        thickness = container.get('thickness', 0.02)
        buf = 0.03
        floor_z = thickness + buf   # ← z_minではなく、こちらを使う

        existing = self._packed_aabbs(container, container_center)

        xs = np.arange(x_min, x_max, GRID_STEP)
        ys = np.arange(y_min, y_max, GRID_STEP)
        if len(xs) == 0:
            xs = np.array([(x_min + x_max) / 2])
        if len(ys) == 0:
            ys = np.array([(y_min + y_max) / 2])

        best = None
        for x in xs:
            for y in ys:
                blocked = False
                for (ocenter, ohalf) in existing:
                    x_overlaps = abs(x - ocenter[0]) < (half_extent[0] + ohalf[0] + SAFETY_MARGIN)
                    y_overlaps = abs(y - ocenter[1]) < (half_extent[1] + ohalf[1] + SAFETY_MARGIN)
                    if x_overlaps and not y_overlaps:
                        obstacle_front_edge = ocenter[1] - ohalf[1]
                        if y + half_extent[1] + SAFETY_MARGIN > obstacle_front_edge:
                            blocked = True
                            break
                if blocked:
                    continue

                depth = self._stack_depth(x, y, existing, half_extent)
                dynamic_margin = SAFETY_MARGIN + 0.015 * depth  # 1段ごとに1.5cm上乗せ

                z = z_min
                for (ocenter, ohalf) in existing:
                    if (abs(x - ocenter[0]) < (half_extent[0] + ohalf[0] + dynamic_margin) and
                            abs(y - ocenter[1]) < (half_extent[1] + ohalf[1] + dynamic_margin)):
                        top = ocenter[2] + ohalf[2] + half_extent[2] + dynamic_margin
                        z = max(z, top)

                # z = z_min
                # for (ocenter, ohalf) in existing:
                #     if (abs(x - ocenter[0]) < (half_extent[0] + ohalf[0] + SAFETY_MARGIN) and
                #             abs(y - ocenter[1]) < (half_extent[1] + ohalf[1] + SAFETY_MARGIN)):
                #         top = ocenter[2] + ohalf[2] + half_extent[2] + SAFETY_MARGIN
                #         z = max(z, top)

                if z > z_max:
                    continue

                cand_center = np.array([x, y, z], dtype=np.float64)

                # v2: 矩形 bounds だけでなく、実 validator の plane で inclusion を確認。
                # 境界ぴったりの候補は worst-plane 法線方向へ内側に投影してから再評価する。
                projected, inc_diag = self._project_inside_container(
                    container, cand_center, half_extent
                )
                if projected is None:
                    continue
                cand_center = projected

                # 投影後に z 上限などを越えた場合は候補として破棄する。
                if cand_center[2] < z_min - 1e-9 or cand_center[2] > z_max + 1e-9:
                    continue
                if not self._passes_exact_inclusion(
                    container, cand_center, half_extent, label='candidate'
                ):
                    continue

                x, y, z = map(float, cand_center)

                collided = False
                for (ocenter, ohalf) in existing:
                    if self._overlaps(cand_center, half_extent, ocenter, ohalf):
                        collided = True
                        break
                if collided:
                    continue

                # ---- ここから追加: 四隅支持による安定性チェック ----
                if not self._is_stable(x, y, z, half_extent, existing, floor_z):
                    continue
                # ----------------------------------------------------

                score = (-y, z)
                if best is None or score < best[0]:
                    best = (score, x, y, z)

        if best is None:
            return None
        _, x, y, z = best
        return np.array([x, y, z], dtype=np.float32)

    # ------------------------------------------------------------------
    def policy(self, observation: dict) -> dict:
        try:
            return self._policy_impl(observation)
        except Exception as e:
            # 想定外の例外で丸ごとエピソードが落ちるのを避け、原因調査用にログだけ残す
            import traceback
            print(f'[Agent.policy] error: {e}')
            traceback.print_exc()
            container_list = observation.get('container_list', [])
            fallback_container_idx = container_list[0]['index'] if container_list else 0
            return {
                'item_idx': 0,
                'container_idx': fallback_container_idx,
                'place_pos': np.array([0.0, 0.0, 0.5], dtype=np.float32),
                'orientation': 0,
            }

    def _policy_impl(self, observation: dict) -> dict:
        pool_list = observation.get('pool_list', [])
        container_list = observation.get('container_list', [])

        best_action = None
        best_score = None

        for item_idx, item in enumerate(pool_list):
            lwh = (item['length'], item['width'], item['height'])
            is_prioritized = item.get('is_prioritized', False)

            for orn_idx in range(6):
                dims = get_dims_for_orn(lwh, orn_idx)
                half_extent = np.array(dims, dtype=np.float64) / 2.0

                for container in container_list:
                    container_idx = container['index']

                    # 優先荷物用コンテナへの割り当てを軽く優先(placement_scoreの減点回避)
                    container_is_prioritized = container.get('is_prioritized', False)
                    mismatch_penalty = 10.0 if (is_prioritized and not container_is_prioritized) else 0.0

                    # for packed in container.get('packed_items', []):
                    #     print(f"[DEBUG] packed item idx={packed.get('index')}"
                    #         f" pos={packed.get('pos')} orn={packed.get('orn')}"
                    #         f" lwh=({packed['length']}, {packed['width']}, {packed['height']})"
                    #           )

                    pos = self._find_placement(container, container_idx, half_extent)
                    if pos is None:
                        continue

                    # 低いzを優先(重心スコア向上) + 属性ミスマッチにペナルティ
                    score = float(pos[2]) + mismatch_penalty
                    if best_score is None or score < best_score:
                        best_score = score
                        best_action = {
                            'item_idx': item_idx,
                            'container_idx': container_idx,
                            'place_pos': pos,
                            'orientation': orn_idx,
                        }

        if best_action is not None:
            return best_action

        # フォールバック: どこにも置けなかった場合(型だけは満たして返す)
        fallback_container_idx = container_list[0]['index'] if container_list else 0
        return {
            'item_idx': 0,
            'container_idx': fallback_container_idx,
            'place_pos': np.array([0.0, 0.0, 0.5], dtype=np.float32),
            'orientation': 0,
        }