import numpy as np
import sys


class Agent:
    """
    足切り突破を最優先した単純1段積みエージェント。

    方針
    ----
    - orientation = 0 固定
    - 1段積みのみ
    - 奥(+Y)から手前(-Y)へ探索
    - 右(+X)から左(-X)へ探索
    - cut_x 側は保守的に使用しない
    - 既配置荷物とのAABB衝突を事前チェック
    - validator と同じ Y -> X の搬入経路も簡易AABBで事前チェック
    """

    def __init__(self, module_path: str):
        self.module_path = module_path

        # 最初はデバッグ有効。
        # スコア測定時には False にしてよい。
        self.debug = True

        # validator の既定値。
        # 実際のconfigが observation に直接渡されないため、
        # 安全側に少し余裕を持たせる。
        self.safety_margin = 0.015

        # 荷物同士に確保する追加余白。
        self.extra_gap = 0.015

        # 探索位置の刻み。
        # 小さすぎると候補数が増え、大きすぎると隙間が増える。
        self.grid_step = 0.03

        self.num_containers = 0

    def _log(self, msg):
        if self.debug:
            print(f"[GRID] {msg}", file=sys.stderr)
            sys.stderr.flush()

    # ------------------------------------------------------------------
    # required API
    # ------------------------------------------------------------------

    def get_init_states(self, init_states: dict):
        self.num_containers = len(
            init_states.get("container_list", [])
        )

        self._log(
            f"init: num_containers={self.num_containers}"
        )

        return True

    def optimize(self, item_list: list):
        """
        最初は高度な最適化をしない。

        大きい荷物を先に処理するだけ。
        """

        def volume(item):
            return (
                item["length"]
                * item["width"]
                * item["height"]
            )

        ordered = sorted(
            item_list,
            key=volume,
            reverse=True,
        )

        return [item["index"] for item in ordered]

    def policy(self, observation: dict):

        pool_list = observation.get("pool_list", [])
        container_list = observation.get(
            "container_list", []
        )

        if not pool_list or not container_list:
            return self._fallback()

        # --------------------------------------------------------------
        # 最初は container 0 のみ
        # --------------------------------------------------------------

        container_idx = 0
        container = container_list[container_idx]

        packed_items = container.get(
            "packed_items", []
        )

        # --------------------------------------------------------------
        # visible poolの中で置ける荷物を探す
        #
        # 大きいものを先に試す。
        # --------------------------------------------------------------

        pool_order = sorted(
            range(len(pool_list)),
            key=lambda i:
                pool_list[i]["length"]
                * pool_list[i]["width"]
                * pool_list[i]["height"],
            reverse=True,
        )

        for pool_idx in pool_order:

            item = pool_list[pool_idx]

            pos = self._find_position(
                container,
                item,
                packed_items,
            )

            if pos is None:
                continue

            self._log(
                f"item={item['index']} "
                f"pool={pool_idx} "
                f"pos={pos}"
            )

            return {
                "item_idx": pool_idx,
                "container_idx": container_idx,
                "place_pos": np.array(
                    pos,
                    dtype=np.float32,
                ),
                "orientation": 0,
            }

        # --------------------------------------------------------------
        # 本来ここへ来る前にほぼ満杯。
        #
        # invalid actionを返すと即終了なので、
        # 最後に最も安全そうな位置を返す。
        # --------------------------------------------------------------

        self._log("no valid candidate")

        return self._fallback(
            container=container,
            item=pool_list[0],
        )

    # ------------------------------------------------------------------
    # candidate generation
    # ------------------------------------------------------------------

    def _find_position(
        self,
        container,
        item,
        packed_items,
    ):
        """
        床面1段だけを探索。

        Y:
            奥(+Y) -> 手前(-Y)

        X:
            右(+X) -> 左(-X)
        """

        L = container["length"]
        W = container["width"]

        thickness = container.get(
            "thickness", 0.02
        )

        cut_x = container.get(
            "cut_x", 0.0
        )

        item_l = item["length"]
        item_w = item["width"]
        item_h = item["height"]

        hx = item_l / 2.0
        hy = item_w / 2.0
        hz = item_h / 2.0

        # --------------------------------------------------------------
        # Z
        #
        # validator.py:
        #
        # resting surface = container.thickness
        #
        # よって荷物底面を床面に合わせる。
        # --------------------------------------------------------------

        z = thickness + hz

        # --------------------------------------------------------------
        # 有効X範囲
        #
        # validator の搬入開始位置:
        #
        # x_min =
        #   -L/2
        #   + thickness
        #   + cut_x
        #   + half_length
        #   + start_margin
        #
        # 足切り版では、target自身もこの安全なX範囲に限定する。
        #
        # つまりcut_x側を捨てる。
        # --------------------------------------------------------------

        margin = (
            self.safety_margin
            + self.extra_gap
        )

        x_min = (
            -L / 2.0
            + thickness
            + cut_x
            + hx
            + margin
        )

        x_max = (
            L / 2.0
            - thickness
            - hx
            - margin
        )

        y_min = (
            -W / 2.0
            + thickness
            + hy
            + margin
        )

        y_max = (
            W / 2.0
            - thickness
            - hy
            - margin
        )

        if x_min > x_max:
            return None

        if y_min > y_max:
            return None

        # --------------------------------------------------------------
        # 奥から手前へ。
        # --------------------------------------------------------------

        y = y_max

        while y >= y_min:

            # ----------------------------------------------------------
            # 右から左へ。
            # ----------------------------------------------------------

            x = x_max

            while x >= x_min:

                candidate = (
                    float(x),
                    float(y),
                    float(z),
                )

                # 最終位置で重ならないか
                if not self._final_position_clear(
                    container,
                    item,
                    candidate,
                    packed_items,
                ):
                    x -= self.grid_step
                    continue

                # L字搬入経路が空いているか
                if not self._transport_path_clear(
                    container,
                    item,
                    candidate,
                    packed_items,
                ):
                    x -= self.grid_step
                    continue

                return candidate

                x -= self.grid_step

            y -= self.grid_step

        return None

    # ------------------------------------------------------------------
    # final placement collision
    # ------------------------------------------------------------------

    def _final_position_clear(
        self,
        container,
        item,
        candidate,
        packed_items,
    ):
        """
        最終位置で既配置荷物とAABBが重ならないか。

        注意:
        3軸すべて重なる場合だけ collision。

        以前のコードの OR 判定は使わない。
        """

        x, y, z = candidate

        hx = item["length"] / 2.0
        hy = item["width"] / 2.0
        hz = item["height"] / 2.0

        margin = (
            self.safety_margin
            + self.extra_gap
        )

        for other in packed_items:

            pos = other.get("pos")

            if pos is None:
                continue

            ox, oy, oz = self._to_local_pos(
                container,
                pos,
            )

            ohx = other["length"] / 2.0
            ohy = other["width"] / 2.0
            ohz = other["height"] / 2.0

            overlap_x = (
                abs(x - ox)
                <
                hx + ohx + margin
            )

            overlap_y = (
                abs(y - oy)
                <
                hy + ohy + margin
            )

            overlap_z = (
                abs(z - oz)
                <
                hz + ohz + margin
            )

            if (
                overlap_x
                and overlap_y
                and overlap_z
            ):
                return False

        return True

    # ------------------------------------------------------------------
    # Y -> X transport path
    # ------------------------------------------------------------------

    def _transport_path_clear(
        self,
        container,
        item,
        candidate,
        packed_items,
    ):
        """
        validator.py の搬入経路を簡易的に再現。

            start
              |
              | Y
              |
              +------ X ------ target

        PyBulletそのものではなくAABBによる保守判定。
        """

        tx, ty, tz = candidate

        L = container["length"]
        W = container["width"]

        thickness = container.get(
            "thickness", 0.02
        )

        cut_x = container.get(
            "cut_x", 0.0
        )

        hx = item["length"] / 2.0
        hy = item["width"] / 2.0
        hz = item["height"] / 2.0

        # validatorのstart_margin既定値
        start_margin = 0.01

        x_min = (
            -L / 2.0
            + thickness
            + cut_x
            + hx
            + start_margin
        )

        x_max = (
            L / 2.0
            - thickness
            - hx
            - start_margin
        )

        # validatorと同じ
        start_x = min(
            max(tx, x_min),
            x_max,
        )

        start_y = -W / 2.0

        # --------------------------------------------------------------
        # 床置きなので validator では
        # effective_start_z = 0 になる。
        #
        # したがって搬入高さもtz。
        # --------------------------------------------------------------

        path_z = tz

        for other in packed_items:

            pos = other.get("pos")

            if pos is None:
                continue

            ox, oy, oz = self._to_local_pos(
                container,
                pos,
            )

            ohx = other["length"] / 2.0
            ohy = other["width"] / 2.0
            ohz = other["height"] / 2.0

            # Zが全く重ならないなら
            # 搬入中にもぶつからない。
            if (
                abs(path_z - oz)
                >=
                hz
                + ohz
                + self.safety_margin
            ):
                continue

            # ==========================================================
            # Phase 1:
            # Y方向
            #
            # X = start_x固定
            #
            # start_y -> target_y
            # ==========================================================

            y_lo = min(start_y, ty)
            y_hi = max(start_y, ty)

            path1_x_overlap = (
                abs(start_x - ox)
                <
                hx
                + ohx
                + self.safety_margin
            )

            other_y_lo = (
                oy
                - ohy
                - self.safety_margin
            )

            other_y_hi = (
                oy
                + ohy
                + self.safety_margin
            )

            path1_y_overlap = not (
                y_hi + hy < other_y_lo
                or
                y_lo - hy > other_y_hi
            )

            if (
                path1_x_overlap
                and path1_y_overlap
            ):
                return False

            # ==========================================================
            # Phase 2:
            # X方向
            #
            # Y = target_y固定
            #
            # start_x -> target_x
            # ==========================================================

            x_lo = min(start_x, tx)
            x_hi = max(start_x, tx)

            path2_y_overlap = (
                abs(ty - oy)
                <
                hy
                + ohy
                + self.safety_margin
            )

            other_x_lo = (
                ox
                - ohx
                - self.safety_margin
            )

            other_x_hi = (
                ox
                + ohx
                + self.safety_margin
            )

            path2_x_overlap = not (
                x_hi + hx < other_x_lo
                or
                x_lo - hx > other_x_hi
            )

            if (
                path2_x_overlap
                and path2_y_overlap
            ):
                return False

        return True

    # ------------------------------------------------------------------
    # coordinate conversion
    # ------------------------------------------------------------------

    def _to_local_pos(
        self,
        container,
        world_pos,
    ):
        """
        Container.global_to_local() と同じ変換。

        containers.pyでは
            local_x = global_x - offset_x
            local_y = global_y
            local_z = global_z

        observationにはoffset_xが直接無い場合があるため、
        centerから復元する。
        """

        cx, _, _ = container.get(
            "center",
            (0.0, 0.0, 0.0),
        )

        # container centerのXがoffset_x。
        offset_x = cx

        return (
            world_pos[0] - offset_x,
            world_pos[1],
            world_pos[2],
        )

    # ------------------------------------------------------------------
    # fallback
    # ------------------------------------------------------------------

    def _fallback(
        self,
        container=None,
        item=None,
    ):
        """
        本当に候補が無い場合の最終フォールバック。

        ここへ来る時点で、現在の単純1段方式では
        ほぼ詰め切っている想定。
        """

        if (
            container is not None
            and item is not None
        ):

            thickness = container.get(
                "thickness", 0.02
            )

            z = (
                thickness
                + item["height"] / 2.0
            )

            return {
                "item_idx": 0,
                "container_idx": 0,
                "place_pos": np.array(
                    [0.0, 0.0, z],
                    dtype=np.float32,
                ),
                "orientation": 0,
            }

        return {
            "item_idx": 0,
            "container_idx": 0,
            "place_pos": np.array(
                [0.0, 0.0, 0.1],
                dtype=np.float32,
            ),
            "orientation": 0,
        }