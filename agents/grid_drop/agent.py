import numpy as np

class Agent:
    def __init__(self, module_path: str):
        # 必要なパラメータの初期化
        self.module_path = module_path
        self.occupancy = {}  # container_idx -> 2D list (ny, nx) bool
        self.grid_info = {}  # container_idx -> dict with cell dimensions and origin
        self.initialized = False

    def get_init_states(self, init_states: dict):
        # コンテナの数や寸法など環境の初期設定を受け取り, エージェント内部に保持する.
        self.num_containers = len(init_states.get('container_list', []))
        # 初回は占有マップなどを初期化しない（policyで行う）
        return True

    def optimize(self, item_list: list):
        # オフライン最適化は使わないが, 受け取った順番をそのまま返す
        return [item['index'] for item in item_list]

    def policy(self, observation: dict):
        # 観測情報から配置場所を推論する処理を記述
        depth_maps = observation.get('depth_map')
        pool_list = observation.get('pool_list', [])
        container_list = observation.get('container_list', [])

        if not pool_list or not container_list:
            # 荷物がないかコンテナがない場合はダミーアクション
            return {
                'item_idx': 0,
                'container_idx': 0,
                'place_pos': np.array([0.0, 0.0, 0.0], dtype=np.float32),
                'orientation': 0
            }

        # 最初のコンテナを対象（単一コンテナ想定だが、複数でも最初を使う）
        container_idx = 0
        container = container_list[container_idx]
        # コンテナの内部寸法（厚さを引く）
        inner_length = container['length'] - 2 * container['thickness']
        inner_width = container['width'] - 2 * container['thickness']
        inner_height = container['height'] - 2 * container['thickness']  # 高さも内部か？

        # プールから最初の荷物を選ぶ（item_idx=0）
        item_idx = 0
        item = pool_list[item_idx]
        item_l = item['length']
        item_w = item['width']
        item_h = item['height']

        # 初回またはコンテナ情報が変わったらグリッド情報を再計算
        if container_idx not in self.grid_info:
            # グリッドセルサイズ：荷物の寸法に少し余裕を持たせる
            # ここでは全荷物のうち最大サイズを使う（プールから取得できる範囲で）
            max_l = max((it['length'] for it in pool_list), default=item_l)
            max_w = max((it['width'] for it in pool_list), default=item_w)
            cell_x = max_l + 0.02  # X方向セル幅
            cell_y = max_w + 0.02  # Y方向セル幅

            # セル数
            nx = max(1, int(inner_length // cell_x))
            ny = max(1, int(inner_width // cell_y))
            # セルサイズを実際に調整（余分を均等に分配）
            actual_cell_x = inner_length / nx if nx > 0 else inner_length
            actual_cell_y = inner_width / ny if ny > 0 else inner_width

            # 原点（コンテナ内部左奥前隅）をコンテナ中心からのオフセットで計算
            # コンテナの中心は (0,0,0) ローカル（offset_x,0,0 は環境側で加算される）
            # したがって、左奥前隅は (-inner_length/2, -inner_width/2, -inner_height/2)
            origin_x = -inner_length / 2.0
            origin_y = -inner_width / 2.0
            # Zは底面（内部底面）からのオフセット：-inner_height/2
            origin_z = -inner_height / 2.0

            self.grid_info[container_idx] = {
                'nx': nx,
                'ny': ny,
                'cell_x': actual_cell_x,
                'cell_y': actual_cell_y,
                'origin_x': origin_x,
                'origin_y': origin_y,
                'origin_z': origin_z
            }
            # 占有マップ初期化（False = 空）
            self.occupancy[container_idx] = [[False for _ in range(nx)] for _ in range(ny)]
            # Debug print
            print(f"[DEBUG] Container {container_idx}: inner LxWxH={inner_length:.3f}x{inner_width:.3f}x{inner_height:.3f}")
            print(f"[DEBUG] Grid {nx}x{ny}, cell size {actual_cell_x:.3f}x{actual_cell_y:.3f}")
            print(f"[DEBUG] Origin ({origin_x:.3f},{origin_y:.3f},{origin_z:.3f})")

        info = self.grid_info[container_idx]
        occ = self.occupancy[container_idx]

        # グリッドを奥から手前（Yの大きい方から小さい方）および左から右（Xの小さい方から大きい方）で探索
        placed = False
        place_x = place_y = 0.0
        for iy in range(info['ny'] - 1, -1, -1):  # 後ろ(大きいY)から前(小さいY)
            for ix in range(info['nx']):  # 左(小さいX)から右(大きいX)
                if not occ[iy][ix]:
                    # セルの中心位置を計算
                    cx = info['origin_x'] + (ix + 0.5) * info['cell_x']
                    cy = info['origin_y'] + (iy + 0.5) * info['cell_y']
                    # Z座標：少し上から落とす（コンテナ内部上部＋余裕）
                    # コンテナ内部トップは inner_height/2
                    cz = info['origin_z'] + inner_height / 2.0 + 0.1  # 上から0.1m上
                    place_x, place_y, place_z = cx, cy, cz
                    # 占有マーク
                    occ[iy][ix] = True
                    placed = True
                    # Debug print first placement
                    print(f"[DEBUG] Placing item {item['index']} at grid ({iy},{ix}) -> pos ({cx:.3f},{cy:.3f},{cz:.3f})")
                    break
            if placed:
                break

        if not placed:
            # グリッドが埋まった場合はダミーアクション
            place_x = place_y = place_z = 0.0
            print("[DEBUG] Grid full, no placement")

        action = {
            'item_idx': item_idx,
            'container_idx': container_idx,
            'place_pos': np.array([place_x, place_y, place_z], dtype=np.float32),
            'orientation': 0  # 回転なし
        }
        # Debug print action
        print(f"[DEBUG] Action: {action}")
        return action