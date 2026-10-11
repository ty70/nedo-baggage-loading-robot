# cpp_aabb_checker v0.2 — Phase 2

Phase 1 の AABB / inclusion checker に、v17 と同じ考え方の **Y -> X 搬入経路チェック**を追加した版です。

## Install and Run

cd cpp_aabb_checker_v02

source ../simulator/.venv/bin/activate

uv pip install pybind11 setuptools wheel

python setup.py build_ext --inplace

python test_checker.py

## 追加API

### `check_transport_path(...)`

開始位置 `(start_x, start_y, path_z)` から

1. `start_x` 固定で `Y` 方向へ `target_y` まで移動
2. `target_y` 固定で `X` 方向へ `target_x` まで移動

し、各 `step_len` ごとに既配置BoxとのAABB衝突を確認します。

デフォルト `step_len=0.01`（1 cm）です。

衝突時の戻り値例:

```python
{
    "ok": False,
    "segment": "Y",
    "step_index": 12,
    "segment_step_index": 12,
    "collision_ids": [32],
    "collision_position": (-0.30, -0.605, 0.40),
    "checked_steps": 13,
}
```

### `check_transport_like_safe_grid(...)`

`safe_grid_v17._path_clear()` に近いラッパーです。

- `cut_x` を考慮して `start_x` をclip
- 開始Yを開口部側に設定
- floor targetなら target_z のまま
- stack targetなら原則 `target_z + 0.08 + extra_lift`
- その後 `Y -> X` sweep

Phase 2 では **packed baggageのみ** を障害物にしています。main shelf / small shelf 自体のAABBは Phase 3 で追加します。

## Build

```bash
cd cpp_aabb_checker_v02
source ../simulator/.venv/bin/activate   # 実環境に合わせて変更
uv pip install pybind11 setuptools wheel
python setup.py build_ext --inplace
python test_checker.py
```

期待出力:

```text
all Phase 1 + Phase 2 tests passed
```

## Python usage

```python
from cpp_aabb_checker import Box, ContainerBounds, check_transport_like_safe_grid

result = check_transport_like_safe_grid(
    item,
    container,
    packed,
    target_x=0.50,
    target_y=0.30,
    target_z=0.80,
    cut_x=0.44,
    thickness=0.04,
    start_margin=0.018,
    safety_margin=0.015,
    step_len=0.01,
    collision_gap=0.015,
)

if not result["ok"]:
    print(result["segment"], result["collision_ids"], result["collision_position"])
```

## Phase 2 の意図

これはPyBulletを置き換える最終判定器ではなく、候補生成時の高速pre-checkです。

特にv17の000/001で起きている「入口付近で後続荷物が既配置荷物にぶつかる」候補を、PyBulletを動かす前に除外するための基盤です。
