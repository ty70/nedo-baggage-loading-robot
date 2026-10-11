# cpp_aabb_checker_v03

Phase 1/2 に加えて、Phase 3 として **small shelf / main shelf** を C++ 側で扱います。

## Install & Run

python setup.py build_ext --inplace

python test_checker.py

## Phase 3 の追加機能

- `make_small_shelf(...)`
- `make_main_shelf(...)`
- `check_shelf_support(...)`
- `check_candidate_with_shelves(...)`
- `check_transport_path_with_shelves(...)`
- `check_transport_like_safe_grid_with_shelves(...)`

固定棚は `ShelfGeometry` として表現します。

- small shelf ID: `-1001`
- main shelf ID: `-1002`

搬入衝突時は `shelf_collision_ids` に棚IDを返します。

## 棚ジオメトリ

### small shelf

v17 と同じローカル座標解釈を採用しています。

```text
lx = cut_x
ly = width - 2*thickness
lz = thickness
cx = -length/2 + thickness + cut_x/2
cy = 0
cz = height/2 + buffer + thickness/2
```

small shelf の支持判定では、外壁側（X負側）へのオーバーハングは禁止です。
内側への小さなオーバーハングのみ許可できます。

### main shelf

```text
lx = length - thickness
ly = width/2 - 2*thickness
lz = thickness
cx = 0
cy = width/4
cz = height/2 + buffer + thickness/2
```

## ビルド

```bash
cd cpp_aabb_checker_v03
source ../../.venv/bin/activate   # パスは環境に合わせる
uv pip install pybind11 setuptools wheel
python setup.py build_ext --inplace
python test_checker.py
```

期待値:

```text
all Phase 1 + Phase 2 + Phase 3 tests passed
```

速度測定:

```bash
python benchmark.py
```

## 注意

Phase 3 はまだ剛体AABBベースです。PyBulletの接触反力、settling、soft baggage変形は再現しません。
棚上配置については `check_shelf_support()` と、棚そのものとの衝突判定を分離しています。

この分離は重要です。棚上の荷物は棚に「接触してよい」一方、搬入途中で棚板を横切るのは「衝突」です。
