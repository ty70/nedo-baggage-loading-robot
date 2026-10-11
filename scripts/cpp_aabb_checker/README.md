# cpp_aabb_checker v0.1

最初の段階として、PyBulletを起動せずに候補配置を高速にふるい落とすための軽量AABBチェッカです。

## 実装済み

- 6種類のaxis-aligned orientationの寸法変換
- コンテナ内包判定
- 荷物同士のAABB衝突判定
- 候補1個を既配置荷物すべてに対して検査する `check_candidate`

まだ未実装:

- Y→X搬入経路判定
- small shelf / main shelfとの衝突
- support / overhang
- settling / soft-item deformation

## WSL / Ubuntuでのビルド

```bash
cd cpp_aabb_checker
python -m pip install pybind11 setuptools wheel
python setup.py build_ext --inplace
python test_checker.py
```

uv環境なら:

```bash
uv pip install pybind11 setuptools wheel
python setup.py build_ext --inplace
python test_checker.py
```

成功すると:

```text
all tests passed
```

## Python使用例

```python
from cpp_aabb_checker import Box, ContainerBounds, check_candidate

c = ContainerBounds(-0.96, 0.96, -0.685, 0.685, 0.04, 1.57)

item = Box()
item.id = 10
item.cx, item.cy, item.cz = 0.1, 0.2, 0.3
item.lx, item.ly, item.lz = 0.55, 0.24, 0.40

result = check_candidate(
    item,
    c,
    packed=[],
    inclusion_margin=0.005,
    collision_gap=0.015,
)
print(result)
```

## v17へ統合する前の方針

まずPython版 `_final_clear()` と同じ候補を1000件程度入力し、C++版との一致率を測ります。
False Positive（C++=safe / PythonまたはPyBullet=unsafe）を優先的にゼロへ近づけてから、v17/v18の候補探索へ組み込みます。
