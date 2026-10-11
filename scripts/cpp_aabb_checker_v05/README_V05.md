# cpp_aabb_checker_v05

v04 を残したまま作る Phase 5.3 用 checker。

## 変更点

`aabb_overlap_python_order(a, b, margin_xy, margin_z)` を追加。

旧方式は moving box を先に膨らませ、その後 `(a_dim + b_dim) / 2` を計算していたため、境界で約 5e-17 m の丸め差が発生した。

v05 は safe_grid Python `_final_clear()` と同じ演算順序:

```text
abs(dx) < a.lx*0.5 + b.lx*0.5 + margin_xy
abs(dy) < a.ly*0.5 + b.ly*0.5 + margin_xy
abs(dz) < a.lz*0.5 + b.lz*0.5 + margin_z
```

を C++ 内で直接評価する。

## 配置

```text
simulator/
  agents/
    safe_grid_v21/
      agent.py
  scripts/
    cpp_aabb_checker_v05/
      checker.cpp
      setup.py
      test_checker.py
```

## build

```bash
cd ~/simulator/scripts/cpp_aabb_checker_v05
uv pip install pybind11 setuptools wheel
python setup.py build_ext --inplace
python test_checker.py
```

期待値:

```text
all Phase 1 + Phase 2 + Phase 3 + Phase 4 + Phase 5 tests passed
```
