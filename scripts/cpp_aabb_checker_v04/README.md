# cpp_aabb_checker_v04

Phase 1〜3 に加え、Phase 4 として **荷物上への stack support / overhang 判定**を C++ 化します。

## Install & Run

python setup.py build_ext --inplace

python test_checker.py

## Phase 4 の追加API

- `check_stack_support(upper, support, ...)`
- `find_stack_supports(upper, packed, ...)`
- `check_stack_candidate(upper, support, container, packed, shelves, ...)`

`Box` には `mass` と `is_soft` を追加しました。Phase 4 では `mass` を利用し、v17 と同様に「上の荷物が支持荷物より重くならない」制約を任意で有効化できます。`is_soft` は次段階の soft baggage ルール用に保持していますが、v04 ではまだ物理変形は扱いません。

## 支持判定

単一の支持荷物に対して次を計算します。

1. 上荷物の底面と支持荷物上面の Z 差
2. XY の重なり幅と重なり面積
3. 上荷物底面に対する support ratio
4. 左右・前後それぞれの overhang
5. 質量順序（オプション）

デフォルト値:

```text
min_support_ratio = 0.90
max_overhang      = 0.015 m
z_tolerance       = 0.03 m
enforce_mass      = True
weight_tolerance  = 0.05 kg
```

`max_overhang` と `min_support_ratio` は独立条件です。面積比が十分でも片側だけ大きく飛び出す候補は reject できます。

## ビルド・テスト

```bash
cd ~/simulator/scripts/cpp_aabb_checker_v04
python setup.py build_ext --inplace
python test_checker.py
```

期待値:

```text
all Phase 1 + Phase 2 + Phase 3 + Phase 4 tests passed
```

速度測定:

```bash
python benchmark.py
```

従来の transport benchmark に加えて、100万回の `check_stack_support()` も測定します。

## 注意

v04 は **単一の箱が1個の支持箱に載る** ケースを扱います。2個以上の荷物に橋渡しして載る multi-support の支持面積unionはまだ実装していません。また soft baggage の変形・沈み込みはPyBulletに残します。
