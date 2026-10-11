2026/09/04T16:40

* cpp_aabb_checker_v04 との連携

# safe_grid_v18

## 導入方法

mkdir -p agents/safe_grid_v18
cp ~/Downloads/agent.py agents/safe_grid_v18/agent.py

## 起動

SAFE_GRID_CPP_MODE=shadow \
python -m scripts.run_test \
  --module-path agents/safe_grid_v18/ \
  --config-path configs/sample_config.json \
  --verbose True \
  --render-mode None

## 次の課題

1. `_path_clear()`だけC++ authority化
2. `_final_clear()`のmismatch原因を修正
3. mismatchを再度0件まで確認
4. その後finalもC++化

# safe_grid_v19 — Phase 5.1

This revision preserves safe_grid_v18 and changes only the C++ integration policy.

## Layout

Expected project layout:

```text
simulator/
├─ agents/
│  └─ safe_grid_v19/
│     └─ agent.py
└─ scripts/
   └─ cpp_aabb_checker_v04/   # or cpp_aabb_checker_v4
      └─ cpp_aabb_checker*.so
```

## Changes from v18

- `_path_clear()` is now C++ authoritative when the extension is available.
- The Python path checker is not run in parallel, reducing duplicate work.
- `_final_clear()` remains Python authoritative in `shadow` mode.
- When final placement disagrees, the log now includes the Python collision item ID, position, dimensions, axis deltas and comparison limits, plus the C++ collision IDs.
- If the C++ module cannot be imported, path checking safely falls back to the original Python implementation.

## Recommended run

```bash
SAFE_GRID_CPP_MODE=shadow \
python -m scripts.run_test \
  --module-path agents/safe_grid_v19/ \
  --config-path configs/sample_config.json \
  --verbose True \
  --render-mode None \
  --result-dir results/safe_grid_v19/ \
  --result-fname evaluation_results_safe_grid_v19.json

```

Expected startup:

```text
[safe_grid_v19][cpp] loaded mode=shadow path=.../scripts/cpp_aabb_checker_v04
```

Summary example:

```text
[safe_grid_v19][cpp-fallback] final=.../... (...) mismatch=... path_cpp_authority=...
```

For mismatch diagnosis, look for:

```text
[safe_grid_v19][cpp-mismatch] kind=final ... detail={'python_collision': {'id': ...}, 'cpp_collision_ids': [...]}
```

Do not use `SAFE_GRID_CPP_MODE=cpp` yet. That would make final placement C++ authoritative before the mismatch cause has been eliminated.

### 次の課題

次は safe_grid_v20 を新規作成し、

①17桁precisionで境界差を確定 → ②C++ final判定を修正 → ③shadowで100%一致を狙う、の順で進めるのがよい。

# safe_grid_v20 — Phase 5.2

Purpose: diagnose the remaining _final_clear() Python/C++ mismatches at full floating-point precision before making C++ authoritative for final placement.

Layout

simulator/
├─ agents/
│  └─ safe_grid_v20/
│     └─ agent.py
└─ scripts/
   └─ cpp_aabb_checker_v04/
      └─ cpp_aabb_checker*.so

Behavior

_path_clear(): C++ authoritative, same as v19.

_final_clear(): Python authoritative + C++ shadow comparison.

On final mismatch, logs include 17-digit values for:

Python collision item id/center/dimensions

delta and limit for X/Y/Z

delta_minus_limit

per-axis < result (axis_lt)

C++ moving/packed centers and dimensions actually passed to aabb_overlap

C++ delta/limits and per-axis <

C++ aabb_overlap result

Interpretation:

delta_minus_limit < 0: overlap on that axis under strict <.

delta_minus_limit == 0: exactly on the boundary.

delta_minus_limit > 0: separated on that axis.

Run

SAFE_GRID_CPP_MODE=shadow \
python -m scripts.run_test \
  --module-path agents/safe_grid_v20/ \
  --config-path configs/sample_config.json \
  --verbose True \
  --render-mode None \
  --result-fname evaluation_results_safe_grid_v20.json \
  --result-dir results/safe_grid_v20/ \
  2>&1 | tee ./results/safe_grid_v20/safe_grid_v20.log

The first 5–10 [safe_grid_v20][cpp-mismatch] kind=final lines from case001 are enough for the next diagnosis.

次の課題

1. final AABB専用関数をC++に追加
2. Pythonと同じ演算順序にする
3. shadowでfinal mismatch 0件を確認
4. その後finalをC++ authorityへ切り替える

# safe_grid_v21

cpp_aabb_checker_v05 と safe_grid_v21を新規作成。

箱を先に膨張
→ (a_dim + b_dim) / 2

をやめて、Python _final_clear()と同じ順序で直接、

abs(dx) < a.lx*0.5 + b.lx*0.5 + margin_xy
abs(dy) < a.ly*0.5 + b.ly*0.5 + margin_xy
abs(dz) < a.lz*0.5 + b.lz*0.5 + margin_z

を評価。

## 導入
cd ~/simulator/scripts/cpp_aabb_checker_v05

uv pip install pybind11 setuptools wheel
python setup.py build_ext --inplace
python test_checker.py

cd ~/simulator
FILE_NAME=safe_grid_v21
mkdir -p ./results/$FILE_NAME/
SAFE_GRID_CPP_MODE=shadow \
python -m scripts.run_test \
  --module-path agents/$FILE_NAME/ \
  --config-path configs/sample_config.json \
  --verbose True \
  --render-mode None \
  --result-dir results/$FILE_NAME/ \
  --result-fname evaluation_results_$FILE_NAME.json \
  2>&1 | tee ./results/$FILE_NAME/$FILE_NAME.log

## ファイル量はちょっと減ったね

path_clear  → すでにC++ authority
final_clear → shadow比較で大量の候補を100%一致

## いままで
Python final判定
+
C++ final判定
+
比較・統計

## 次の課題では
C++ final判定だけ

# safe_grid_v22

## 実行手順

cd ~/simulator
FILE_NAME=safe_grid_v22
mkdir -p ./results/$FILE_NAME/
SAFE_GRID_CPP_MODE=shadow \
python -m scripts.run_test \
  --module-path agents/$FILE_NAME/ \
  --config-path configs/sample_config.json \
  --verbose True \
  --render-mode None \
  --result-dir results/$FILE_NAME/ \
  --result-fname evaluation_results_$FILE_NAME.json \
  2>&1 | tee ./results/$FILE_NAME/$FILE_NAME.log

  * 3D-BPPの導入とその問題点

1. 「幾何学的に入る」と「ロボットが実際に運べる」が違う
    奥側優先
    入口から遠い場所を先に使用
    搬入通路を潰す候補にペナルティ

  EP法 + transport-aware pruning

2. Extreme Pointの候補数が爆発する

3D-BPPは強いNP-hard問題であり、厳密解を直接求めるのは実用上非常に難しいため、候補削減とヒューリスティックが重要になります。

3. 高い充填率を狙うほど、安定性・重心・soft/priorityのスコアと衝突する

克服方法

「一番詰まる候補」を選ぶのではなく、多目的スコアにします。

Score =
 + 容積効率
 + 接触面積
 + 奥側配置
 + 低いZ
 + 支持率

 - 搬入口閉塞
 - 重心上昇
 - overhang
 - soft下積み
 - heavy-on-light
 - priority埋没

「禁止条件」と「評価条件」を分ける

禁止
----------------
container outside
AABB collision
transport collision
heavy-on-light大違反
support不足

評価
----------------
低い
奥
隙間が少ない
支持面積が広い
入口を塞がない

4. 「Extreme Pointは理想的な箱を前提としているが、PyBulletでは置いた後に少し動く」

EP法では、

Box A top = z=0.500
↓
次の箱
z=0.500

という厳密な幾何を作りがちです。

ところがPyBulletでは、

0.500
→ 0.498
→ 少し傾く

ことがあります。

すると、その後のEP全体が少しずつ狂います。

ここは、

計画上のpose

ではなく、

PyBulletでsettle後に観測したpose

からExtreme Pointを再生成する必要があります。

つまり、

EPを計算
↓
1個配置
↓
PyBullet settle
↓
actual pose取得
↓
EP集合を更新

です。

これをしないと20個、30個積んだころに誤差が蓄積します。

## 今回導入すべき全体構造

私は次の構造が一番よいと思います。

                incoming items
                      │
                      ▼
              Extreme Point生成
                      │
              ┌───────┴────────┐
              │ orientations ×6│
              └───────┬────────┘
                      ▼
            C++ fast feasibility
        ┌─────────────┼─────────────┐
        │             │             │
     inside         AABB         support
        │             │             │
        └─────────────┼─────────────┘
                      ▼
                transport path
                      │
               feasible candidates
                      │
                      ▼
                scoring/ranking
        ┌─────────────┼──────────────┐
        │             │              │
      fill          CoG         entrance
    efficiency     stability      reserve
        └─────────────┼──────────────┘
                      ▼
                best candidate
                      │
                      ▼
                   PyBullet
                      │
                      ▼
                 actual pose
                      │
                      └────→ EP更新

EP = 良い候補点を作る仕組み
C++ = 候補を高速に捨てる仕組み
scoring = コンテスト向けに候補を選ぶ仕組み
PyBullet = 最終的な物理世界                  

私なら導入順はこうします

1. EP v1
箱の +X,+Y,+Z 面からEP生成
duplicate除去
6 orientations
C++ final/path checker

2. EP v2
projected Extreme Points
support判定
EP dominance pruning

3. EP v3
entrance reservation
far-to-near score
CoG / mass score

4. EP v4
soft/priority
actual settled poseからEP再構築

## 次の実装ではまず safe_grid_v23 ではなく、アルゴリズム系統を分けて extreme_point_v01 として新しいagentを作る

