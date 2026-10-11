# NEDO Challenge Baggage-Loading Robot コンテスト2
## 直近議論の引き継ぎコンテキスト

### 1. 現在の目的

PyBullet上のNEDO Challenge Baggage-Loading Robot コンテスト2について、
`search_cand_v6` と `decide_position_v2` を中心に、
「計画上は配置可能と判断された荷物が、実行時に `not included` / `not inside` になる問題」を解消する。

現在の最重要課題は、単に探索で置ける配置を増やすことではなく、

> **最終的に実機・評価環境の包含判定を通る配置を、計画段階で確実に選ぶこと**

である。

---

## 2. これまでの方針

### search_cand系

`search_cand_v5` までで、候補探索・BPP・挿入順序・支持関係などを改善してきた。

v5では探索結果として、

- `safe_placed`
- `safe_volume`
- `geom_placed`
- `geom_volume`

などを使って候補を比較する。

また、BPPによって配置順序を作り、

- support
- route
- precedence

などの関係を考慮して実行順序を決めている。

### 重要な判断

以前、計画フェーズから「5について警告」が出ていたため、

> **5にはまだ手を付けず、1〜4までの修正をv6へ反映し、その結果を見てから5を考える**

という方針にした。

---

## 3. search_cand_v5で確認された重要な問題

v5の実行ログでは、探索自体はかなり良い結果を出している。

例：

```text
[search_cand_v5][SEARCH-BEST]
strategy=seed_20260908/beam1
safe_placed=15
safe_volume=1.136025
geom_placed=35
geom_volume=2.238025
```

BPP最適化後も、

```text
[search_cand_v5][BPP-DUMP]
task=000 stage=optimize planned=15
```

となっている。

さらに挿入計画では15個について、

```text
[search_cand_v5][INSERT-GRAPH]
...
cycles=[]
```

であり、計画上は循環もない。

実際に15個を順番に搬送・配置している。

---

## 4. しかし評価では重大な問題が発生

実行ログの後半：

```text
transport item 3
place item 3
...
transport item 0
place item 0
```

までは進む。

しかし、その後、

```text
not included.
[-0.6400000071525573, 3.015000095367432, -0.6399999928474427,
 -4.285000095367431, -3.076493124028456, -0.5, -0.45999999999999996]
```

が発生。

さらに、

```text
item 21 not inside (hit boundary plane)
item 17 not inside (hit boundary plane)
item 24 not inside (hit boundary plane)
item 35 not inside (hit boundary plane)
item 23 not inside (hit boundary plane)
item 4 not inside (hit boundary plane)
item 29 not inside (hit boundary plane)
```

となった。

最終評価：

```text
evaluation {
    'fill_score': 15.088061420881978,
    'num_placed_items': 0.36585365853658536
}

place_states {
    'is_included': False,
    'is_valid': False,
    'is_placed_safe': False
}
```

つまり、

> **探索・BPP・配置計画では成功しているように見えるが、評価側の最終包含判定では失敗している。**

ここが現在の最大の問題。

---

# 5. `not included` と `not inside` の意味

ログから重要なのは、

```text
not included
```

と、

```text
item XX not inside (hit boundary plane)
```

が別レイヤーの問題として存在している可能性が高いこと。

特に、

```text
item 21 not inside (hit boundary plane)
```

のように明示的に

> `hit boundary plane`

と出ているため、

- 物体が大きく飛び出している
- 完全に別の場所へ行っている

というより、

> **境界面ギリギリ、または境界面を越えた状態**

で評価されている可能性が高い。

したがって、単純に「もっと内側へ置く」だけではなく、

- planner側の座標
- execution側の座標
- orientation
- half extent
- boundary margin
- inclusion判定
- PyBulletの実際の姿勢
- settle後の姿勢

の整合性を調べる必要がある。

---

# 6. v5ログから見える具体的な違和感

v5では、

```text
boundary_margin=0.015
xy_gap=0.020
ep_gap=0.020
inclusion_margin=-0.005
```

となっている。

特に、

```text
inclusion_margin=-0.005
```

は注意が必要。

名前だけを見ると、

> 包含判定に5mmの余裕を持たせる

ようにも見えるが、実際の符号の意味はコードを確認しないと断定できない。

また、

```text
[PACK]
final_target=(...)
```

と、

```text
[INSERT]
execution_target=(...)
```

が一致しないケースがある。

例えば item 3：

```text
final_target=(-0.670,0.295,1.128)
execution_target=(-0.670,0.295,1.149)
shelf_lift=0.021
```

item 8も、

```text
final_target=(-0.660,-0.325,1.173)
execution_target=(-0.660,-0.325,1.194)
shelf_lift=0.021
```

となっている。

これは小さい差だが、

> **計画時の包含判定対象座標と、実際にplaceする座標が違う**

ことを意味するため、非常に重要。

---

# 7. item 21などの「boundary plane」問題

item 21：

```text
[PACK]
item=21
final_target=(0.670,0.390,0.430)

[INSERT]
item=21
execution_target=(0.670,0.390,0.430)
```

それでも、

```text
item 21 not inside (hit boundary plane)
```

となっている。

つまり、

> final_targetとexecution_targetが同じでも、
> 評価時の実際の物体姿勢・AABB・境界判定では失敗している。

可能性がある。

したがって、v6以降では

**「配置点が箱の中心にあるか」だけではなく、
「その姿勢・寸法を含む実際のAABBがコンテナ内部に完全に収まるか」**

を最終判定と同じ定義で確認する必要がある。

---

# 8. decide_position_v1 / v2について

`decide_position_v1` は配置位置決定を担当する。

その後、

> `decide_position_v2`

を作成する方針になった。

ただしv2については、

- v1のコードをベースにする
- 既存の処理を勝手に捨てない
- 必要な部分だけ改変する
- `bpp_layout.json` を生成・利用する

という方針。

---

# 9. bpp_layout.jsonについて

途中で、

> `bpp_layout.json` がない

という問題が発生した。

これは重要。

`decide_position_v1` ではBPP結果をJSONとして出力する仕組みが存在していたため、

> **v2でも同様に `bpp_layout.json` を生成する必要がある。**

単に内部メモリ上で配置情報を持つだけではなく、

```text
bpp_layout.json
```

として後段処理から利用可能な形にする。

ファイルの構造・キー名については、既存の`decide_position_v1`の実装を基準にする。

---

# 10. Codexへの作業方針

Codexには、

1. 既存コードを読む
2. ログを読む
3. `not included` / `not inside` の発生箇所を特定
4. plannerとevaluationの包含判定の定義差を調査
5. `decide_position_v2` を作成
6. `bpp_layout.json` を必ず生成
7. 実行して結果を出す
8. 変更点と結果を報告

という流れで作業させる。

重要なのは、

> **いきなり大規模なアルゴリズム変更をしないこと。**

まず、

- 座標系
- AABB
- orientation
- margin
- boundary判定
- execution_target
- final_target
- settle後の位置

のどこで食い違っているかを特定する。

---

# 11. 現在の優先順位

### 最優先：not included解消

現在の最大のボトルネック。

`safe_placed=15` でも評価では

```text
num_placed_items=0.36585365853658536
```

まで落ちている。

したがって、

> 探索数を増やすより、まず評価側で確実にincludedになる配置を作る。

ことが優先。

### 次点：decide_position_v2

v1の配置決定処理をベースに、

- 実際の評価座標との整合
- 境界余裕
- AABB完全包含
- BPP結果との一致

を強化する。

### その後：search_cand_v6

v6では、

- v5までの改善を維持
- decide_position_v2と接続
- 実行時の包含判定を重視
- 5番目の計画フェーズにはまだ手を付けない

という方針。

---

# 12. 現時点での重要な仮説

## 仮説A：plannerとevaluationで包含判定が違う

最も重要。

plannerでは、

```text
safe
```

と判断しているが、evaluationでは

```text
not included
```

になっている。

つまり、

```text
planner inclusion
        ↓
execution
        ↓
PyBullet actual pose
        ↓
evaluation inclusion
```

のどこかで定義が一致していない。

---

## 仮説B：boundary marginが不足

`boundary_margin=0.015` でも、

```text
hit boundary plane
```

が出ている。

15mmでは不足している可能性がある。

ただし、いきなりmarginを大きくするのではなく、

> **評価側の境界条件を確認してから調整する。**

---

## 仮説C：orientation / AABB計算の差

特に、

```text
item 21
item 17
item 24
item 35
item 23
item 4
item 29
```

が同じようにboundaryで失敗している。

これらに共通するorientationや寸法があるか確認する。

---

## 仮説D：settle後に物体が微妙に動いている

計画時：

```text
execution_target
```

ではOK。

しかし、

```text
place
→ physics settle
→ evaluation
```

で微妙にずれる可能性がある。

この場合、

- settle前座標
- settle後座標
- AABB
- orientation

をログに出す必要がある。

---

# 13. やってはいけないこと

- v5を直接破壊しない
- 既存の探索ロジックを全部作り直さない
- 5番目の計画フェーズにまだ手を付けない
- `not included`を単純に無視しない
- `safe_placed`だけを成功指標にしない
- BPP結果を消さない
- `bpp_layout.json`を省略しない
- 「15個置けたから成功」と判断しない

---

# 14. 最終的な目標

最終的には、

```text
search_cand_v6
        ↓
候補探索
        ↓
BPP
        ↓
decide_position_v2
        ↓
実際に実行可能な配置位置
        ↓
transport / place
        ↓
settle
        ↓
evaluation
        ↓
included=True
```

という一貫したパイプラインにする。

重要なのは、

> **「計画上置ける」ではなく「評価環境で実際にincludedになる」こと。**

これをv6の設計基準とする。
