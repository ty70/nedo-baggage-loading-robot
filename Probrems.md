# NEDO Challenge Baggage-Loading Robot ― 引き継ぎコンテキスト

## 1. プロジェクトの目的

PyBulletベースのNEDO Challenge Baggage-Loading Robotコンテスト用シミュレータで、
荷物をコンテナ内へ安全に搬送・配置する探索エージェントを改善している。

現在の主な問題は、

- BPP（3D Bin Packing）では配置計画が作れている
- transport / place 自体も実行できている
- しかし最終評価で `not included` / `not inside (hit boundary plane)` が発生する
- そのため、BPP上では配置できているのに、実際の評価環境では「コンテナ内に入っていない」と判定される

という問題。

---

## 2. これまでのバージョン

### search_cand_v5

`search_cand_v5` は `search_cand_v4` をベースに改善した探索コード。

主な探索設定：

```text
boundary_margin=0.015
xy_gap=0.020
ep_gap=0.020
inclusion_margin=-0.005
cpp_mode=cpp
search_trials=16
search_budget=60.0s
bpp_beam=4
bpp_branch=3
feasibility_beam=12
feasibility_branch=8

C++ AABB checker：

/home/ty70/simulator/scripts/cpp_aabb_checker_v05

結果ファイル：

/home/ty70/simulator/results/search_cand_v5/bpp_layout_search_cand_v5.json

3. search_cand_v5 の探索結果

16 trial を実行。

特に良かったのは：

trial=12
strategy=seed_20260908
geom_placed=35
geom_volume=2.238025
safe_placed=15
safe_volume=1.136025

最終BEST：

[SEARCH-BEST]
strategy=seed_20260908/beam1
safe_placed=15
safe_volume=1.136025
geom_placed=35
geom_volume=2.238025
layouts=64
elapsed=34.709s

つまり探索/BPP側ではかなり改善しており、
15個のsafe placementを作れている。

4. しかし実行時に問題発生

BEST BPP order：

[3, 8, 21, 17, 33, 24, 35, 20, 23, 5, 32, 4, 18, 29, 0, ...]

最初の15個は実際に

transport item
place item

まで正常に実行された。

ところが、その後評価環境から：

not included.
[-0.6400000071525573, 3.015000095367432, -0.6399999928474427,
 -4.285000095367431, -3.076493124028456, -0.5,
 -0.45999999999999996]

さらに：

item 21 not inside (hit boundary plane)
item 17 not inside (hit boundary plane)
item 24 not inside (hit boundary plane)
item 35 not inside (hit boundary plane)
item 23 not inside (hit boundary plane)
item 4 not inside (hit boundary plane)
item 29 not inside (hit boundary plane)

が発生した。

最終評価：

evaluation {
    'fill_score': 15.088061420881978,
    'num_placed_items': 0.36585365853658536
}

place_states {
    'is_included': False,
    'is_valid': False,
    'is_placed_safe': False
}

message ok
status success

重要なのは、最適化そのものは成功しているが、実配置の評価で「inside」が落ちていること。

5. 重要な観察

search_cand_v5 のログでは、

[INSERT] step=3 item=21
execution_target=(0.670,0.390,0.430)
path_ok=True
shelf_lift=0.000
forced=False
precedence_bypass=False
support_runtime={
    'ok': True,
    'kind': 'floor',
    'ratio': 1.0,
    'source': 'floor'
}

など、配置時の内部判定は ok=True になっている。

例えば item 33：

support_runtime={
    'ok': True,
    'kind': 'box',
    'ratio': 0.9895904385972129,
    'source': 'settled',
    'reason': 'box-footprint',
    'support_item': 17
}

item 20：

ratio=0.99962684952609

item 5：

ratio=0.9982504946206666

item 32：

ratio=0.9986456644421805

item 18：

ratio=0.9999998256236181

item 0：

ratio=0.9979355083594913

つまり、内部のsupport判定ではほぼ完全に載っているのに、最終評価のinside判定ではboundary planeに当たっている。

ここが現在の核心問題。

6. 直近の方針

以前の議論では、計画フェーズの「5」に警告が出ていた。

そのため、

5の計画フェーズには今は手をつけない。
まず4までの修正をv6に反映して、それから5を考える。

という方針を採用。

つまり現在は、探索アルゴリズムを大きく変更するより、

BPP → target position → execution → simulator inclusion判定

の間にある座標/境界/マージンの不整合を潰すことを優先する。

7. search_cand_v6

search_cand_v6 は v5 をベースに作成する。

重要な方針：

v5を直接変更しない
v5の探索性能をなるべく維持
BPPの配置結果そのものをむやみに変更しない
not include 問題を優先的に修正
実行時 target position と simulator の inclusion 判定の整合性を確認する
「内部ではsafeなのに外部評価ではnot included」という二重判定問題を解消する

特に、

boundary_margin
inclusion_margin

などの意味を混同しないこと。

8. decide_position_v1 / v2

BPPで決定した配置を、実際の搬送・配置位置へ落とし込む処理も検討している。

decide_position_v1 には、

bpp_layout.json

があり、これを使ってBPP結果を固定的に再利用する構成になっている。

decide_position_v2 も作成している。

ユーザーから、

v1のようにbpp_layout.jsonを作って

という要求があり、v1形式の bpp_layout.json をv2にも持たせる方針。

9. bpp_layout.json の重要性

bpp_layout.json は単なるログではなく、

BPPで決めた最終配置計画を、decide_position側で再利用するための固定データ

として扱う。

v1の形式を踏襲する。

概念的には：

000:
  optimize時のBPP結果

001:
  rolling時のBPP結果

placements:
  各itemの配置位置・orientation等

bpp_order:
  BPPで決めた配置順

container_list:
  対象container情報

などを持つ。

今回、decide_position_v2 に bpp_layout.json がないことが判明したため、
v1と同じ形式のJSONをv2へ追加する必要があった。

10. 現在の最重要問題：not include

現在もっとも注目しているログは：

not included.

および

item XX not inside (hit boundary plane)

。

これは単純に「BPPが悪い」と決めつけない。

むしろ現時点では、

BPP上のposition
visualizer用position
decide_positionで生成するposition
execution_target
robotのplace action
simulatorが実際に評価する最終object pose
inclusion checkerのboundary判定

のどこかで座標がずれている可能性を疑っている。

11. 特に重要なログ

item 21：

final_target=(0.670,0.390,0.430)
execution_target=(0.670,0.390,0.430)

なのに、

item 21 not inside (hit boundary plane)

。

つまり、少なくとも「execution_targetに余計なshelf_liftが加わった」ことだけでは説明できない。

一方、item 3 / 8 は：

final_target z=1.128
execution_target z=1.149
shelf_lift=0.021

となっている。

したがって、

execution targetとfinal targetの差分があるケース/ないケースを分離して考える必要がある。

12. 現時点での仮説
仮説A：BPP座標の境界余裕が評価側と一致していない

BPPでは、

boundary_margin=0.015

などを使っている。

しかし評価側は別の幾何学的境界判定をしている可能性がある。

BPP上では入っていても、実際のobject AABB / OBBの端がboundary planeに触れると、

not inside (hit boundary plane)

になる可能性がある。

仮説B：BPPの「inside」と評価の「inside」が違う

BPP/CPP checker：

cpp_aabb_checker_v05

と、最終評価側の判定ロジックが同一とは限らない。

したがって、

BPP safe

と

evaluation is_included=True

は別物として扱うべき。

仮説C：placement後の物理的なpose変化

ログでは、

transport
place

が成功している。

しかしplace後に、

微小な姿勢変化
浮動小数点誤差
PyBulletの接触解決
boundary planeとの接触
support objectとの接触

などによって、最終AABBが数mmずれる可能性がある。

not inside が「hit boundary plane」と明示しているので、これはかなり重要な確認対象。

仮説D：target positionではなく「item中心 vs bounding box」の定義差

BPPがcenter positionを扱っていて、
評価側がcollision geometryの実寸を使っている場合、

center + half_extent

の境界が一致していない可能性がある。

特に、

item 21
item 17
item 24
item 35
item 23
item 4
item 29

が連続してboundaryに当たっていることから、
配置された物体群のgeometry/coordinate conversionを確認する価値が高い。

13. やってはいけないこと

現段階では、いきなり

BPP探索を全面的に作り直す
trial数を大幅に増やす
beam searchを大幅に変更する
新しい評価関数を大量に追加する
計画フェーズ5に手を出す

ことはしない。

まず、

「なぜBPPではsafeなのにevaluationではnot includedなのか」

を特定する。

14. 次にやるべき調査

優先順位は以下。

Priority A

実際の評価時のgeometryとBPP geometryを比較する。

特に、

item 21
item 17
item 24
item 35
item 23
item 4
item 29

について、

center
half extents
rotation
world AABB
container boundary
boundary plane
最小clearance

をログ出力する。

Priority B

not inside (hit boundary plane) を出している評価側コードを確認する。

ここで、

何を「inside」と定義しているのか

を正確に把握する。

推測で直さない。

Priority C

BPP checker (cpp_aabb_checker_v05) の判定と、
最終evaluation側の判定を同じ入力に対して比較する。

例えば：

BPP checker:
inside = True

evaluation:
inside = False

となる具体例を作る。

これができれば問題箇所がかなり絞れる。

Priority D

必要ならdecide_position_v2側で、
BPPのtarget positionをそのまま使うのではなく、

target position
    ↓
actual geometry extent
    ↓
evaluation boundary
    ↓
必要な inward offset
    ↓
execution target

という変換を明示的に入れる。

ただし、offset量はログ/geometryから決める。
適当に 0.01 や 0.02 を足すだけにはしない。

15. ユーザーの現在の意図

ユーザーは、

「とにかく新機能を増やしたい」のではなく、

現在のnot include問題を潰して、実際の評価でplaced item数を増やしたい

という方針。

また、

少し減ったね、考察してください

という流れから、
v5の結果を見て「探索結果の数字」だけではなく、
実際の評価ログまで含めて原因を考察することを重視している。

16. コーディング方針

コードを変更する場合は、

既存v5を壊さない
v6/v2など新しいディレクトリで作業
変更箇所を明確にする
ログを増やして原因を観測可能にする
推測だけでmarginを変更しない
変更前後で同じtaskを実行して比較
evaluation の
fill_score
num_placed_items
is_included
is_valid
is_placed_safe
を必ず比較する
17. 直近で作ったもの

decide_position_v2_fixed.zip を作成済み。

内容：

decide_position_v2_fixed/
├── agent.py
├── bpp_layout.json
├── README.md
└── README_source.md

bpp_layout.json は decide_position_v1 の形式を踏襲している。

18. このチャットで今後やること

次に作業するときは、

decide_position_v2 の現行コードを確認
bpp_layout.json の読み込み箇所を確認
not include を出している評価側コード/ログを確認
BPP geometryとevaluation geometryを比較
item 21等のboundary hitを再現
原因が分かってからv2/v6を修正
同じtaskで再実行
num_placed_items と is_included の改善を確認

という順番で進める。

最優先は「探索を賢くすること」ではなく、

BPPで安全と判定した配置が、なぜ実評価ではboundary hitになるのかを特定すること。


このまま**次のChatGPTチャットの最初に貼れば、かなりのところまで継続できます**。

特に今回の核心は、**「BPPの探索性能が悪い」のではなく、「BPP上のsafe判定と最終評価のinside判定が食い違っている可能性が高い」**というところです。ここを次の作業の起点にしましょう。