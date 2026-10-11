最初の目標は、

「あるアイテムが配置失敗したとき、それが Algorithm/Transport/Physics/Evaluationのどこで起きたのかを説明できる状態」

そのために、現在のシミュレーターを次のように観測する。

                ITEM
                  │
                  ▼
        ┌─────────────────┐
        │ Candidate生成   │
        └────────┬────────┘
                 │
          candidate[]
                 │
                 ▼
        ┌─────────────────┐
        │ Candidate評価   │
        └────────┬────────┘
                 │
             selected
                 │
                 ▼
        ┌─────────────────┐
        │ C++ checker      │
        │                  │
        │ path_clear       │
        │ final_clear      │
        └────────┬────────┘
                 │
                 ▼
        ┌─────────────────┐
        │ PyBullet         │
        │                  │
        │ spawn            │
        │ transport       │
        │ settle           │
        └────────┬────────┘
                 │
                 ▼
        ┌─────────────────┐
        │ Evaluation       │
        └─────────────────┘

・観測点を入れる。

1. 最初に作るログ

例えば1アイテムについて、

{
  "item_id": 17,

  "candidate_generation": {
    "count": 43
  },

  "selected_candidate": {
    "candidate_id": 21,
    "position": [1.20, 2.30, 0.50],
    "orientation": [0, 0, 90]
  },

  "cpp_check": {
    "path_clear": true,
    "final_clear": true
  },

  "physics": {
    "settled": true,
    "actual_position": [1.14, 2.35, 0.45],
    "position_delta": 0.085,
    "orientation_delta": 6.2
  },

  "evaluation": {
    "success": false
  }
}

まずはこれだけでも非常に大きい。

なぜなら、

C++ OK
   ↓
Physics NG

なのか、

Candidate生成 NG

なのか、

Physics OK
   ↓
Evaluation NG

なのかが見えるから。

2. さらに重要なのが「全候補」

選択された候補だけでは足りません。

例えば、

candidate 0
candidate 1
candidate 2
...
candidate 42

について、

candidate_id
position
orientation
path_clear
final_clear
score
selected

を残す。

すると後から、

43 candidates
   │
   ├── 31 infeasible
   ├── 11 feasible
   └── 1 selected

という構造が見える。

ここで、

「そもそも候補が悪い」

と

「候補は良いのに選択を間違えた」

を分離できる。

3. 最初の診断指標

最初から複雑な指標を作る必要はない。

まずは5つ。

Candidate Coverage

候補生成数

Feasibility Rate

C++で合法だった候補／全候補

Selection Quality

選択候補が全合法候補の中でどの程度だったか

これは最初は単純に、

selected candidate
vs
all feasible candidates

を保存しておけば十分。

Physics Divergence

planned_pose
      ↓
actual_pose

Δposition
Δrotation

Evaluation Divergence

Physics上は成功
       ↓
Evaluationでは失敗

4. ここで非常に重要な実験をする

次に、

「同じ候補をPhysicsに入れる」

実験をする。

現在のアルゴリズムが選んだ候補だけでなく、

feasible candidate #1
feasible candidate #2
feasible candidate #3
...

を個別にPyBulletへ投入してみる。

すると、

candidate       C++       Physics
--------------------------------
#1              OK        OK
#2              OK        NG
#3              OK        OK
#4              OK        NG
#5              OK        OK

というデータが得られる。

これはかなり重要。

もしこうなったら、

C++ checkerとPhysicsの間に体系的な乖離がある

という強い証拠になる。

逆に

candidate       C++       Physics
--------------------------------
#1              OK        OK
#2              OK        OK
#3              OK        OK
#4              OK        OK
#5              OK        OK

なのに実際のagentが悪いなら、

候補生成・候補選択点アイテム順序などのAlgorithm側

を疑えばいい。

5. さらに「Physicsを固定」する

次の実験では逆にする。

例えば、

A → B → C → D → E

というアイテム順を固定。

さらに、

candidate selection

も固定。

その状態で、

Physics

だけを見る。

つまり、

同じ入力を何回やっても同じ結果になるか

を確認する。

これによって

決定的なアルゴリズム問題

と

物理シミュレーションの挙動問題

を切り分けられる。

6. そして初めてJevを入れる

ここまでのログができたら、Jev/TypeSafeの出番。

重要なのは、

Jevに直接Simulatorを操作させる必要はない。

最初は、

diagnostic_report.json

を渡して、

現在の観測結果
↓
既存の仮説
↓
各仮説が予測する結果
↓
実験結果

を比較させる。

例えば、

H1 candidate generation problem

Evidence:
candidate count = 43
feasible = 11

Unknown:
whether a better candidate exists outside current generator

Next experiment:
increase candidate generation coverage

のような形。

7. Jevには「仮説の採点」ではなく「証拠との整合性」を扱わせる

例えば、

H1 = Algorithm
H2 = Physics

について、

H1: 80点
H2: 20点

みたいな使い方を最初からすると、結局AIの主観になってしまう。

それより、

Hypothesis
Prediction
Observed evidence
Contradicting evidence
Missing evidence
Next experiment

という構造にする。

例えば、

hypothesis: physics_settling_mismatch

prediction:
  - planned_pose and actual_pose diverge
  - divergence is systematic
  - C++ says feasible while physics fails

observed:
  position_delta: 0.082
  cpp_final_clear: true
  physics_success: false

contradicting:
  none

missing:
  - orientation_delta distribution
  - same test with alternative candidates

next_experiment:
  replay_10_feasible_candidates

これなら、AIが「正解」を作るのではなく、科学的なデバックの補助となる。

8. ここでTypeSafeの意味が出てくる

TypeSafeを使うなら、出力を自由文にしない。

例えば、概念的には、

class Diagnosis:
    hypothesis_id: str
    evidence: list[str]
    contradictions: list[str]
    missing_evidence: list[str]
    next_experiment: str

のような構造化された診断結果にする。

そうすれば、

LLM
 ↓
Diagnosis schema
 ↓
Python
 ↓
experiment runner
 ↓
Simulator
 ↓
result JSON
 ↓
LLM

というループが作れる。

9. 最終形はこうなる

                    ┌───────────────────┐
                    │     Simulator     │
                    └─────────┬─────────┘
                              │
                              ▼
                    ┌───────────────────┐
                    │ Diagnostic Logger │
                    └─────────┬─────────┘
                              │
                              ▼
                    ┌───────────────────┐
                    │ Experiment Result │
                    └─────────┬─────────┘
                              │
                              ▼
                    ┌───────────────────┐
                    │ Jev / TypeSafe    │
                    │                   │
                    │ Hypothesis        │
                    │ Analysis          │
                    │ Experiment Design │
                    └─────────┬─────────┘
                              │
                              ▼
                    ┌───────────────────┐
                    │ Experiment Runner │
                    └─────────┬─────────┘
                              │
                              ▼
                         Simulator

つまり、

SimulatorをAIに解かせるのではなく、Simulator自身を「実験装置」にする。

そして、次にやること

ここからは抽象論をやめて、実際の ~/simulator に合わせます。

次のステップは diagnostic logger を設計することです。

ただし、いきなりコードを書くのではなく、まず現在の

src/
agents/
scripts/
run.sh

の中で、

候補を生成している場所
候補を選択している場所
C++ checkerを呼んでいる場所
PyBulletで実際に配置している場所
評価結果を作っている場所

を特定します。

ここが分かれば、現在のコード構造を壊さずに、

diagnostic logger

を差し込めます。

なので次は ~/simulator のコード構造を調べて、この5箇所を特定するところから進める

