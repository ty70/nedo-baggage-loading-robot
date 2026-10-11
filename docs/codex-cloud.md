# Codex Cloud 開発環境

既存checkoutをそのまま使います。別のGit worktreeは作成しません。
配置アルゴリズム、公式評価関数、既存設定・結果・追跡済みcheckerバイナリは変更しません。

## 構成と依存関係

- `agents/`: base、grid、extreme point、safe grid、search candidate v1〜v14などの配置・探索実装。
- `configs/`: サンプル2課題、開発用設定、荷物パラメータ表。
- `src/ground_handling/`: `app.py`が課題を実行し、`runner.py`がspawn子プロセス・共有メモリ・時間/メモリ制限を管理。
  `env.py`がGymnasium/PyBulletを統合、`validator.py`が包含・Y→X搬入・定着判定、`evaluator.py`が評価を担当。
- `scripts/`: 評価入口、C++ checkerの初版〜v05、各版の既存テストとベンチマーク。
- `results/`: 既存評価・配置計画。今回の検証はここへ書き込みません。
- `samples/`, `figs/`: サンプル提出物・説明画像。ルートHTMLは配置計画の可視化用。
- `Agents.md`: 候補生成/選択、C++、Physics、Evaluationの診断構想。今回は診断追加やアルゴリズム変更は対象外。

Dockerの公式例はUbuntu 24.04、Miniconda Python 3.12、Gymnasium 1.2.3、PyBullet 3.2.7、Pillow 10.3.0、CPU Torch 2.7.0。
`docker-compose.yml`はリポジトリを`/workspace`へマウントする開発コンテナです。
今回のCloudはDebian 13、Python 3.12.14、GCC/G++ 14.2で直接実行します。
NumPy、Gymnasium、PyBullet（`pybullet_data`/`pybullet_utils`を含む）、Pillowがシミュレーターの必須依存です。
baseとsearch candidate v14はTorchをimportしていないため、Torchは今回の検証に不要です。

C++ checkerはPythonヘッダー、C++17コンパイラー、pybind11、setuptoolsが必要です。
`setup.py`では`-O3`を使用します。代替CMake手順にはCMake 3.18以上が必要ですが、今回のビルドには不要です。
同じモジュール名`cpp_aabb_checker`を各版で使うため、v05を明示してロードします。
C++は幾何学・搬入・支持の事前判定であり、PyBulletの変形・定着・公式判定を代替しません。

## セットアップ

```bash
cd /workspace/nedo-baggage-loading-robot
bash scripts/cloud_setup.sh
```

既定の保存先は`/workspace/nedo-cloud`です。`NEDO_CLOUD_STATE`で外部の絶対パスへ変更できます。
Python仮想環境、PyBulletソース/wheel、再ビルドしたcheckerはこの保存先に置きます。
PyPIと`files.pythonhosted.org`へのHTTPS接続が必要です。実行時の外部サービス・追加秘密情報は不要です。

この環境のPythonビルド設定は`clang++`を指しますが、clangは未導入でした。
`CC=gcc CXX=g++`で既存のコンパイラーを使用します。
PyBullet 3.2.7にはCPython 3.12 Linux用wheelがなく、ソースビルドが必要でした。
公式PyPIに掲載されたSHA-256を確認してからビルドします。TLS検証は維持します。
upstreamビルドのCPU数を2として4並列に制限します。
GCC 14でエラー化されたNumPy APIのポインタ型警告だけを`-Wno-error=incompatible-pointer-types`で警告へ戻します。
対象は`PyArray_SimpleNew`が返す配列を`PyArray_DATA`へ渡す箇所です。ソースを変更しません。
checkerは必ず外部へ再ビルドし、Phase 1〜5の既存assertionテストを実行します。
セットアップを再実行した場合、導入済みPyBulletは再ビルドせず、checkerは現在のソースから再ビルドします。

## シミュレーター実行

```bash
cd /workspace/nedo-baggage-loading-robot
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
EP_CPP_MODE=cpp SC_SEARCH_TRIALS=16 SC_SEARCH_MAX_SECONDS=60 \
SC_FEASIBILITY_BEAM=12 SC_FEASIBILITY_BRANCH=8 \
/workspace/nedo-cloud/venv/bin/python scripts/cloud_run.py \
  --module-path agents/search_cand_v14_order_2716_diagnostic/ \
  --config-path configs/sample_config.json --verbose True
```

`human`を指定しないためPyBullet DIRECTモードで動作し、画面・GPU・Docker daemonは不要です。
各実行は新しい`/workspace/nedo-cloud/runs/simulation-*`へ評価JSON、配置計画、診断ログ、実時間記録を保存します。
再ビルドcheckerを親・spawn子プロセスへ事前ロードして、追跡済みバイナリへの依存を避けます。
`cloud_run.py`は既存`run_test.py`へ委譲するだけで、配置や評価の処理を変更しません。
結果JSONの例外状態・欠落課題も確認します。配置途中停止は実行障害と区別します。

`run.sh`は同じ名前の結果を上書きし、`tee`でPythonの終了コードを隠し得るため、そのまま使用しません。
元の`run_test.py`も例外をJSONへ記録してexit 0になり得るため、終了コードだけで成功を判断しません。
`--verbose`は元実装の`type=bool`に従うため、無効化する場合は引数を省略してください。
GUI、全エージェント、コンテスト本番の非公開課題、Torch依存の学習処理は今回の検証対象外です。

## 評価値の読み方

このcheckoutの`Evaluator.evaluate()`が返すのは`fill_score`と`num_placed_items`のみです。
READMEにある重心・安定性・優先荷物・ソフト貨物のスコアは、この実装からは出ません。
`num_placed_items`は個数ではなく全荷物に対する積載割合です。
JSONの`status=success`は評価処理が完了したことを示し、全荷物を積めたことは保証しません。
`place_states`の包含・搬入・定着状態と、ログの停止理由を併せて確認します。

## 2026年10月11日の検証結果

対象commit: `ee27c66a2bdd9882610fcfe5a0abce77dc1e7979`。
セットアップスクリプト実行、C++ v05 Phase 1〜5の既存テスト、`pip check`が通過しました。
PyBullet DIRECTで物理ステップとTinyRendererによる64×64のRGB/深度取得を確認しました。
baseおよびv14で、既存サンプルの全2課題（000/001）を評価しました。例外・欠落課題・タイムアウトはありません。

| 実行 | 全体実時間 | プロセス終了コード | 評価処理 | 全荷物積載 |
|---|---:|---:|---|---|
| base | 0.303797秒 | 0 | 2課題完了 | 両課題とも途中停止 |
| v14 | 65.144475秒 | 0 | 2課題完了 | 両課題とも途中停止 |

| agent | 課題 | fill_score | 積載数/総数 | 積載割合 | 最大optimize時間 | 最大policy時間 |
|---|---|---:|---|---:|---:|---:|
| base | 000 | 0.000000000 | 1/41 | 0.024390244 | 0.000185929秒 | 0.000555115秒 |
| base | 001 | 0.000000000 | 1/42 | 0.023809524 | 0.000000000秒 | 0.000896738秒 |
| v14 | 000 | 11.134955637 | 9/41 | 0.219512195 | 60.715898319秒 | 0.014527521秒 |
| v14 | 001 | 13.533175238 | 17/42 | 0.404761905 | 0.000000000秒 | 0.314555458秒 |

停止理由:

- base: 000は荷物17と既配置3、001は荷物1と既配置0の搬入衝突で停止。
- v14/000: 荷物39と小棚が搬入開始位置で衝突。ログの距離は0.014999998807907153 mで、安全マージン0.015 mの直下。
  `place_states`は包含=true、搬入=false、定着=falseです。探索は30個を搬入可能と予測しましたが、実配置は9個で停止。
  これは幾何学checkerの予測とPyBullet搬入判定の差を示す観測です。原因の修正や再現性調査は今回の環境構築とは分けます。
- v14/001: 18ステップ目、再計画後も配置計画がなく`NO-BPP-PLAN-AFTER-REPLAN`の`SAFE-ABORT`。
  荷物14の意図的な範囲外座標が包含判定で拒否され、包含/搬入/定着がすべてfalseとなりました。

評価時には一部の配置済み荷物が境界判定で除外されたログもあります。fill_scoreと積載割合は異なる判定を反映します。
配置途中停止は開発環境の実行不能ではありません。アルゴリズムや公式判定を変更せずに、その結果を記録しています。

生データとログ:

- base: `/workspace/nedo-cloud/runs/simulation-fpc606dj/{evaluation.json,runtime.json}`
- v14: `/workspace/nedo-cloud/runs/simulation-mj77jwin/{evaluation.json,runtime.json,bpp_layout.json}`
- ログ: `/workspace/nedo-cloud-artifacts/{base.log,v14.log,setup-validation.log,pybullet-build-retry.log}`

既存`results/`の84ファイルは事前・事後のSHA-256が完全一致し、追加・削除もありません。
既存の追跡ファイルの差分もありません。変更は新規環境用スクリプト2本とこの文書だけです。
検証は現在のCloudインスタンスでの実績です。公開後の新規タスクへの復元は未検証です。
