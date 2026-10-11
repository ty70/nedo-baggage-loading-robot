# v15の汎化性能を測る30ケース

課題000・001だけを見ながら改善したv15が、荷物やコンテナの条件を変えたときにどこで止まるかを測定します。エージェント、公式判定器、評価関数、元のsample_config.json、既存results、追跡済みcheckerバイナリは変更しません。

## ケース設計

各条件につき5ケース、計30ケースです。乱数シードを固定し、各条件の3ケースを開発用、2ケースを検証用としています。今回のみ、未調整のv15の基準測定として両方を実行します。以後の調整は開発用を使い、検証用はリリース候補の確認時に限定します。検証結果を見ながら繰り返し修正した場合は、新しい独立した検証セットが必要です。

| 条件 | 変更内容 |
|---|---|
| order | 荷物の入力順をシャッフル。オフラインではoptimizeが再並べ替えするためオンラインと解釈を分ける |
| dimensions | 荷物の寸法を新規生成。薄型・小型・細長い荷物を含む |
| load | 既存荷物の物理特性を使い、20・32・48・60・72個へ変更 |
| containers | 1〜3個、幅1.5〜2.5 m・奥行1.2〜1.9 m・高さ1.25〜1.95 m |
| shelf_opening | 棚の有無、切り欠き幅・高さ、壁厚を変更 |
| attributes | 質量3〜25 kg、優先荷物とソフト貨物の割合を変更 |

000をオフライン、001をオンラインのテンプレートにします。各条件でオフライン3ケース、オンライン2ケースを生成します。IDは101〜130を使い、荷物IDは各ケース内で0から振り直します。荷物が単体で少なくとも1つのコンテナの内寸に入る必要条件を検証します。ただし、棚・開口部を通過できることや、全荷物を同時に積載できることは保証しません。総体積負荷もmanifestへ記録します。

これは未知の大会データの分布を推定したテストではありません。元データのテンプレートを使う合成感度テストです。スコアの低下だけで過学習と断定せず、条件別の弱点を探す用途に使います。

## 実行

Python 3.12とGCC/G++で実行します。以下の保存先は環境・生成ケース・生ログをリポジトリ外へ置きます。

```bash
export NEDO_CLOUD_STATE="$HOME/nedo-robustness"
TAR_OPTIONS=--no-same-owner bash scripts/cloud_setup.sh
python -m scripts.generate_robustness_cases --output-dir "$NEDO_CLOUD_STATE/cases"
"$NEDO_CLOUD_STATE/venv/bin/python" -m scripts.test_robustness_cases
"$NEDO_CLOUD_STATE/venv/bin/python" -m scripts.benchmark_robustness \
  --cases-dir "$NEDO_CLOUD_STATE/cases" \
  --output-root "$NEDO_CLOUD_STATE/benchmarks" \
  --split development
```

初回の固定基準評価のみ `--split all` を指定しました。検証用だけは `--split holdout`、個別ケースは `--case-ids 101 116` を指定できます。

各ケースを別プロセスで順番に実行するため、1ケースのエラーで後続ケースが欠落しません。ケース全体の上限は420秒です。エージェントの制限は初期化10秒、optimize180秒、policy8秒、メモリ12 GBです。探索設定は提出版と同じ16回・60秒・beam12・branch8、OpenBLAS/OMP=1です。時間制限による探索打ち切りのため、CPU性能や負荷で計画が変わる可能性があります。

`run_robustness_case.py`は既存scripts.run_testを呼びます。元の評価処理を実行し、Agentの計画更新と実積載IDを観測するだけで、行動や物理判定を変更しません。spawnワーカーにも外部へ再ビルドしたC++ v05を事前ロードします。GitHubのv15を評価するため、提出ZIPのバイナリ互換性を再検証するものではありません。

## 出力の読み方

各実行は新しいv15-robustness-*ディレクトリに保存します。

- report.json：ケース別と条件別・split別の集計、環境・ソースハッシュ。
- cases.csv：比較用の表。
- ケースID/run.log：元の評価ログと停止理由。
- ケースID/evaluation.json：公式ローカル評価器の結果。
- ケースID/plan_history.jsonl：各更新段階の計画ID。
- ケースID/actual_counts.json：評価時点の実積載ID。
- ケースID/bpp_layout.json：最後に保存された配置計画（途中の計画はplan_history参照）。

初期オフライン計画数はoptimize直後の計画数です。planned_union_countは全更新段階で一度でも計画に採用されたIDの集合サイズであり、同時に存在した計画数ではありません。オンラインでは初期オフライン計画数をnullにします。actual_packed_countはpacked_itemsに登録された荷物数です。fill_scoreは実積載数とは別に包含条件で計算されます。

SAFE-ABORT、包含拒否、搬入拒否、定着拒否、エージェントタイムアウト、評価エラー、ケース時間超過を区別します。status=successは評価処理の完了であり、全荷物積載とは限りません。エラーのあるケースは平均から除外し、件数を別記するので平均値だけを読まないでください。

ローカルEvaluatorはfill_scoreと積載割合しか返しません。重心・安定性・優先荷物・ソフト貨物スコアや非公開課題を含む提出サイトの総合スコアは再現できません。attributes群は主に配置選択・物理定着への感度を確認します。

実行ログはPYTHONUNBUFFERED=1で保存します。初回測定のケース110ではログが欠けたため、同じ設定を再測定し、同じ計画数・実積載数・スコアとALL-ROUTES-BLOCKEDを確認しました。再測定は検証セットを使った調整ではなく、ログ診断のみです。詳細は基準測定結果文書を参照してください。

開発用は各群でオフライン2・オンライン1、検証用は各群でオフライン1・オンライン1です。モード比率が違うため、split別の単純平均の差だけを過学習の証拠として扱わないでください。
