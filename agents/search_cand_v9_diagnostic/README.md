# search_cand_v9_diagnostic

v8_geometryから診断ログのみ追加。採点・候補の選択・Beam幅・制約は変更しません。
`SC_DIAG_ITEMS=38`（カンマ区切りで複数指定可）で候補棄却理由を記録します。

## 実行
```bash
cd ~/simulator2
FILE_NAME=search_cand_v9_diagnostic
mkdir -p results/$FILE_NAME
SC_DIAG_ITEMS=38 EP_CPP_MODE=cpp \
EP_NEED_3D_BPP=./results/$FILE_NAME/bpp_layout_$FILE_NAME.json \
SC_SEARCH_TRIALS=8 SC_SEARCH_MAX_SECONDS=60 \
SC_BPP_BEAM=4 SC_BPP_BRANCH=3 SC_FEASIBILITY_BEAM=32 \
SC_FEASIBILITY_BRANCH=16 SC_REAR_OPEN_PENALTY=1.5 \
python -m scripts.run_test --module-path agents/$FILE_NAME/ \
--config-path configs/sample_config.json --verbose True --render-mode None \
--result-dir results/$FILE_NAME/ \
--result-fname evaluation_results_$FILE_NAME.json \
2>&1 | tee results/$FILE_NAME/$FILE_NAME.log
```

`[DIAG-ITEM]`: item38の各EP/姿勢の棄却内訳。`ep_total` はEP位置の合計（姿勢数を掛ける前）。
`inside`,`shelf`,`support`,`collision`,`static_path`: 各段階で落ちた数。
`[DIAG-BEAM]`: item38を置いた枝がBeam枝刈り前後に残ったか。
`[DIAG-RESULT]`: 完成レイアウトにitem38を含む枝の数。
`[DIAG-SUMMARY]`: 試行ごとの累積。大量のログになるので、まず38だけを指定。

**注意**: この診断だけでは「なぜ最終的に38を置けないか」を断定しません。結果をもとに絞ります。
