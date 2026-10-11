# decide_position_v2

`decide_position_v1` 相当の既存ロジックを維持し、`not included` の解消を目的として、候補位置に対する **PlacementValidator と同一の inclusion plane 判定**を追加した版。

## v2 の変更点

1. 既存の `_safe_bounds()`、衝突判定、安定性判定、6姿勢探索、スコアリングは維持。
2. `container['n_vecs']` / `container['points']` が存在する場合、validator と同じ
   `dot(n, target-point) + dot(abs(n), half_lwh)` を候補ごとに計算。
3. 境界付近/外側の候補は `worst plane` の法線方向へ内側に投影。
4. validator の `INCLUSION_MARGIN=-0.005` 自体は変更しない。
5. 投影後にも exact inclusion を再確認し、失敗した候補は破棄。
6. `[decide_position_v2][INCLUSION-REJECT]` で plane / max_dot / target を記録。

## 狙い

従来の `_safe_bounds()` は単純な矩形境界しか直接扱っていないため、実際のコンテナ境界面と候補中心が「ほぼ接している」ケースを完全には排除できない。v2 は simulator の inclusion 判定を候補生成段階へ持ち込み、`not included` を後段で発生させるのではなく、候補生成時に除外する。

## 重要な非変更点

- `INCLUSION_MARGIN` は変更しない。
- C++ AABB checker は変更しない。
- final target の意味を変更しない。
- 既存の safety margin を inclusion 解決のために勝手に変更しない。
- 従来の fallback を新しい再計画ロジックへ拡張しない。

## 実行例

```bash
python -m scripts.run_test \
  --module-path agents/decide_position_v2/ \
  --config-path configs/sample_config.json \
  --verbose True \
  --render-mode None \
  --result-fname evaluation_results_decide_position_v2.json
```
