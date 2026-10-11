"""Codex Cloud用の実行入口。既存の評価コードへ処理を委譲する。"""
import os
import sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
STATE = Path(os.environ.get('NEDO_CLOUD_STATE', '/workspace/nedo-cloud'))
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(STATE / 'checker'))
# spawnされたワーカーにも、再ビルドしたcheckerを事前ロードする。
import cpp_aabb_checker

if Path(cpp_aabb_checker.__file__).resolve().parent != (STATE / 'checker').resolve():
    raise RuntimeError('cloud_setup.shでcheckerをビルドしてください')

if __name__ == '__main__':
    if '--result-dir' in sys.argv or '--result-fname' in sys.argv:
        raise SystemExit('出力先は自動生成します。結果ファイルの引数は指定しないでください。')
    import json
    import tempfile
    import time
    from scripts.run_test import main, parse_args

    os.chdir(PROJECT)
    output_root = STATE / 'runs'
    output_root.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='simulation-', dir=output_root))
    os.environ['EP_NEED_3D_BPP'] = str(output / 'bpp_layout.json')
    sys.argv += ['--result-dir', str(output), '--result-fname', 'evaluation.json']
    expected_tasks = set(json.loads(Path(parse_args().config_path).read_text()))
    print('checker:', cpp_aabb_checker.__file__, flush=True)
    print('output:', output, flush=True)
    started = time.perf_counter()
    main()
    elapsed = time.perf_counter() - started
    results = json.loads((output / 'evaluation.json').read_text())
    errors = [k for k, v in results.items() if v.get('status') != 'success' or v.get('evaluation') is None]
    # 元runnerは例外をJSONへ格納してexit 0で終わるため、結果も検査する。
    missing_tasks = sorted(expected_tasks - set(results))
    exit_code = 1 if errors or missing_tasks or not results else 0
    (output / 'runtime.json').write_text(json.dumps({
        'wall_seconds': elapsed, 'exit_code': exit_code,
        'task_ids': list(results), 'error_tasks': errors,
        'missing_tasks': missing_tasks,
        'checker': cpp_aabb_checker.__file__,
    }, indent=2) + '\n')
    print('wall_seconds:', elapsed, 'exit_code:', exit_code, flush=True)
    raise SystemExit(exit_code)
