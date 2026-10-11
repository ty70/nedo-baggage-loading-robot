#!/usr/bin/env bash
# Codex Cloud: 既存ソース・結果を変更せず、外部ディレクトリへ導入する。
set -euo pipefail
project=$(cd "$(dirname "$0")/.." && pwd)
state=${NEDO_CLOUD_STATE:-/workspace/nedo-cloud}
case "$state" in /*) ;; *) echo "NEDO_CLOUD_STATEは絶対パスで指定してください" >&2; exit 1;; esac
mkdir -p "$state"
if [ ! -x "$state/venv/bin/python" ]; then
  python -m venv "$state/venv"
fi
py="$state/venv/bin/python"
export CC=gcc CXX=g++ PYTHONDONTWRITEBYTECODE=1
# GCC 14でエラー化したupstream NumPy APIのポインタ型警告を警告として保持。
export CFLAGS="${CFLAGS:-} -Wno-error=incompatible-pointer-types"
"$py" -m pip install --no-cache-dir gymnasium==1.2.3 pillow==10.3.0 numpy==2.3.5 pybind11==3.0.4 setuptools==84.0.0 wheel==0.48.0 cloudpickle==3.1.2 typing_extensions==4.16.0 farama-notifications==0.0.6 packaging==26.3
if ! "$py" -c 'from importlib.metadata import version; assert version("pybullet") == "3.2.7"' 2>/dev/null; then
  mkdir -p "$state/source" "$state/wheels"
  "$py" -m pip download --no-deps --no-binary=:all: pybullet==3.2.7 -d "$state/source"
  printf '%s  %s\n' 042879db8d101ac7590dee475fc6aded508b85fe1273fdbbfde1d88bd200e14f "$state/source/pybullet-3.2.7.tar.gz" | sha256sum -c -
  tar -xzf "$state/source/pybullet-3.2.7.tar.gz" -C "$state/source"
  "$py" - "$state" <<'PY'
import multiprocessing, os, runpy, sys
state = sys.argv[1]
# upstream setup.py は CPU 数の2倍でビルドするため、4並列に制限。
multiprocessing.cpu_count = lambda: 2
os.chdir(state + '/source/pybullet-3.2.7')
sys.argv = ['setup.py', 'bdist_wheel', '--dist-dir', state + '/wheels']
runpy.run_path('setup.py', run_name='__main__')
PY
  "$py" -m pip install --no-deps "$state"/wheels/pybullet-3.2.7-*.whl
fi
mkdir -p "$state/checker"
cd "$project/scripts/cpp_aabb_checker_v05"
"$py" setup.py build_ext --force --build-lib "$state/checker" --build-temp "$state/checker/build"
"$py" - "$state/checker" <<'PY'
import runpy, sys
sys.path.insert(0, sys.argv[1])
import cpp_aabb_checker
print('Rebuilt checker:', cpp_aabb_checker.__file__)
runpy.run_path('test_checker.py', run_name='__main__')
PY
"$py" -m pip check
