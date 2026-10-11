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
