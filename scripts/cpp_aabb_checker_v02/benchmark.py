import time
from cpp_aabb_checker import Box, ContainerBounds, check_transport_like_safe_grid


def make_box(idx, pos, dims):
    b = Box()
    b.id = idx
    b.cx, b.cy, b.cz = pos
    b.lx, b.ly, b.lz = dims
    return b


container = ContainerBounds(-0.96, 0.96, -0.685, 0.685, 0.04, 1.57)
item = make_box(999, (0, 0, 0), (0.55, 0.24, 0.40))
packed = [
    make_box(i, (-0.7 + (i % 7) * 0.22, -0.45 + (i // 7) * 0.18, 0.30), (0.18, 0.16, 0.20))
    for i in range(28)
]

N = 100_000
start = time.perf_counter()
ok = 0
for i in range(N):
    tx = -0.65 + (i % 100) * 0.013
    ty = -0.35 + ((i // 100) % 60) * 0.011
    r = check_transport_like_safe_grid(
        item, container, packed,
        tx, ty, 0.75,
        0.44, 0.04,
        0.018, 0.015, 0.0,
        0.01, 0.015,
    )
    ok += int(r["ok"])
elapsed = time.perf_counter() - start
print(f"checks={N:,} elapsed={elapsed:.3f}s rate={N/elapsed:,.0f} checks/s ok={ok}")
