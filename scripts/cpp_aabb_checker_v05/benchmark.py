from time import perf_counter
from cpp_aabb_checker import (
    Box, ContainerBounds,
    make_small_shelf, make_main_shelf,
    check_transport_like_safe_grid_with_shelves,
)


def box(idx, x, y, z, lx, ly, lz):
    b = Box()
    b.id = idx
    b.cx, b.cy, b.cz = x, y, z
    b.lx, b.ly, b.lz = lx, ly, lz
    return b


container = ContainerBounds(-0.95, 0.95, -0.71, 0.71, 0.05, 1.57)
small = make_small_shelf(2.0, 1.52, 1.62, 0.05, 0.43, 0.0)
main = make_main_shelf(2.0, 1.52, 1.62, 0.05, 0.0)
shelves = [small, main]

packed = [
    box(i, -0.60 + (i % 4) * 0.35, 0.55 - (i // 4) * 0.25,
        0.30, 0.24, 0.20, 0.40)
    for i in range(8)
]

moving = box(999, 0, 0, 0, 0.24, 0.20, 0.40)

N = 100_000
ok = 0
t0 = perf_counter()
for i in range(N):
    tx = -0.50 + (i % 101) * 0.01
    ty = -0.30 + (i % 61) * 0.01
    r = check_transport_like_safe_grid_with_shelves(
        moving, container, packed, shelves,
        target_x=tx, target_y=ty, target_z=0.90,
        cut_x=0.43, thickness=0.05,
        start_margin=0.018, safety_margin=0.015,
        extra_lift=0.0, step_len=0.01, collision_gap=0.015,
    )
    ok += int(r["ok"])
elapsed = perf_counter() - t0
print(f"checks={N:,} elapsed={elapsed:.3f}s rate={N/elapsed:,.0f} checks/s ok={ok}")

# Phase 4 micro-benchmark: stack support checks
from cpp_aabb_checker import check_stack_support
support = make_box(900, (0.0, 0.0, 0.25), (0.65, 0.45, 0.40))
support.mass = 13.0
upper = make_box(901, (0.0, 0.0, 0.65), (0.55, 0.40, 0.40))
upper.mass = 8.0

N2 = 1_000_000
t0 = time.perf_counter()
ok2 = 0
for _ in range(N2):
    r = check_stack_support(upper, support)
    ok2 += int(r["supported"])
elapsed2 = time.perf_counter() - t0
print(f"stack_checks={N2:,} elapsed={elapsed2:.3f}s rate={N2/elapsed2:,.0f} checks/s ok={ok2}")
