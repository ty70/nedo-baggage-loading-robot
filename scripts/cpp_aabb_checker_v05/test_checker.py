from cpp_aabb_checker import (
    Box,
    ContainerBounds,
    oriented_dims,
    inside_container,
    aabb_overlap,
    aabb_overlap_python_order,
    aabb_overlap_python_order_debug,
    check_candidate,
    check_transport_path,
    check_transport_like_safe_grid,
    make_small_shelf,
    make_main_shelf,
    check_shelf_support,
    check_candidate_with_shelves,
    check_transport_path_with_shelves,
    check_transport_like_safe_grid_with_shelves,
    check_stack_support,
    find_stack_supports,
    check_stack_candidate,
)


def make_box(idx, pos, dims, mass=0.0, is_soft=False):
    b = Box()
    b.id = idx
    b.cx, b.cy, b.cz = map(float, pos)
    b.lx, b.ly, b.lz = map(float, dims)
    b.mass = float(mass)
    b.is_soft = bool(is_soft)
    return b


# ---------------------------------------------------------------------------
# Phase 1
# ---------------------------------------------------------------------------
container000 = ContainerBounds(
    -1.0 + 0.04, 1.0 - 0.04,
    -1.45 / 2 + 0.04, 1.45 / 2 - 0.04,
    0.04, 1.61 - 0.04,
)

assert oriented_dims(0.75, 0.56, 0.27, 1) == (0.75, 0.27, 0.56)

b0 = make_box(3, (0.567, 0.530, 0.326), (0.75, 0.27, 0.56))
assert inside_container(b0, container000, 0.005)

b1 = make_box(17, (-0.5, -0.4, 0.326), (0.75, 0.27, 0.56))
assert not aabb_overlap(b0, b1, 0.0)

b2 = make_box(99, (0.60, 0.53, 0.326), (0.55, 0.24, 0.40))
assert aabb_overlap(b0, b2, 0.0)

r = check_candidate(b2, container000, [b0, b1], inclusion_margin=0.005, collision_gap=0.0)
assert r["inside"] is True
assert r["collision_free"] is False
assert r["ok"] is False
assert 3 in r["collisions"]


# ---------------------------------------------------------------------------
# Phase 2: Y -> X transport
# ---------------------------------------------------------------------------
moving = make_box(38, (0, 0, 0), (0.55, 0.24, 0.40))
block_y = make_box(32, (-0.30, -0.37, 0.40), (0.24, 0.40, 0.55))

r = check_transport_path(
    moving, [block_y],
    start_x=-0.30, start_y=-0.725, path_z=0.40,
    target_x=0.50, target_y=0.30,
    step_len=0.01, collision_gap=0.0,
)
assert r["ok"] is False
assert r["segment"] == "Y"
assert 32 in r["collision_ids"]

r = check_transport_path(
    moving, [],
    start_x=-0.30, start_y=-0.725, path_z=0.40,
    target_x=0.50, target_y=0.30,
    step_len=0.01, collision_gap=0.0,
)
assert r["ok"] is True

block_x = make_box(77, (0.10, 0.30, 0.40), (0.20, 0.20, 0.40))
r = check_transport_path(
    moving, [block_x],
    start_x=-0.30, start_y=-0.725, path_z=0.40,
    target_x=0.50, target_y=0.30,
    step_len=0.01, collision_gap=0.0,
)
assert r["ok"] is False
assert r["segment"] == "X"
assert 77 in r["collision_ids"]

r = check_transport_like_safe_grid(
    moving, container000, [],
    target_x=0.50, target_y=0.30, target_z=0.80,
    cut_x=0.44, thickness=0.04,
    start_margin=0.018, safety_margin=0.015,
    extra_lift=0.0, step_len=0.01, collision_gap=0.015,
)
assert r["ok"] is True
assert abs(r["path_z"] - 0.88) < 1e-9


# ---------------------------------------------------------------------------
# Phase 3: small shelf / main shelf
# ---------------------------------------------------------------------------
small = make_small_shelf(
    length=2.0, width=1.45, height=1.61,
    thickness=0.04, cut_x=0.44, buffer=0.0,
)
assert small.kind == "small_shelf"
assert small.id == -1001
assert abs(small.box.cx - (-0.740)) < 1e-9
assert abs(small.box.cy - 0.0) < 1e-9
assert abs(small.box.cz - 0.825) < 1e-9
assert abs(small.box.lx - 0.44) < 1e-9
assert abs(small.box.ly - 1.37) < 1e-9
assert abs(small.box.lz - 0.04) < 1e-9

# Real v17-like small shelf placement from case000: item=9, ori=5,
# dims=(0.25, 0.65, 0.45), target=(-0.740, 0.340, 1.091).
small_item = make_box(9, (-0.740, 0.340, 1.091), (0.25, 0.65, 0.45))
r = check_shelf_support(
    small_item, small,
    outer_wall_clearance=0.006,
    inward_overhang=0.006,
    y_overhang=0.0,
    z_tolerance=0.03,
)
assert r["supported"] is True
assert r["x_ok"] is True
assert r["y_ok"] is True
assert r["z_ok"] is True

# The old v16 failure pattern: 0.45 m wide item centered on 0.44 m shelf
# protrudes 5 mm through the outer wall. v17/v03 must reject support.
old_bad = make_box(999, (-0.740, 0.0, 1.091), (0.45, 0.24, 0.45))
r = check_shelf_support(
    old_bad, small,
    outer_wall_clearance=0.006,
    inward_overhang=0.006,
    y_overhang=0.0,
    z_tolerance=0.03,
)
assert r["supported"] is False
assert r["x_ok"] is False

# Main shelf geometry for case001.
main = make_main_shelf(
    length=2.0, width=1.52, height=1.62,
    thickness=0.05, buffer=0.0,
)
assert main.kind == "main_shelf"
assert main.id == -1002
assert abs(main.box.cx - 0.0) < 1e-9
assert abs(main.box.cy - 0.38) < 1e-9
assert abs(main.box.cz - 0.835) < 1e-9
assert abs(main.box.lx - 1.95) < 1e-9
assert abs(main.box.ly - 0.66) < 1e-9
assert abs(main.box.lz - 0.05) < 1e-9

# Real v17-like main shelf placement: item=2 target=(0.607,0.565,1.106),
# dims=(0.65,0.25,0.45). Bottom is about 21 mm above shelf top.
main_item = make_box(2, (0.607, 0.565, 1.106), (0.65, 0.25, 0.45))
r = check_shelf_support(
    main_item, main,
    outer_wall_clearance=0.0,
    inward_overhang=0.006,
    y_overhang=0.0,
    z_tolerance=0.03,
)
assert r["supported"] is True

# Candidate physically intersecting the small shelf must be rejected.
through_small = make_box(88, (-0.740, 0.0, 0.825), (0.20, 0.20, 0.20))
r = check_candidate_with_shelves(
    through_small, container000, [], [small],
    inclusion_margin=0.005, collision_gap=0.0,
)
assert r["ok"] is False
assert -1001 in r["shelf_collisions"]

# Transport path intersects main shelf at shelf height.
transport_item = make_box(200, (0, 0, 0), (0.30, 0.20, 0.20))
r = check_transport_path_with_shelves(
    transport_item,
    packed=[],
    shelves=[main],
    start_x=0.0,
    start_y=-0.76,
    path_z=0.835,
    target_x=0.0,
    target_y=0.60,
    step_len=0.01,
    collision_gap=0.0,
)
assert r["ok"] is False
assert -1002 in r["shelf_collision_ids"]

# Same route high enough above the main shelf must pass.
r = check_transport_path_with_shelves(
    transport_item,
    packed=[],
    shelves=[main],
    start_x=0.0,
    start_y=-0.76,
    path_z=1.10,
    target_x=0.0,
    target_y=0.60,
    step_len=0.01,
    collision_gap=0.0,
)
assert r["ok"] is True

# Safe-grid wrapper with shelf list.
container001 = ContainerBounds(
    -1.0 + 0.05, 1.0 - 0.05,
    -1.52 / 2 + 0.05, 1.52 / 2 - 0.05,
    0.05, 1.62 - 0.05,
)
r = check_transport_like_safe_grid_with_shelves(
    transport_item,
    container001,
    packed=[],
    shelves=[main],
    target_x=0.0,
    target_y=0.60,
    target_z=1.10,
    cut_x=0.43,
    thickness=0.05,
    start_margin=0.018,
    safety_margin=0.015,
    extra_lift=0.0,
    step_len=0.01,
    collision_gap=0.0,
)
assert "shelf_collision_ids" in r

# ---------------------------------------------------------------------------
# Phase 4: stack support / overhang
# ---------------------------------------------------------------------------
support = make_box(500, (0.0, 0.0, 0.25), (0.65, 0.45, 0.40), mass=13.0)
upper = make_box(501, (0.0, 0.0, 0.65), (0.55, 0.40, 0.40), mass=8.0)

r = check_stack_support(
    upper, support,
    min_support_ratio=0.90,
    max_overhang=0.015,
    z_tolerance=0.03,
    enforce_mass=True,
    weight_tolerance=0.05,
)
assert r["supported"] is True
assert r["mass_ok"] is True
assert abs(r["support_ratio"] - 1.0) < 1e-12
assert r["max_actual_overhang"] == 0.0

# Too much X overhang: geometric overlap remains, but support must be rejected.
overhang = make_box(502, (0.12, 0.0, 0.65), (0.55, 0.40, 0.40), mass=8.0)
r = check_stack_support(overhang, support, min_support_ratio=0.70, max_overhang=0.015)
assert r["supported"] is False
assert r["overhang_ok"] is False
assert r["overhang_right"] > 0.015

# Wrong Z contact: upper box is floating too high.
floating = make_box(503, (0.0, 0.0, 0.72), (0.55, 0.40, 0.40), mass=8.0)
r = check_stack_support(floating, support, z_tolerance=0.03)
assert r["supported"] is False
assert r["z_ok"] is False

# Heavy-on-light is rejected when mass monotonicity is enabled.
heavy_upper = make_box(504, (0.0, 0.0, 0.65), (0.55, 0.40, 0.40), mass=18.0)
r = check_stack_support(heavy_upper, support, enforce_mass=True, weight_tolerance=0.05)
assert r["supported"] is False
assert r["mass_ok"] is False
r = check_stack_support(heavy_upper, support, enforce_mass=False)
assert r["mass_ok"] is True

# Search all packed items for a valid single-box support.
far_support = make_box(505, (0.8, 0.0, 0.25), (0.65, 0.45, 0.40), mass=13.0)
valid = find_stack_supports(upper, [far_support, support])
assert len(valid) == 1
assert valid[0]["support_id"] == 500

# Combined candidate check ignores intentional contact with support,
# but still detects collision against every other packed box.
r = check_stack_candidate(
    upper, support, container000, [support], [],
    inclusion_margin=0.005, collision_gap=0.0,
)
assert r["ok"] is True

blocker = make_box(506, (0.20, 0.0, 0.65), (0.20, 0.20, 0.20), mass=1.0)
r = check_stack_candidate(
    upper, support, container000, [support, blocker], [],
    inclusion_margin=0.005, collision_gap=0.0,
)
assert r["ok"] is False
assert 506 in r["baggage_collisions"]


# ---------------------------------------------------------------------------
# Phase 5: Python-order AABB boundary regression
# ---------------------------------------------------------------------------
a = make_box(900, (0.0, 0.245621, 0.171), (0.65, 0.35, 0.23))
b = make_box(901, (0.0, 0.5656209999999999, 0.274988), (0.65, 0.25, 0.45))
margin_xy = 0.02
dx = abs(a.cx-b.cx)
dy = abs(a.cy-b.cy)
dz = abs(a.cz-b.cz)
sx = a.lx*0.5 + b.lx*0.5 + margin_xy
sy = a.ly*0.5 + b.ly*0.5 + margin_xy
sz = a.lz*0.5 + b.lz*0.5 + 0.003
py_expected = (dx < sx and dy < sy and dz < sz)
got = aabb_overlap_python_order(a, b, margin_xy, 0.003)
dbg = aabb_overlap_python_order_debug(a, b, margin_xy, 0.003)
assert got == py_expected
assert bool(dbg['overlap']) == py_expected
assert tuple(bool(v) for v in dbg['axis_lt']) == (dx < sx, dy < sy, dz < sz)

print("all Phase 1 + Phase 2 + Phase 3 + Phase 4 + Phase 5 tests passed")
