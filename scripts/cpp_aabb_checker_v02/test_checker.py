from cpp_aabb_checker import (
    Box,
    ContainerBounds,
    oriented_dims,
    inside_container,
    aabb_overlap,
    check_candidate,
    check_transport_path,
    check_transport_like_safe_grid,
)


def make_box(idx, pos, dims):
    b = Box()
    b.id = idx
    b.cx, b.cy, b.cz = map(float, pos)
    b.lx, b.ly, b.lz = map(float, dims)
    return b


# case000 approximate physical inner box: L=2.0, W=1.45, H=1.61, t=0.04
container = ContainerBounds(
    -1.0 + 0.04, 1.0 - 0.04,
    -1.45 / 2 + 0.04, 1.45 / 2 - 0.04,
    0.04, 1.61 - 0.04,
)

assert oriented_dims(0.75, 0.56, 0.27, 1) == (0.75, 0.27, 0.56)

b0 = make_box(3, (0.567, 0.530, 0.326), (0.75, 0.27, 0.56))
assert inside_container(b0, container, 0.005)

b1 = make_box(17, (-0.5, -0.4, 0.326), (0.75, 0.27, 0.56))
assert not aabb_overlap(b0, b1, 0.0)

b2 = make_box(99, (0.60, 0.53, 0.326), (0.55, 0.24, 0.40))
assert aabb_overlap(b0, b2, 0.0)

r = check_candidate(b2, container, [b0, b1], inclusion_margin=0.005, collision_gap=0.0)
assert r["inside"] is True
assert r["collision_free"] is False
assert r["ok"] is False
assert 3 in r["collisions"]

# --- Phase 2: Y -> X transport tests ---
# A blocker directly in the Y corridor.
moving = make_box(38, (0, 0, 0), (0.55, 0.24, 0.40))
block_y = make_box(32, (-0.30, -0.37, 0.40), (0.24, 0.40, 0.55))

r = check_transport_path(
    moving,
    [block_y],
    start_x=-0.30,
    start_y=-0.725,
    path_z=0.40,
    target_x=0.50,
    target_y=0.30,
    step_len=0.01,
    collision_gap=0.0,
)
assert r["ok"] is False
assert r["segment"] == "Y"
assert 32 in r["collision_ids"]

# No blocker: both segments must pass.
r = check_transport_path(
    moving,
    [],
    start_x=-0.30,
    start_y=-0.725,
    path_z=0.40,
    target_x=0.50,
    target_y=0.30,
    step_len=0.01,
    collision_gap=0.0,
)
assert r["ok"] is True
assert r["collision_ids"] == []

# Block only the X segment after reaching target_y.
block_x = make_box(77, (0.10, 0.30, 0.40), (0.20, 0.20, 0.40))
r = check_transport_path(
    moving,
    [block_x],
    start_x=-0.30,
    start_y=-0.725,
    path_z=0.40,
    target_x=0.50,
    target_y=0.30,
    step_len=0.01,
    collision_gap=0.0,
)
assert r["ok"] is False
assert r["segment"] == "X"
assert 77 in r["collision_ids"]

# Safe-grid-compatible wrapper computes clipped start_x and +0.08 path lift.
r = check_transport_like_safe_grid(
    moving,
    container,
    [],
    target_x=0.50,
    target_y=0.30,
    target_z=0.80,
    cut_x=0.44,
    thickness=0.04,
    start_margin=0.018,
    safety_margin=0.015,
    extra_lift=0.0,
    step_len=0.01,
    collision_gap=0.015,
)
assert r["ok"] is True
assert abs(r["path_z"] - 0.88) < 1e-9

print("all Phase 1 + Phase 2 tests passed")
