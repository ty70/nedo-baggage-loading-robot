from cpp_aabb_checker import Box, ContainerBounds, oriented_dims, inside_container, aabb_overlap, check_candidate


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

# Far-away box should not overlap.
b1 = make_box(17, (-0.5, -0.4, 0.326), (0.75, 0.27, 0.56))
assert not aabb_overlap(b0, b1, 0.0)

# Nearly same location should overlap.
b2 = make_box(99, (0.60, 0.53, 0.326), (0.55, 0.24, 0.40))
assert aabb_overlap(b0, b2, 0.0)

r = check_candidate(b2, container, [b0, b1], inclusion_margin=0.005, collision_gap=0.0)
assert r["inside"] is True
assert r["collision_free"] is False
assert r["ok"] is False
assert 3 in r["collisions"]

print("all tests passed")
