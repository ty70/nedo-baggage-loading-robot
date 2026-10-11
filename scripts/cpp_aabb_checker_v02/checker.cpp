#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace py = pybind11;

struct Box {
    int id{-1};
    double cx{0.0}, cy{0.0}, cz{0.0};
    double lx{0.0}, ly{0.0}, lz{0.0};
};

struct ContainerBounds {
    double xmin, xmax;
    double ymin, ymax;
    double zmin, zmax;
};

static std::tuple<double,double,double> oriented_dims(
    double L, double W, double H, int orientation) {
    switch (orientation) {
        case 0: return {L, W, H};
        case 1: return {L, H, W};
        case 2: return {H, W, L};
        case 3: return {W, L, H};
        case 4: return {W, H, L};
        case 5: return {H, L, W};
        default: throw std::invalid_argument("orientation must be 0..5");
    }
}

static bool inside_container(const Box& b, const ContainerBounds& c, double margin) {
    const double hx = b.lx * 0.5;
    const double hy = b.ly * 0.5;
    const double hz = b.lz * 0.5;
    return (b.cx - hx >= c.xmin + margin &&
            b.cx + hx <= c.xmax - margin &&
            b.cy - hy >= c.ymin + margin &&
            b.cy + hy <= c.ymax - margin &&
            b.cz - hz >= c.zmin + margin &&
            b.cz + hz <= c.zmax - margin);
}

static bool aabb_overlap(const Box& a, const Box& b, double gap) {
    const double sx = (a.lx + b.lx) * 0.5 + gap;
    const double sy = (a.ly + b.ly) * 0.5 + gap;
    const double sz = (a.lz + b.lz) * 0.5 + gap;

    return (std::abs(a.cx - b.cx) < sx &&
            std::abs(a.cy - b.cy) < sy &&
            std::abs(a.cz - b.cz) < sz);
}

static std::vector<int> collision_ids_at(
    const Box& moving,
    const std::vector<Box>& packed,
    double collision_gap) {

    std::vector<int> ids;
    for (const auto& p : packed) {
        if (aabb_overlap(moving, p, collision_gap)) {
            ids.push_back(p.id);
        }
    }
    return ids;
}

static py::dict check_candidate(
    const Box& candidate,
    const ContainerBounds& container,
    const std::vector<Box>& packed,
    double inclusion_margin,
    double collision_gap) {

    py::dict out;
    const bool inside = inside_container(candidate, container, inclusion_margin);
    out["inside"] = inside;

    std::vector<int> collisions;
    if (inside) {
        collisions = collision_ids_at(candidate, packed, collision_gap);
    }

    out["collisions"] = collisions;
    out["collision_free"] = collisions.empty();
    out["ok"] = inside && collisions.empty();
    return out;
}

static int segment_steps(double a, double b, double step_len) {
    if (step_len <= 0.0) {
        throw std::invalid_argument("step_len must be > 0");
    }
    const double d = std::abs(b - a);
    return std::max(1, static_cast<int>(std::ceil(d / step_len)));
}

static py::dict check_transport_path(
    const Box& item,
    const std::vector<Box>& packed,
    double start_x,
    double start_y,
    double path_z,
    double target_x,
    double target_y,
    double step_len,
    double collision_gap,
    bool include_start,
    bool include_target) {

    py::dict out;
    Box moving = item;
    moving.cz = path_z;

    int global_step = 0;

    auto make_hit = [&](const std::string& segment,
                        int segment_step,
                        double x,
                        double y,
                        const std::vector<int>& ids) {
        py::dict hit;
        hit["ok"] = false;
        hit["segment"] = segment;
        hit["step_index"] = global_step;
        hit["segment_step_index"] = segment_step;
        hit["collision_ids"] = ids;
        hit["collision_position"] = py::make_tuple(x, y, path_z);
        hit["checked_steps"] = global_step + 1;
        return hit;
    };

    auto check_at = [&](const std::string& segment,
                        int segment_step,
                        double x,
                        double y,
                        bool do_check,
                        py::dict& hit_out) -> bool {
        moving.cx = x;
        moving.cy = y;
        if (!do_check) {
            return false;
        }
        const auto ids = collision_ids_at(moving, packed, collision_gap);
        if (!ids.empty()) {
            hit_out = make_hit(segment, segment_step, x, y, ids);
            return true;
        }
        ++global_step;
        return false;
    };

    // Segment 1: move along Y at fixed start_x.
    const int ny = segment_steps(start_y, target_y, step_len);
    for (int i = 0; i <= ny; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(ny);
        const double y = start_y + (target_y - start_y) * u;
        const bool is_start = (i == 0);
        const bool is_y_target = (i == ny);
        bool do_check = true;
        if (is_start && !include_start) do_check = false;
        // The Y-segment endpoint is the X-segment start, so keep it checked.
        // include_target only controls the final target of segment 2.
        (void)is_y_target;
        py::dict hit;
        if (check_at("Y", i, start_x, y, do_check, hit)) {
            return hit;
        }
    }

    // Segment 2: move along X at fixed target_y.
    const int nx = segment_steps(start_x, target_x, step_len);
    for (int i = 1; i <= nx; ++i) { // i=0 was already checked as Y endpoint
        const double u = static_cast<double>(i) / static_cast<double>(nx);
        const double x = start_x + (target_x - start_x) * u;
        const bool is_final = (i == nx);
        const bool do_check = !(is_final && !include_target);
        py::dict hit;
        if (check_at("X", i, x, target_y, do_check, hit)) {
            return hit;
        }
    }

    out["ok"] = true;
    out["segment"] = py::none();
    out["step_index"] = py::none();
    out["segment_step_index"] = py::none();
    out["collision_ids"] = std::vector<int>{};
    out["collision_position"] = py::none();
    out["checked_steps"] = global_step;
    return out;
}

static py::dict check_transport_like_safe_grid(
    const Box& item,
    const ContainerBounds& container,
    const std::vector<Box>& packed,
    double target_x,
    double target_y,
    double target_z,
    double cut_x,
    double thickness,
    double start_margin,
    double safety_margin,
    double extra_lift,
    double step_len,
    double collision_gap) {

    const double hx = item.lx * 0.5;
    const double hz = item.lz * 0.5;

    const double side_margin = std::max(start_margin, safety_margin + 0.003);
    const double x_min = container.xmin + cut_x + hx + side_margin;
    const double x_max = container.xmax - hx - side_margin;
    const double start_x = std::min(std::max(target_x, x_min), x_max);
    const double start_y = container.ymin - thickness; // equivalent to local -W/2

    const double floor_bottom = target_z - hz;
    const double floor_z = thickness;
    double path_z;
    if (0.0 <= floor_bottom - floor_z && floor_bottom - floor_z <= 0.05) {
        path_z = target_z;
    } else {
        path_z = std::min(
            target_z + 0.08 + extra_lift,
            container.zmax - hz - start_margin);
    }

    auto out = check_transport_path(
        item, packed,
        start_x, start_y, path_z,
        target_x, target_y,
        step_len, collision_gap,
        true, true);

    out["start_x"] = start_x;
    out["start_y"] = start_y;
    out["path_z"] = path_z;
    out["target_x"] = target_x;
    out["target_y"] = target_y;
    out["target_z"] = target_z;
    return out;
}

PYBIND11_MODULE(cpp_aabb_checker, m) {
    m.doc() = "Lightweight axis-aligned baggage checker with Y->X transport sweep";

    py::class_<Box>(m, "Box")
        .def(py::init<>())
        .def_readwrite("id", &Box::id)
        .def_readwrite("cx", &Box::cx)
        .def_readwrite("cy", &Box::cy)
        .def_readwrite("cz", &Box::cz)
        .def_readwrite("lx", &Box::lx)
        .def_readwrite("ly", &Box::ly)
        .def_readwrite("lz", &Box::lz);

    py::class_<ContainerBounds>(m, "ContainerBounds")
        .def(py::init<double,double,double,double,double,double>(),
             py::arg("xmin"), py::arg("xmax"),
             py::arg("ymin"), py::arg("ymax"),
             py::arg("zmin"), py::arg("zmax"))
        .def_readwrite("xmin", &ContainerBounds::xmin)
        .def_readwrite("xmax", &ContainerBounds::xmax)
        .def_readwrite("ymin", &ContainerBounds::ymin)
        .def_readwrite("ymax", &ContainerBounds::ymax)
        .def_readwrite("zmin", &ContainerBounds::zmin)
        .def_readwrite("zmax", &ContainerBounds::zmax);

    m.def("oriented_dims", &oriented_dims,
          py::arg("length"), py::arg("width"), py::arg("height"), py::arg("orientation"));
    m.def("inside_container", &inside_container,
          py::arg("box"), py::arg("container"), py::arg("margin") = 0.0);
    m.def("aabb_overlap", &aabb_overlap,
          py::arg("a"), py::arg("b"), py::arg("gap") = 0.0);
    m.def("check_candidate", &check_candidate,
          py::arg("candidate"), py::arg("container"), py::arg("packed"),
          py::arg("inclusion_margin") = 0.0,
          py::arg("collision_gap") = 0.0);
    m.def("check_transport_path", &check_transport_path,
          py::arg("item"), py::arg("packed"),
          py::arg("start_x"), py::arg("start_y"), py::arg("path_z"),
          py::arg("target_x"), py::arg("target_y"),
          py::arg("step_len") = 0.01,
          py::arg("collision_gap") = 0.015,
          py::arg("include_start") = true,
          py::arg("include_target") = true);
    m.def("check_transport_like_safe_grid", &check_transport_like_safe_grid,
          py::arg("item"), py::arg("container"), py::arg("packed"),
          py::arg("target_x"), py::arg("target_y"), py::arg("target_z"),
          py::arg("cut_x"), py::arg("thickness"),
          py::arg("start_margin") = 0.018,
          py::arg("safety_margin") = 0.015,
          py::arg("extra_lift") = 0.0,
          py::arg("step_len") = 0.01,
          py::arg("collision_gap") = 0.015);
}
