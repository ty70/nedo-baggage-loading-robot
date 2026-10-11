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
        for (const auto& p : packed) {
            if (aabb_overlap(candidate, p, collision_gap)) {
                collisions.push_back(p.id);
            }
        }
    }

    out["collisions"] = collisions;
    out["collision_free"] = collisions.empty();
    out["ok"] = inside && collisions.empty();
    return out;
}

PYBIND11_MODULE(cpp_aabb_checker, m) {
    m.doc() = "Lightweight axis-aligned baggage checker";

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
}
