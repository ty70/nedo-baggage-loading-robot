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
    double mass{0.0};
    bool is_soft{false};
};

struct ContainerBounds {
    double xmin, xmax;
    double ymin, ymax;
    double zmin, zmax;
};

struct ShelfGeometry {
    int id{-1000};
    std::string kind{"shelf"};
    Box box;
};

static ShelfGeometry make_small_shelf(
    double length, double width, double height,
    double thickness, double cut_x, double buffer) {

    ShelfGeometry s;
    s.id = -1001;
    s.kind = "small_shelf";
    s.box.id = s.id;
    s.box.lx = cut_x;
    s.box.ly = std::max(0.0, width - 2.0 * thickness);
    s.box.lz = thickness;
    // v17 convention: narrow shelf starts at the inner left wall.
    s.box.cx = -length * 0.5 + thickness + cut_x * 0.5;
    s.box.cy = 0.0;
    s.box.cz = height * 0.5 + buffer + thickness * 0.5;
    return s;
}

static ShelfGeometry make_main_shelf(
    double length, double width, double height,
    double thickness, double buffer) {

    ShelfGeometry s;
    s.id = -1002;
    s.kind = "main_shelf";
    s.box.id = s.id;
    s.box.lx = std::max(0.0, length - thickness);
    s.box.ly = std::max(0.0, width * 0.5 - 2.0 * thickness);
    s.box.lz = thickness;
    s.box.cx = 0.0;
    s.box.cy = width * 0.25;
    s.box.cz = height * 0.5 + buffer + thickness * 0.5;
    return s;
}

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

static std::vector<int> shelf_collision_ids_at(
    const Box& moving,
    const std::vector<ShelfGeometry>& shelves,
    double collision_gap) {

    std::vector<int> ids;
    for (const auto& s : shelves) {
        if (aabb_overlap(moving, s.box, collision_gap)) {
            ids.push_back(s.id);
        }
    }
    return ids;
}

static py::dict check_shelf_support(
    const Box& item,
    const ShelfGeometry& shelf,
    double outer_wall_clearance,
    double inward_overhang,
    double y_overhang,
    double z_tolerance) {

    const double ihx = item.lx * 0.5;
    const double ihy = item.ly * 0.5;
    const double ihz = item.lz * 0.5;
    const double shx = shelf.box.lx * 0.5;
    const double shy = shelf.box.ly * 0.5;
    const double shz = shelf.box.lz * 0.5;

    const double item_x0 = item.cx - ihx;
    const double item_x1 = item.cx + ihx;
    const double item_y0 = item.cy - ihy;
    const double item_y1 = item.cy + ihy;
    const double item_bottom = item.cz - ihz;

    const double shelf_x0 = shelf.box.cx - shx;
    const double shelf_x1 = shelf.box.cx + shx;
    const double shelf_y0 = shelf.box.cy - shy;
    const double shelf_y1 = shelf.box.cy + shy;
    const double shelf_top = shelf.box.cz + shz;

    bool x_ok = false;
    if (shelf.kind == "small_shelf") {
        // v17 rule: the small shelf touches the outer/left wall.
        // No overhang is permitted toward that wall; a small inward
        // overhang is allowed toward the container interior.
        x_ok = (item_x0 >= shelf_x0 + outer_wall_clearance &&
                item_x1 <= shelf_x1 + inward_overhang);
    } else {
        x_ok = (item_x0 >= shelf_x0 - inward_overhang &&
                item_x1 <= shelf_x1 + inward_overhang);
    }

    const bool y_ok = (item_y0 >= shelf_y0 - y_overhang &&
                       item_y1 <= shelf_y1 + y_overhang);
    const bool z_ok = std::abs(item_bottom - shelf_top) <= z_tolerance;

    py::dict out;
    out["supported"] = x_ok && y_ok && z_ok;
    out["x_ok"] = x_ok;
    out["y_ok"] = y_ok;
    out["z_ok"] = z_ok;
    out["shelf_id"] = shelf.id;
    out["shelf_kind"] = shelf.kind;
    out["shelf_top"] = shelf_top;
    out["item_bottom"] = item_bottom;
    return out;
}



static py::dict check_stack_support(
    const Box& upper,
    const Box& support,
    double min_support_ratio,
    double max_overhang,
    double z_tolerance,
    bool enforce_mass,
    double weight_tolerance) {

    const double uhx = upper.lx * 0.5;
    const double uhy = upper.ly * 0.5;
    const double uhz = upper.lz * 0.5;
    const double shx = support.lx * 0.5;
    const double shy = support.ly * 0.5;
    const double shz = support.lz * 0.5;

    const double ux0 = upper.cx - uhx;
    const double ux1 = upper.cx + uhx;
    const double uy0 = upper.cy - uhy;
    const double uy1 = upper.cy + uhy;
    const double sx0 = support.cx - shx;
    const double sx1 = support.cx + shx;
    const double sy0 = support.cy - shy;
    const double sy1 = support.cy + shy;

    const double overlap_x = std::max(0.0, std::min(ux1, sx1) - std::max(ux0, sx0));
    const double overlap_y = std::max(0.0, std::min(uy1, sy1) - std::max(uy0, sy0));
    const double overlap_area = overlap_x * overlap_y;
    const double upper_area = std::max(1e-12, upper.lx * upper.ly);
    const double support_ratio = overlap_area / upper_area;

    const double overhang_left  = std::max(0.0, sx0 - ux0);
    const double overhang_right = std::max(0.0, ux1 - sx1);
    const double overhang_front = std::max(0.0, sy0 - uy0);
    const double overhang_back  = std::max(0.0, uy1 - sy1);
    const double max_actual_overhang = std::max(
        std::max(overhang_left, overhang_right),
        std::max(overhang_front, overhang_back));

    const double upper_bottom = upper.cz - uhz;
    const double support_top = support.cz + shz;
    const double z_gap = upper_bottom - support_top;

    const bool z_ok = std::abs(z_gap) <= z_tolerance;
    const bool overlap_ok = overlap_x > 0.0 && overlap_y > 0.0;
    const bool ratio_ok = support_ratio + 1e-12 >= min_support_ratio;
    const bool overhang_ok = max_actual_overhang <= max_overhang + 1e-12;
    const bool mass_ok = (!enforce_mass || upper.mass <= 0.0 || support.mass <= 0.0 ||
                          upper.mass <= support.mass + weight_tolerance);

    py::dict out;
    out["supported"] = z_ok && overlap_ok && ratio_ok && overhang_ok && mass_ok;
    out["support_id"] = support.id;
    out["z_ok"] = z_ok;
    out["overlap_ok"] = overlap_ok;
    out["ratio_ok"] = ratio_ok;
    out["overhang_ok"] = overhang_ok;
    out["mass_ok"] = mass_ok;
    out["overlap_x"] = overlap_x;
    out["overlap_y"] = overlap_y;
    out["overlap_area"] = overlap_area;
    out["upper_footprint_area"] = upper_area;
    out["support_ratio"] = support_ratio;
    out["overhang_left"] = overhang_left;
    out["overhang_right"] = overhang_right;
    out["overhang_front"] = overhang_front;
    out["overhang_back"] = overhang_back;
    out["max_actual_overhang"] = max_actual_overhang;
    out["upper_bottom"] = upper_bottom;
    out["support_top"] = support_top;
    out["z_gap"] = z_gap;
    out["upper_mass"] = upper.mass;
    out["support_mass"] = support.mass;
    return out;
}

static py::list find_stack_supports(
    const Box& upper,
    const std::vector<Box>& packed,
    double min_support_ratio,
    double max_overhang,
    double z_tolerance,
    bool enforce_mass,
    double weight_tolerance) {

    py::list results;
    for (const auto& p : packed) {
        auto r = check_stack_support(
            upper, p, min_support_ratio, max_overhang,
            z_tolerance, enforce_mass, weight_tolerance);
        if (r["supported"].cast<bool>()) {
            results.append(r);
        }
    }
    return results;
}

static py::dict check_stack_candidate(
    const Box& upper,
    const Box& support,
    const ContainerBounds& container,
    const std::vector<Box>& packed,
    const std::vector<ShelfGeometry>& shelves,
    double inclusion_margin,
    double collision_gap,
    double min_support_ratio,
    double max_overhang,
    double z_tolerance,
    bool enforce_mass,
    double weight_tolerance) {

    py::dict out;
    const bool inside = inside_container(upper, container, inclusion_margin);
    auto support_result = check_stack_support(
        upper, support, min_support_ratio, max_overhang,
        z_tolerance, enforce_mass, weight_tolerance);

    std::vector<int> baggage_collisions;
    if (inside) {
        for (const auto& p : packed) {
            if (p.id == support.id) continue; // intended contact surface
            if (aabb_overlap(upper, p, collision_gap)) baggage_collisions.push_back(p.id);
        }
    }

    std::vector<int> shelf_collisions;
    if (inside) shelf_collisions = shelf_collision_ids_at(upper, shelves, collision_gap);

    const bool collision_free = baggage_collisions.empty() && shelf_collisions.empty();
    const bool supported = support_result["supported"].cast<bool>();
    out["inside"] = inside;
    out["supported"] = supported;
    out["support"] = support_result;
    out["baggage_collisions"] = baggage_collisions;
    out["shelf_collisions"] = shelf_collisions;
    out["collision_free"] = collision_free;
    out["ok"] = inside && supported && collision_free;
    return out;
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

static py::dict check_candidate_with_shelves(
    const Box& candidate,
    const ContainerBounds& container,
    const std::vector<Box>& packed,
    const std::vector<ShelfGeometry>& shelves,
    double inclusion_margin,
    double collision_gap) {

    py::dict out;
    const bool inside = inside_container(candidate, container, inclusion_margin);
    out["inside"] = inside;

    std::vector<int> baggage_collisions;
    std::vector<int> shelf_collisions;
    if (inside) {
        baggage_collisions = collision_ids_at(candidate, packed, collision_gap);
        shelf_collisions = shelf_collision_ids_at(candidate, shelves, collision_gap);
    }

    out["baggage_collisions"] = baggage_collisions;
    out["shelf_collisions"] = shelf_collisions;
    out["collision_free"] = baggage_collisions.empty() && shelf_collisions.empty();
    out["ok"] = inside && baggage_collisions.empty() && shelf_collisions.empty();
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

static py::dict check_transport_path_with_shelves(
    const Box& item,
    const std::vector<Box>& packed,
    const std::vector<ShelfGeometry>& shelves,
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

    auto check_here = [&](const std::string& segment, int segment_step,
                          double x, double y, bool do_check) -> py::object {
        if (!do_check) return py::none();
        moving.cx = x;
        moving.cy = y;
        auto bag_ids = collision_ids_at(moving, packed, collision_gap);
        auto shelf_ids = shelf_collision_ids_at(moving, shelves, collision_gap);
        if (!bag_ids.empty() || !shelf_ids.empty()) {
            py::dict hit;
            hit["ok"] = false;
            hit["segment"] = segment;
            hit["step_index"] = global_step;
            hit["segment_step_index"] = segment_step;
            hit["collision_ids"] = bag_ids;
            hit["shelf_collision_ids"] = shelf_ids;
            hit["collision_position"] = py::make_tuple(x, y, path_z);
            hit["checked_steps"] = global_step + 1;
            return std::move(hit);
        }
        ++global_step;
        return py::none();
    };

    const int ny = segment_steps(start_y, target_y, step_len);
    for (int i = 0; i <= ny; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(ny);
        const double y = start_y + (target_y - start_y) * u;
        const bool do_check = !(i == 0 && !include_start);
        auto hit = check_here("Y", i, start_x, y, do_check);
        if (!hit.is_none()) return hit.cast<py::dict>();
    }

    const int nx = segment_steps(start_x, target_x, step_len);
    for (int i = 1; i <= nx; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(nx);
        const double x = start_x + (target_x - start_x) * u;
        const bool do_check = !(i == nx && !include_target);
        auto hit = check_here("X", i, x, target_y, do_check);
        if (!hit.is_none()) return hit.cast<py::dict>();
    }

    out["ok"] = true;
    out["segment"] = py::none();
    out["step_index"] = py::none();
    out["segment_step_index"] = py::none();
    out["collision_ids"] = std::vector<int>{};
    out["shelf_collision_ids"] = std::vector<int>{};
    out["collision_position"] = py::none();
    out["checked_steps"] = global_step;
    return out;
}

static py::dict check_transport_like_safe_grid_with_shelves(
    const Box& item,
    const ContainerBounds& container,
    const std::vector<Box>& packed,
    const std::vector<ShelfGeometry>& shelves,
    double target_x, double target_y, double target_z,
    double cut_x, double thickness,
    double start_margin, double safety_margin, double extra_lift,
    double step_len, double collision_gap) {

    const double hx = item.lx * 0.5;
    const double hz = item.lz * 0.5;
    const double side_margin = std::max(start_margin, safety_margin + 0.003);
    const double x_min = container.xmin + cut_x + hx + side_margin;
    const double x_max = container.xmax - hx - side_margin;
    const double start_x = std::min(std::max(target_x, x_min), x_max);
    const double start_y = container.ymin - thickness;

    const double floor_bottom = target_z - hz;
    const double floor_z = thickness;
    double path_z;
    if (0.0 <= floor_bottom - floor_z && floor_bottom - floor_z <= 0.05) {
        path_z = target_z;
    } else {
        path_z = std::min(target_z + 0.08 + extra_lift,
                          container.zmax - hz - start_margin);
    }

    auto out = check_transport_path_with_shelves(
        item, packed, shelves, start_x, start_y, path_z,
        target_x, target_y, step_len, collision_gap, true, true);
    out["start_x"] = start_x;
    out["start_y"] = start_y;
    out["path_z"] = path_z;
    out["target_x"] = target_x;
    out["target_y"] = target_y;
    out["target_z"] = target_z;
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
    m.doc() = "Lightweight axis-aligned baggage checker with shelves and stack support";

    py::class_<Box>(m, "Box")
        .def(py::init<>())
        .def_readwrite("id", &Box::id)
        .def_readwrite("cx", &Box::cx)
        .def_readwrite("cy", &Box::cy)
        .def_readwrite("cz", &Box::cz)
        .def_readwrite("lx", &Box::lx)
        .def_readwrite("ly", &Box::ly)
        .def_readwrite("lz", &Box::lz)
        .def_readwrite("mass", &Box::mass)
        .def_readwrite("is_soft", &Box::is_soft);

    py::class_<ShelfGeometry>(m, "ShelfGeometry")
        .def(py::init<>())
        .def_readwrite("id", &ShelfGeometry::id)
        .def_readwrite("kind", &ShelfGeometry::kind)
        .def_readwrite("box", &ShelfGeometry::box);

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

    m.def("make_small_shelf", &make_small_shelf,
          py::arg("length"), py::arg("width"), py::arg("height"),
          py::arg("thickness"), py::arg("cut_x"), py::arg("buffer") = 0.0);
    m.def("make_main_shelf", &make_main_shelf,
          py::arg("length"), py::arg("width"), py::arg("height"),
          py::arg("thickness"), py::arg("buffer") = 0.0);
    m.def("check_shelf_support", &check_shelf_support,
          py::arg("item"), py::arg("shelf"),
          py::arg("outer_wall_clearance") = 0.006,
          py::arg("inward_overhang") = 0.006,
          py::arg("y_overhang") = 0.0,
          py::arg("z_tolerance") = 0.03);
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
    m.def("check_candidate_with_shelves", &check_candidate_with_shelves,
          py::arg("candidate"), py::arg("container"), py::arg("packed"), py::arg("shelves"),
          py::arg("inclusion_margin") = 0.0,
          py::arg("collision_gap") = 0.0);
    m.def("check_stack_support", &check_stack_support,
          py::arg("upper"), py::arg("support"),
          py::arg("min_support_ratio") = 0.90,
          py::arg("max_overhang") = 0.015,
          py::arg("z_tolerance") = 0.03,
          py::arg("enforce_mass") = true,
          py::arg("weight_tolerance") = 0.05);
    m.def("find_stack_supports", &find_stack_supports,
          py::arg("upper"), py::arg("packed"),
          py::arg("min_support_ratio") = 0.90,
          py::arg("max_overhang") = 0.015,
          py::arg("z_tolerance") = 0.03,
          py::arg("enforce_mass") = true,
          py::arg("weight_tolerance") = 0.05);
    m.def("check_stack_candidate", &check_stack_candidate,
          py::arg("upper"), py::arg("support"), py::arg("container"),
          py::arg("packed"), py::arg("shelves") = std::vector<ShelfGeometry>{},
          py::arg("inclusion_margin") = 0.005,
          py::arg("collision_gap") = 0.0,
          py::arg("min_support_ratio") = 0.90,
          py::arg("max_overhang") = 0.015,
          py::arg("z_tolerance") = 0.03,
          py::arg("enforce_mass") = true,
          py::arg("weight_tolerance") = 0.05);
    m.def("check_transport_path", &check_transport_path,
          py::arg("item"), py::arg("packed"),
          py::arg("start_x"), py::arg("start_y"), py::arg("path_z"),
          py::arg("target_x"), py::arg("target_y"),
          py::arg("step_len") = 0.01,
          py::arg("collision_gap") = 0.015,
          py::arg("include_start") = true,
          py::arg("include_target") = true);
    m.def("check_transport_path_with_shelves", &check_transport_path_with_shelves,
          py::arg("item"), py::arg("packed"), py::arg("shelves"),
          py::arg("start_x"), py::arg("start_y"), py::arg("path_z"),
          py::arg("target_x"), py::arg("target_y"),
          py::arg("step_len") = 0.01,
          py::arg("collision_gap") = 0.015,
          py::arg("include_start") = true,
          py::arg("include_target") = true);
    m.def("check_transport_like_safe_grid_with_shelves", &check_transport_like_safe_grid_with_shelves,
          py::arg("item"), py::arg("container"), py::arg("packed"), py::arg("shelves"),
          py::arg("target_x"), py::arg("target_y"), py::arg("target_z"),
          py::arg("cut_x"), py::arg("thickness"),
          py::arg("start_margin") = 0.018,
          py::arg("safety_margin") = 0.015,
          py::arg("extra_lift") = 0.0,
          py::arg("step_len") = 0.01,
          py::arg("collision_gap") = 0.015);
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
