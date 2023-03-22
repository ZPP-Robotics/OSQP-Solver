#include <tuple>
#include <vector>
#include <array>

#include "../constraints/constraints.h"
#include "../horizontal-line.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;

// struct Obstacle {
//     int x;
// };

// def solve(start_pos_joints: Tuple[float x 6], end_pos_tcp: Tuple[float x 3], time_step: float, waypoints_count: int, velocity_constraints: Tuple[Tuple[float x 6], Tuple[float x 6]], acceleration_constraints: Tuple[Tuple[float x 6], Tuple[float x 6]], position_constraints: Tuple[Tuple[float x 6], Tuple[float x 6]], obstacles: List[Tuple[Tuple[float x 3], Tuple[float x 3]]]) -> Tuple[List[float x 6], List[float x 6], List[float x 6]]

const size_t N_DIM = 6;

using single_constraint_t = std::array<float, 6>;
using constraint_t = std::tuple<single_constraint_t, single_constraint_t>;
using point_t = std::tuple<float, float, float>;

// std::tuple<std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>> solve_0(
//     std::array<float, 6> start_pos_joints, 
//     std::array<float, 3> end_pos_tcp, 
//     float time_step, 
//     int waypoints_count, 
//     constraint_t velocity_constraints, 
//     constraint_t acceleration_constraints,
//     constraint_t position_constraints,
//     std::vector<Obstacle> obstacles) {
//         return {{{start_pos_joints.at(0),2,3,4,5,6}}, {{1,2,3,4,5,6}}, {{1,2,3,4,5,6}}};
//     }

constraints::Bound<N_DIM> createBound(const std::array<float, 6>& arr) {
    const Eigen::Map<const Eigen::Array<float, 6, 1>> arr_map(arr.data());

    const auto double_arr = arr_map.cast<double>();
    
    return Eigen::Vector<double, 6>(double_arr);
}

constraints::Constraint<N_DIM> createConstraint(constraint_t& constraint) {
    return {createBound(std::get<0>(constraint)), createBound(std::get<1>(constraint))};
}

std::vector<HorizontalLine> createHorizontalLines(std::vector<std::tuple<point_t, point_t>>& obstacles) {
    std::vector<HorizontalLine> horizontal_lines;
    for (auto& obstacle : obstacles) {
        point_t direction = std::get<0>(obstacle);
        point_t point = std::get<1>(obstacle);

        QPVector2D direction_vec{std::get<0>(direction), std::get<1>(direction)};
        QPVector3D point_vec{std::get<0>(point), std::get<1>(point), std::get<2>(point)};

        horizontal_lines.push_back(HorizontalLine(direction_vec, point_vec));
    }
    return horizontal_lines;
}

std::tuple<std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>> solve_1(
    std::array<float, 6> start_pos_joints, 
    std::array<float, 6> end_pos_joints, 
    float time_step, 
    int waypoints_count, 
    constraint_t velocity_constraints, 
    constraint_t acceleration_constraints,
    constraint_t position_constraints,
    std::vector<std::tuple<point_t, point_t>> obstacles) {

        constraints::Constraint<N_DIM> velocity_constraints_transformed = createConstraint(velocity_constraints);
        constraints::Constraint<N_DIM> acceleration_constraints_transformed = createConstraint(acceleration_constraints);
        constraints::Constraint<N_DIM> position_constraints_transformed = createConstraint(position_constraints);
        
        

        return {{{start_pos_joints.at(0),2,3,4,5,6}}, {{1,2,3,4,5,6}}, {{1,2,3,4,5,6}}};
    }

PYBIND11_MODULE(gomp, m) {
    // pybind11::class_<Obstacle>(m, "Obstacle")
    //     .def(pybind11::init<int>())
    //     .def_readwrite("x", &Obstacle::x);

    // m.def("solve_0", &solve_0, "Runs the GOMP solver.");

    m.def("solve_1", &solve_1, "Runs the GOMP solver.");
}

// Compile using:
// g++ -O3 -Wall -shared -std=gnu++11 `python3-config --cflags --ldflags --libs` ../src/bindings/gomp-solver-binding.cpp -o gomp.so -fPIC -I/home/olaf/anaconda3/lib/python3.9/site-packages/pybind11/include