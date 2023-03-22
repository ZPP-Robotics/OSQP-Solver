#include <tuple>
#include <vector>
#include <array>

#include "../constraints/constraints.h"

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

std::tuple<std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>> solve_1(
    std::array<float, 6> start_pos_joints, 
    std::array<float, 6> end_pos_joints, 
    float time_step, 
    int waypoints_count, 
    constraint_t velocity_constraints, 
    constraint_t acceleration_constraints,
    constraint_t position_constraints,
    std::vector<std::tuple<point_t, point_t>> obstacles) {

        constraints::Constraint<6> velocity_constraints_transformed{std::get<0>(velocity_constraints), std::get<1>(velocity_constraints)};
        constraints::Constraint<6> acceleration_constraints_transformed{std::get<0>(acceleration_constraints), std::get<1>(acceleration_constraints)};
        constraints::Constraint<6> position_constraints_transformed{std::get<0>(position_constraints), std::get<1>(position_constraints)};



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