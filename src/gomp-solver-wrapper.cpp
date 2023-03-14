#ifndef GOMP_SOLVER_WRAPPER_H
#define GOMP_SOLVER_WRAPPER_H

#include <tuple>
#include <vector>
#include <array>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

// def solve(start_pos_joints: Tuple[float x 6], end_pos_tcp: Tuple[float x 3], time_step: float, waypoints_count: int, velocity_constraints: Tuple[List[float x 6], List[float x 6]], acceleration_constraints: Tuple[List[float x 6], List[float x 6]], position_constraints: Tuple[List[float x 6], List[float x 6]], obstacles: List[Obstacle]) -> Tuple[List[float x 6], List[float x 6], List[float x 6]]

struct Obstacle {
    int x;
};

std::tuple<std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>> solve(
    std::array<float, 6> start_pos_joints, 
    std::array<float, 3> end_pos_tcp, 
    float time_step, 
    int waypoints_count, 
    std::tuple<std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>> velocity_constraints, 
    std::tuple<std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>> acceleration_constraints,
    std::tuple<std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>> position_constraints,
    std::vector<Obstacle> obstacles) {
        return {{{1,2,3,4,5,6}}, {{1,2,3,4,5,6}}, {{1,2,3,4,5,6}}};
    }

PYBIND11_MODULE(gomp, m) {
    pybind11::class_<Obstacle>(m, "Obstacle")
        .def(pybind11::init<int>())
        .def_readwrite("x", &Obstacle::x);

    m.def("solve", &solve, "Runs the GOMP solver.");
}

// Compile using:
// g++ -O3 -Wall -shared -std=c++11 -fPIC `python3 -m pybind11 --includes` src/gomp-solver-wrapper.cpp -o gomp`python3-config --extension-suffix`

#endif // GOMP_SOLVER_WRAPPER_H