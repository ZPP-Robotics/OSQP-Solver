#include <tuple>
#include <vector>
#include <array>

#include "../constraints/constraints.h"
#include "../constraints/constraint-builder.h"
#include "../horizontal-line.h"

#include "../../Kinematics-UR5e-arm/src/analytical_ik.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;

// struct Obstacle {
//     int x;
// };

// def solve(start_pos_joints: Tuple[float x 6], end_pos_tcp: Tuple[float x 3], time_step: float, waypoints_count: int, velocity_constraints: Tuple[Tuple[float x 6], Tuple[float x 6]], acceleration_constraints: Tuple[Tuple[float x 6], Tuple[float x 6]], position_constraints: Tuple[Tuple[float x 6], Tuple[float x 6]], obstacles: List[Tuple[Tuple[float x 3], Tuple[float x 3]]]) -> Tuple[List[float x 6], List[float x 6], List[float x 6]]

const size_t N_DIM = 6;

using single_constraint_n_dim_t = std::array<float, N_DIM>;
using constraint_n_dim_t = std::tuple<single_constraint_n_dim_t, single_constraint_n_dim_t>;

using single_constraint_3d_t = std::array<float, 3>;
using constraint_3d_t = std::tuple<single_constraint_3d_t, single_constraint_3d_t>;

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

template<size_t N>
constraints::Bound<N> createBound(const std::array<float, N>& arr) {
    const Eigen::Map<const Eigen::Array<float, N, 1>> arr_map(arr.data());

    const Eigen::Array<double, N, 1> double_arr = arr_map.template cast<double>();
    
    return Eigen::Vector<double, N>(double_arr);
}

template<size_t N>
constraints::Constraint<N> createConstraint(constraint_n_dim_t& constraint) {
    return {createBound<N>(std::get<0>(constraint)), createBound<N>(std::get<1>(constraint))};
}

std::vector<HorizontalLine> createHorizontalLines(std::vector<std::tuple<point_t, point_t>>& obstacles) {
    std::vector<HorizontalLine> horizontal_lines;

    for (auto& obstacle : obstacles) {
        point_t direction = std::get<0>(obstacle);
        point_t point = std::get<1>(obstacle);

        QPVector2d direction_vec{std::get<0>(direction), std::get<1>(direction)};
        QPVector3d point_vec{std::get<0>(point), std::get<1>(point), std::get<2>(point)};

        horizontal_lines.push_back(HorizontalLine(direction_vec, point_vec));
    }
    return horizontal_lines;
}

std::tuple<std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>> solve_1(
    std::array<float, 6> start_pos_joints, 
    std::array<float, 6> end_pos_joints, 
    float time_step, 
    int waypoints_count, 
    constraint_n_dim_t velocity_constraints, 
    constraint_n_dim_t acceleration_constraints,
    constraint_n_dim_t position_constraints,
    constraint_3d_t constraints_3d,
    std::vector<std::tuple<point_t, point_t>> obstacles) {

        constraints::Constraint<N_DIM> velocity_constraints_transformed = createConstraint<N_DIM>(velocity_constraints);
        constraints::Constraint<N_DIM> acceleration_constraints_transformed = createConstraint<N_DIM>(acceleration_constraints);
        constraints::Constraint<N_DIM> position_constraints_transformed = createConstraint<N_DIM>(position_constraints);
        
        std::vector<HorizontalLine> obstacles_transformed = createHorizontalLines(obstacles);

        std::vector<std::pair<ForwardKinematicsFun, JacobianFun>> mappers{ 
            {&forward_kinematics_6_back, &joint_jacobian_6_back},
            {&forward_kinematics, &joint_jacobian},
        };



        return {{{start_pos_joints.at(0),2,3,4,5,6}}, {{1,2,3,4,5,6}}, {{1,2,3,4,5,6}}};
    }

PYBIND11_MODULE(gomp, m) {
    // pybind11::class_<Obstacle>(m, "Obstacle")
    //     .def(pybind11::init<int>())
    //     .def_readwrite("x", &Obstacle::x);

    // m.def("solve_0", &solve_0, "Runs the GOMP solver.");

    m.def("solve_1", &solve_1, "Runs the GOMP solver.");
}

// Compile using (now compiled with CMakeLists.txt):
// g++ -O3 -Wall -shared -std=gnu++11 `python3-config --cflags --ldflags --libs` ../src/bindings/gomp-solver-binding.cpp -o gomp.so -fPIC -I/home/olaf/anaconda3/lib/python3.9/site-packages/pybind11/include

// Shared object library will be created. Make sure to rename the result file to: gomp.so, and move to the directory with test files.