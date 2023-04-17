#include <tuple>
#include <vector>
#include <array>

#include "../constraints/constraints.h"
#include "../constraints/constraint-builder.h"
#include "../horizontal-line.h"
#include "../gomp-solver.h"

#include "../../Kinematics-UR5e-arm/src/analytical_ik.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;

// def solve(start_pos_joints: Tuple[float x 6], end_pos_tcp: Tuple[float x 3], time_step: float, waypoints_count: int, velocity_constraints: Tuple[Tuple[float x 6], Tuple[float x 6]], acceleration_constraints: Tuple[Tuple[float x 6], Tuple[float x 6]], position_constraints: Tuple[Tuple[float x 6], Tuple[float x 6]], obstacles: List[Tuple[Tuple[float x 3], Tuple[float x 3]]]) -> Tuple[List[float x 6], List[float x 6], List[float x 6]]

const size_t N_DIM = 6;

template<size_t N>
using single_constraint_t = std::array<float, N>;

template<size_t N>
using constraint_t = std::tuple<single_constraint_t<N>, single_constraint_t<N>>;

using point_t = std::tuple<float, float, float>;

using horizontal_line_t = std::tuple<point_t, point_t, bool>;

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
constraints::Constraint<N> createConstraint(constraint_t<N>& constraint) {
    return {createBound<N>(std::get<0>(constraint)), createBound<N>(std::get<1>(constraint))};
}

std::vector<HorizontalLine> createHorizontalLines(std::vector<horizontal_line_t>& obstacles) {
    std::vector<HorizontalLine> horizontal_lines;

    for (auto& obstacle : obstacles) {
        point_t direction = std::get<0>(obstacle);
        point_t point = std::get<1>(obstacle);
        bool bypass_from_below = std::get<2>(obstacle);

        QPVector2d direction_vec{std::get<0>(direction), std::get<1>(direction)};

        QPVector3d point_vec{std::get<0>(point), std::get<1>(point), 
            std::get<2>(point)};

        horizontal_lines.push_back(HorizontalLine(direction_vec, 
            point_vec, bypass_from_below));
    }
    return horizontal_lines;
}

std::pair<OsqpExitCode, std::vector<double>> solve_1(
    std::array<float, N_DIM> start_pos_joints, 
    std::array<float, N_DIM> end_pos_joints, 
    float time_step, 
    int waypoints_count, 
    constraint_t<N_DIM> velocity_constraints, 
    constraint_t<N_DIM> acceleration_constraints,
    constraint_t<N_DIM> position_constraints,
    constraint_t<3> constraints_3d,
    std::vector<horizontal_line_t> obstacles) {

        constraints::Constraint<N_DIM> velocity_constraints_transformed = 
            createConstraint<N_DIM>(velocity_constraints);

        constraints::Constraint<N_DIM> acceleration_constraints_transformed = 
            createConstraint<N_DIM>(acceleration_constraints);

        constraints::Constraint<N_DIM> position_constraints_transformed = 
            createConstraint<N_DIM>(position_constraints);
        
        constraints::Constraint<3> constraints_3d_transformed = 
            createConstraint<3>(constraints_3d);

        std::vector<HorizontalLine> obstacles_transformed = 
            createHorizontalLines(obstacles);

        std::vector<RobotBall> mappers{ 
            RobotBall(&forward_kinematics_6_back, &joint_jacobian_6_back, 0.15, false),
            RobotBall(&forward_kinematics, &joint_jacobian, 0.05, true),
        };

        GOMPSolver<N_DIM> gomp_obj(waypoints_count,
            time_step,
            position_constraints_transformed,
            velocity_constraints_transformed,
            acceleration_constraints_transformed,
            constraints_3d_transformed,
            obstacles_transformed,
            mappers,
            &inverse_kinematics);

        Ctrl<N_DIM> start_pos_joints_transformed;
        Ctrl<N_DIM> end_pos_joints_transformed;

        for (int i = 0; i < N_DIM; i++)
        {
            start_pos_joints_transformed(i) = static_cast<double>(start_pos_joints[i]);
            end_pos_joints_transformed(i) = static_cast<double>(end_pos_joints[i]);
        }

        auto [e, b1] = gomp_obj.run(start_pos_joints_transformed, end_pos_joints_transformed);

        std::vector<double> return_vector(b1.data(), b1.data() + b1.size());

        return std::make_pair(e, return_vector);
    }

void define_enum(py::module &m) {
  py::enum_<OsqpExitCode>(m, "OsqpExitCode")
      .value("kOptimal", OsqpExitCode::kOptimal)
      .value("kPrimalInfeasible", OsqpExitCode::kPrimalInfeasible)
      .value("kDualInfeasible", OsqpExitCode::kDualInfeasible)
      .value("kOptimalInaccurate", OsqpExitCode::kOptimalInaccurate)
      .value("kPrimalInfeasibleInaccurate", OsqpExitCode::kPrimalInfeasibleInaccurate)
      .value("kDualInfeasibleInaccurate", OsqpExitCode::kDualInfeasibleInaccurate)
      .value("kMaxIterations", OsqpExitCode::kMaxIterations)
      .value("kInterrupted", OsqpExitCode::kInterrupted)
      .value("kTimeLimitReached", OsqpExitCode::kTimeLimitReached)
      .value("kNonConvex", OsqpExitCode::kNonConvex)
      .value("kUnknown", OsqpExitCode::kUnknown)
      .export_values();
}

PYBIND11_MODULE(gomp, m) {
    m.def("solve_1", &solve_1, "Runs the GOMP solver.");

    define_enum(m);
}

// Compile using (now compiled with CMakeLists.txt):
// g++ -O3 -Wall -shared -std=gnu++11 `python3-config --cflags --ldflags --libs` ../src/bindings/gomp-solver-binding.cpp -o gomp.so -fPIC -I/home/olaf/anaconda3/lib/python3.9/site-packages/pybind11/include

// Shared object library will be created. Make sure to rename the result file to: gomp.so, and move to the directory with test files.

// Got this error:
// /usr/bin/ld: ../_deps/abseil-cpp-build/absl/strings/libabsl_cordz_functions.a(cordz_functions.cc.o): relocation R_X86_64_TPOFF32 against symbol `_ZN4absl13cord_internal17cordz_next_sampleE' can not be used when making a shared object; recompile with -fPIC
// /usr/bin/ld: failed to set dynamic section sizes: bad value
// collect2: error: ld returned 1 exit status
//
// Did:
// cd _deps/abseil-cpp-build/absl/strings/
// make clean
// make CXXFLAGS=-fPIC
// Doesn't work
//
// Adding this to cmakelists works: set(CMAKE_POSITION_INDEPENDENT_CODE ON)
