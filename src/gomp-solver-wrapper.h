#ifndef GOMP_SOLVER_WRAPPER_H
#define GOMP_SOLVER_WRAPPER_H

#include <tuple>
#include "gomp-solver.h"

// def solve(start_pos_joints: Tuple[float x 6], end_pos_tcp: Tuple[float x 3], time_step: float, waypoints_count: int, velocity_constraints: Tuple[List[float x 6], List[float x 6]], acceleration_constraints: Tuple[List[float x 6], List[float x 6]], position_constraints: Tuple[List[float x 6], List[float x 6]], obstacles: List[Obstacle]) -> Tuple[List[float x 6], List[float x 6], List[float x 6]]

std::tuple<std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>> solve(
    std::array<float, 6> start_pos_joints, 
    std::array<float, 3> end_pos_tcp, 
    float time_step, 
    int waypoints_count, 
    std::tuple<std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>> velocity_constraints, 
    std::tuple<std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>> acceleration_constraints,
    std::tuple<std::vector<std::array<float, 6>>, std::vector<std::array<float, 6>>> position_constraints,
    std::vector<Obstacle> obstacles);

#endif // GOMP_SOLVER_WRAPPER_H