#ifndef GOMP_SOLVER_H
#define GOMP_SOLVER_H

#include <utility>

#include "constraints/constraint-builder.h"
#include "osqp-wrapper.h"

using InverseKinematics = std::function<int(double *, double, double, double)>;

template<size_t N_DIM>
class GOMPSolver {

public:

    GOMPSolver(size_t waypoints, double time_step,
                const Constraint<N_DIM> &pos_con,
                const Constraint<N_DIM> &vel_con,
                const Constraint<N_DIM> &acc_con,
                const Constraint<3> &con_3d,
                const std::vector<HorizontalLine> &obstacles,
                const std::vector<std::pair<ForwardKinematicsFun, JacobianFun>> &m,
                const InverseKinematics &ik)
            : max_waypoints(waypoints),
              time_step(time_step),
              pos_con(pos_con),
              vel_con(vel_con),
              acc_con(acc_con),
              con_3d(con_3d),
              problem_matrix(triDiagonalMatrix(2, -1, max_waypoints * N_DIM * 2, max_waypoints * N_DIM, N_DIM)),
              obstacles(obstacles),
              mappers(m),
              ik(ik) {
        assert(max_waypoints >= 2);
    }

    std::pair<ExitCode, QPVector> run(Ctrl<N_DIM> start_pos, Ctrl<N_DIM> end_pos) {
        auto warm_start = calcWarmStart(start_pos, end_pos);
        auto constraint_builder = initConstraints(start_pos, end_pos, warm_start);

        auto qp_solver = QPSolver{constraint_builder.build(), problem_matrix};
        qp_solver.setWarmStart(warm_start);

        QPVector last_solution = warm_start;
        auto last_code = ExitCode::kUnknown;
        while (true) {
            auto [exit_code, solution] = qp_solver.solve();
            if (exit_code != ExitCode::kOptimal) {
                // There are no solutions.
                break;
            }

            last_solution = solution;
            if (isSolutionOK(last_solution)) {
                last_code = ExitCode::kOptimal;

                break;
            }
            
            qp_solver.update(
                    constraint_builder
                            .withObstacles(con_3d, obstacles, solution)
                            .build()
            );
        }
        return {last_code, last_solution};
    }

private:

    const double time_step;
    const size_t max_waypoints;
    const Constraint<N_DIM> pos_con;
    const Constraint<N_DIM> vel_con;
    const Constraint<N_DIM> acc_con;
    const Constraint<3> con_3d;
    const QPMatrixSparse problem_matrix;
    const std::vector<HorizontalLine> obstacles;
    const std::vector<std::pair<ForwardKinematicsFun, JacobianFun>> mappers;
    const InverseKinematics ik;

QPVector calcWarmStart(const Ctrl<N_DIM> &start_pos, const Ctrl<N_DIM> &end_pos) {
        // Warm start as described in paper.
        QPVector positions = linspace<N_DIM>(start_pos, end_pos, max_waypoints);

        QPVector velocities;
        velocities.setZero(max_waypoints * N_DIM);

        QPVector warm_start(2 * max_waypoints * N_DIM);
        warm_start << positions, velocities;

        return warm_start;
    }

    ConstraintBuilder<N_DIM> initConstraints(const Ctrl<N_DIM> &start_pos,
                                             const Ctrl<N_DIM> &end_pos,
                                             const QPVector &warm_start) {
        // Constrain all positions, velocities, accelerations with
        // pos_con, vel_con, acc_con respectively.
        // Set start and end positions.
        // Set start and end velocities/accelerations to zero.
        return ConstraintBuilder<N_DIM>{max_waypoints, time_step, mappers}
                .positions(0, max_waypoints - 1, pos_con)
                .velocities(0, max_waypoints - 1, vel_con)
                .accelerations(0, max_waypoints - 2, acc_con)
                .position(0, constraints::equal<N_DIM>(start_pos))
                .velocity(0, EQ_ZERO<N_DIM>)
                .acceleration(0, EQ_ZERO<N_DIM>)
                .position(max_waypoints - 1, constraints::equal<N_DIM>(end_pos))
                .velocity(max_waypoints - 1, EQ_ZERO<N_DIM>)
                .acceleration(max_waypoints - 2, EQ_ZERO<N_DIM>)
                .withObstacles(con_3d, obstacles, warm_start);
    }

    bool isSolutionOK(const QPVector &q_trajectory) const {
        auto [low, upp] = con_3d;

        for (const auto &[fk_fun, _] : mappers) {
            QPVector trajectory_xyz = mapJointTrajectoryToXYZ<N_DIM>(q_trajectory, fk_fun);
                int waypoints = trajectory_xyz.size() / 3;
                for (int waypoint = 0; waypoint < waypoints; ++waypoint) {
                    Point p = trajectory_xyz.segment(waypoint * 3, 3);
                    printf("(%f, %f, %f)\n", p[Axis::X], p[Axis::Y], p[Axis::Z]);
                    for (auto axis : XYZ_AXES) {
                        double axis_low = -INF;
                        double axis_upp = INF;
                        if (low.has_value()) {
                            axis_low = (*low)[axis];
                        }
                        if (upp.has_value()) {
                            axis_upp = (*upp)[axis];
                        }
                        if (!(axis_low - CENTIMETER <= p[axis] && p[axis] <= axis_upp + CENTIMETER)) return false;
                    }

                    for (const auto &obstacle : obstacles) {
                        if (obstacle.hasCollision(waypoint, trajectory_xyz, 5 * CENTIMETER)
                            && !obstacle.isAbove(p)) {
                            return false;
                        }
                    }
                }
        }
        return true;
    }

};

#endif //GOMP_SOLVER_H
