#ifndef GOMP_SOLVER_H
#define GOMP_SOLVER_H

#include <utility>

#include "constraints/constraint-builder.h"
#include "osqp-wrapper.h"

using InverseKinematics = std::function<int(double *, double, double, double)>;

template<size_t N_DIM>
class GOMPSolver {

public:

    GOMPSolver(size_t waypoints,
                const Constraint<N_DIM> &pos_con,
                const Constraint<N_DIM> &vel_con,
                const Constraint<N_DIM> &acc_con,
                const Constraint<3> &con_3d,
                const std::vector<HorizontalLine> &obstacles,
                const std::vector<std::pair<ForwardKinematicsFun, JacobianFun>> &m,
                const InverseKinematics &ik)
            : max_waypoints(waypoints),
              pos_con(pos_con),
              vel_con(vel_con),
              acc_con(acc_con),
              con_3d(con_3d),
              obstacles(obstacles),
              mappers(m),
              ik(ik) {
        assert(max_waypoints >= 2);
    }

    std::pair<ExitCode, QPVector> run(Ctrl<N_DIM> start_pos, Ctrl<N_DIM> end_pos) {
        QPVector last_solution = calcWarmStart(start_pos, end_pos);
        ExitCode last_code = ExitCode::kUnknown;
        // for (int i = 10; i >= 10; --i) {
        //     int waypoints = max_waypoints;
        //     QPVector warm_start(waypoints * N_DIM * 2);
        //     warm_start << last_solution.segment(0, waypoints * N_DIM), last_solution.segment(waypoints * N_DIM, waypoints * N_DIM);
        //     auto [exit_code, solution] = run(start_pos, end_pos, waypoints, warm_start);
        //     if (exit_code != ExitCode::kOptimal) {
        //         break;
        //     }
        //     last_code = ExitCode::kOptimal;
        //     last_solution = solution;
        // }
        // return {last_code, last_solution};
        return run(start_pos, end_pos, max_waypoints, last_solution);
    }

    std::pair<ExitCode, QPVector> run(Ctrl<N_DIM> start_pos, Ctrl<N_DIM> end_pos,
                                        size_t waypoints, const QPVector &warm_start) {
        auto constraint_builder = initConstraints(start_pos, end_pos, warm_start, waypoints);

        auto qp_solver = QPSolver{
            constraint_builder.build(),
            triDiagonalMatrix(2, -1, N_DIM * 2 * waypoints, waypoints * N_DIM, N_DIM)
        };
        qp_solver.setWarmStart(warm_start);

        QPVector last_solution = warm_start;
        auto last_code = ExitCode::kUnknown;
        int i = 0;
        while (true) {
            auto [exit_code, solution] = qp_solver.solve();
            if (exit_code != ExitCode::kOptimal) {
                // There are no solutions.
                break;
            }
            if (isSolutionOK(solution)) {
                last_solution = solution;
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

    const size_t max_waypoints;
    const Constraint<N_DIM> pos_con;
    const Constraint<N_DIM> vel_con;
    const Constraint<N_DIM> acc_con;
    const Constraint<3> con_3d;
    const std::vector<HorizontalLine> obstacles;
    const std::vector<std::pair<ForwardKinematicsFun, JacobianFun>> mappers;
    const InverseKinematics ik;

QPVector calcWarmStart(const Ctrl<N_DIM> &start_pos, const Ctrl<N_DIM> &end_pos) {
        // Warm start as described in paper.
        QPVector positions = linspace<N_DIM>(start_pos, end_pos, max_waypoints);

        QPVector velocities;
        velocities.setZero(max_waypoints * N_DIM);

        QPVector warm_start(2 * max_waypoints* N_DIM);
        warm_start << positions, velocities;

        return warm_start;
    }

    ConstraintBuilder<N_DIM> initConstraints(const Ctrl<N_DIM> &start_pos,
                                             const Ctrl<N_DIM> &end_pos,
                                             const QPVector &warm_start,
                                             size_t waypoints) {
        // Constrain all positions, velocities, accelerations with
        // pos_con, vel_con, acc_con respectively.
        // Set start and end positions.
        // Set end velocity to zero.
        assert (waypoints >= 4);
        Ctrl<N_DIM> q = end_pos;
        printf("(%f, %f)\n", q[0], q[1]);

        return ConstraintBuilder<N_DIM>{waypoints, mappers}
                .position(0, constraints::equal<N_DIM>(start_pos))
                .positions(1, waypoints - 2, pos_con)
                .position(waypoints - 3, constraints::equal<N_DIM>(end_pos))
                .velocities(0, waypoints - 4, vel_con)
                .velocity(waypoints - 3, EQ_ZERO<N_DIM>)
                .accelerations(0, waypoints - 4, acc_con)
                .acceleration(waypoints - 3, EQ_ZERO<N_DIM>)
                .withObstacles(con_3d, obstacles, warm_start);
    }

    bool isSolutionOK(const QPVector &q_trajectory) const {
        auto [low, upp] = con_3d;
        bool res = true;
        int waypoints = q_trajectory.size() / N_DIM / 2;
        for (int waypoint = 0; waypoint < waypoints; ++waypoint) {
            Ctrl<N_DIM> q = q_trajectory.segment(N_DIM * waypoint, N_DIM);
            printf("q: (");
            for (int i = 0; i < N_DIM; ++i) {
                printf("%f, ", q[i]);
            }
            printf("), v(");
            Ctrl<N_DIM> v = q_trajectory.segment(waypoints * N_DIM + N_DIM * waypoint, N_DIM);
            for (int i = 0; i < N_DIM; ++i) {
                printf("%f, ", v[i]);
            }
            printf(")\n");
        
        }

        for (const auto &[fk_fun, _] : mappers) {
            printf("solution for end effector: \n");
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
                        if (!(axis_low - 2 * CENTIMETER <= p[axis] && p[axis] <= axis_upp + 2 * CENTIMETER)) res = false;
                    }

                    for (const auto &obstacle : obstacles) {
                        if (obstacle.hasCollision(waypoint, trajectory_xyz, 5 * CENTIMETER)
                            && !obstacle.isAbove(p)) {
                            res = false;
                        }
                    }
                }
        }
        return res;
    }

};

#endif //GOMP_SOLVER_H
