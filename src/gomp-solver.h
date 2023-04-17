#ifndef GOMP_SOLVER_H
#define GOMP_SOLVER_H

#include <utility>

#include "constraints/constraint-builder.h"
#include "osqp-wrapper.h"

using InverseKinematics = std::function<int(double *, double, double, double)>;
using InverseKinematicsMaxSolution = std::function<int(double *, double, double, double)>;
const int MAX_ITERATIONS = 100;
const int SEGMENTS = 10;

template<size_t N_DIM>
class GOMPSolver {

public:

    GOMPSolver(size_t waypoints, double time_step,
                const Constraint<N_DIM> &pos_con,
                const Constraint<N_DIM> &vel_con,
                const Constraint<N_DIM> &acc_con,
                const Constraint<3> &con_3d,
                const std::vector<HorizontalLine> &obstacles,
                const std::vector<RobotBall> &m,
                const InverseKinematics &gripper_ik)
            : max_waypoints(waypoints),
              time_step(time_step),
              pos_con(pos_con),
              vel_con(scaled<N_DIM>(vel_con, time_step)),
              acc_con(scaled<N_DIM>(acc_con, time_step * time_step)),
              con_3d(con_3d),
              obstacles(obstacles),
              mappers(m),
              gripper_ik(gripper_ik) {
        assert(max_waypoints >= 4);
    }

    std::pair<ExitCode, QPVector> run(Ctrl<N_DIM> start_pos, Ctrl<N_DIM> end_pos) {
        QPVector last_solution = calcWarmStart(start_pos, end_pos);
        ExitCode last_code = ExitCode::kUnknown;
        for (int i = SEGMENTS; i >= 1; --i) {
            int waypoints = max_waypoints * i / SEGMENTS;
            QPVector warm_start(waypoints * N_DIM * 2);
            warm_start << last_solution.segment(0, waypoints * N_DIM), last_solution.segment(waypoints * N_DIM, waypoints * N_DIM);
            auto [exit_code, solution] = run(start_pos, end_pos, waypoints, warm_start);
            if (exit_code != ExitCode::kOptimal && exit_code != ExitCode::kUnknown) {
                break;
            } else if (exit_code == ExitCode::kOptimal) {
                last_code = ExitCode::kOptimal;
                last_solution = solution;   
            }
        }
        last_solution.segment(last_solution.size() / 2, last_solution.size() / 2) /= time_step;
        return {last_code, last_solution};
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
        while (i++ < MAX_ITERATIONS) {
            auto [exit_code, solution] = qp_solver.solve();
            if (exit_code != ExitCode::kOptimal) {
                last_solution = solution;
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
                .withObstacles(con_3d, solution)
                .build()
            );
        }
        
        return {last_code, last_solution};
    }

    std::pair<ExitCode, QPVector> run(Ctrl<N_DIM> start_pos, Coordinates end_pos) {
        double q_sols_end[8 * 6];
        int num_sols_end = inverse_kinematics(q_sols_end, std::get<0>(end_pos), std::get<1>(end_pos), std::get<2>(end_pos));
        
        if (num_sols_end == 0) {
            return {ExitCode::kUnknown, QPVector()};
        }
        
        double q1 = q_sols_end[0 * 6 + 0];
        double q2 = q_sols_end[0 * 6 + 1];
        double q3 = q_sols_end[0 * 6 + 2];
        double q4 = q_sols_end[0 * 6 + 3];
        double q5 = q_sols_end[0 * 6 + 4];
        double q6 = q_sols_end[0 * 6 + 5];

        Ctrl<N_DIM> joint_end_pos = {q1, q2, q3, q4, q5, q6};
        auto [exit_code, solution] = run(start_pos, joint_end_pos);

        for (int i = 1; i < num_sols_end; ++i) {
            q1 = q_sols_end[i * 6 + 0];
            q2 = q_sols_end[i * 6 + 1];
            q3 = q_sols_end[i * 6 + 2];
            q4 = q_sols_end[i * 6 + 3];
            q5 = q_sols_end[i * 6 + 4];
            q6 = q_sols_end[i * 6 + 5];

            joint_end_pos = {q1, q2, q3, q4, q5, q6};
            auto [exit_code_sol, solution_sol] = run(start_pos, joint_end_pos);

            if (exit_code_sol == ExitCode::kOptimal) {
                continue;
            }
            else if (solution_sol.size() < solution.size()) {
                exit_code = exit_code_sol;
                solution = solution_sol;
            }

        }

        return {exit_code, solution};
    }

    std::pair<ExitCode, QPVector> run_max_ik(Ctrl<N_DIM> start_pos, {x, y, z}, InverseKinematicsMaxSolution &max_ik_solution ) {
        double q_sol_end[8 * 6];
        int num_sols_end = max_ik_solution(q_sol_end, x, y, z);
        
        if (num_sols_end == 0) {
            return {ExitCode::kUnknown, QPVector()};
        }
        
        double q1 = q_sols_end[0 * 6 + 0];
        double q2 = q_sols_end[0 * 6 + 1];
        double q3 = q_sols_end[0 * 6 + 2];
        double q4 = q_sols_end[0 * 6 + 3];
        double q5 = q_sols_end[0 * 6 + 4];
        double q6 = q_sols_end[0 * 6 + 5];

        Ctrl<N_DIM> joint_end_pos = {q1, q2, q3, q4, q5, q6};
        auto [exit_code, solution] = run(start_pos, joint_end_pos);

        return {exit_code, solution};
    }


private:

    const size_t max_waypoints;
    const double time_step;
    const Constraint<N_DIM> pos_con;
    const Constraint<N_DIM> vel_con;
    const Constraint<N_DIM> acc_con;
    const Constraint<3> con_3d;
    const std::vector<HorizontalLine> obstacles;
    const std::vector<RobotBall> mappers;
    const InverseKinematics gripper_ik;

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

        return ConstraintBuilder<N_DIM>{waypoints, mappers, obstacles}
                .position(0, constraints::equal<N_DIM>(start_pos))
                .positions(1, waypoints - 2, pos_con)
                .position(waypoints - 3, constraints::equal<N_DIM>(end_pos))
                .velocities(0, waypoints - 4, vel_con)
                .velocity(waypoints - 3, EQ_ZERO<N_DIM>)
                .accelerations(0, waypoints - 4, acc_con)
                .acceleration(waypoints - 3, EQ_ZERO<N_DIM>)
                .withObstacles(con_3d, warm_start);
    }

    bool isSolutionOK(const QPVector &q_trajectory) const {
        auto [low, upp] = con_3d;
        bool res = true;
        int waypoints = q_trajectory.size() / N_DIM / 2;
        Ctrl<N_DIM> prevV = Eigen::VectorXd::Zero(N_DIM);
        for (int waypoint = 0; waypoint < waypoints; ++waypoint) {
            Ctrl<N_DIM> q = q_trajectory.segment(N_DIM * waypoint, N_DIM);
            printf("waypoint %d\n", waypoint);
            printf("q: (");
            for (int i = 0; i < N_DIM; ++i) {
                printf("%f, ", q[i]);
            }
            printf(")\nv: (");
            Ctrl<N_DIM> v = q_trajectory.segment(waypoints * N_DIM + N_DIM * waypoint, N_DIM);
            for (int i = 0; i < N_DIM; ++i) {
                printf("%f, ", v[i] * 10);
            }
            printf(")\nacc: (");
            for (int i = 0; i < N_DIM; ++i) {
                printf("%f, ", (v[i] - prevV[i]) * 10 * 10);
            }
            prevV = v;
            printf(")\n\n");
        
        }

        for (const auto &ball : mappers) {
            printf("solution for end effector: \n");
            QPVector trajectory_xyz = mapJointTrajectoryToXYZ<N_DIM>(q_trajectory, ball.fk);
                int waypoints = trajectory_xyz.size() / 3;
                for (int waypoint = 0; waypoint < waypoints; ++waypoint) {
                    Point p = trajectory_xyz.segment(waypoint * 3, 3);
                    printf("(%f, %f, %f)\n", p[Axis::X], p[Axis::Y], p[Axis::Z]);

                    if (ball.is_gripper) {
                        for (auto axis : XYZ_AXES) {
                            double axis_low = -INF;
                            double axis_upp = INF;
                            if (low.has_value()) {
                                axis_low = (*low)[axis];
                            }
                            if (upp.has_value()) {
                                axis_upp = (*upp)[axis];
                            }
                            if (!(axis_low - ERROR <= p[axis] - ball.radius 
                                && p[axis] + ball.radius <= axis_upp + ERROR)) res = false;
                        }
                    }

                    for (const auto &obstacle : obstacles) {
                        if (obstacle.hasCollision(waypoint, trajectory_xyz, ball)
                            && !obstacle.isAbove(p, ball)) {
                            res = false;
                        }
                    }
                }
        }
        return res;
    }

};

#endif //GOMP_SOLVER_H
