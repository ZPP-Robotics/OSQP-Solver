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
               Constraint<N_DIM> pos_con,
               Constraint<N_DIM> vel_con,
               Constraint<N_DIM> acc_con,
               const QPMatrixSparse& P,
               std::vector<HorizontalLine> z_obstacles_geq,
               std::map<size_t, std::pair<ForwardKinematics, Jacobian>> m,
               InverseKinematics ik)
            : max_waypoints(waypoints),
              time_step(time_step),
              pos_con(pos_con),
              vel_con(vel_con),
              acc_con(acc_con),
              problem_matrix(P),
              z_obstacles_geq(std::move(z_obstacles_geq)),
              mappers(std::move(m)),
              ik(std::move(ik)) {
        assert(max_waypoints >= 2);
    }

    std::pair<ExitCode, QPVector> run(Ctrl<N_DIM> start_pos, Ctrl<N_DIM> end_pos) {
        auto warm_start = calcWarmStart(start_pos, end_pos);
        auto constraint_builder = initConstraints(start_pos, end_pos, warm_start);

        auto qp_solver = QPSolver{constraint_builder.build(), problem_matrix};
        qp_solver.setWarmStart(warm_start);

        QPVector last_solution;
        auto last_code = ExitCode::kUnknown;

        for (auto i = 0; i < 30; i++) {
            auto [exit_code, solution] = qp_solver.solve();
            if (exit_code != ExitCode::kOptimal) {
                // There are no solutions.
                last_solution = solution;
                last_code = exit_code;
                break;
            }

            last_code = ExitCode::kOptimal;
            last_solution = solution;
            qp_solver.update(
                    constraint_builder
                            .zObstacles(z_obstacles_geq, solution)
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
    const QPMatrixSparse problem_matrix;
    const std::vector<HorizontalLine> z_obstacles_geq;
    const std::map<size_t, std::pair<ForwardKinematics, Jacobian>> mappers;
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
                .zObstacles(z_obstacles_geq, warm_start);
    }

};

#endif //GOMP_SOLVER_H
