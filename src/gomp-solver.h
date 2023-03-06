#ifndef GOMP_SOLVER_H
#define GOMP_SOLVER_H

#include <utility>

#include "constraints/constraint-builder.h"
#include "osqp-wrapper.h"

template<size_t N_DIM>
class GOMPSolver {

public:

    GOMPSolver(size_t waypoints, double time_step,
               Constraint<N_DIM> pos_con,
               Constraint<N_DIM> vel_con,
               Constraint<N_DIM> acc_con,
               QPMatrixSparse &&P,
               std::vector<HorizontalLine> z_obstacles_geq)
            : max_waypoints(waypoints),
              time_step(time_step),
              pos_con(pos_con),
              vel_con(vel_con),
              acc_con(acc_con),
              problem_matrix(P),
              z_obstacles_geq(std::move(z_obstacles_geq)) {
        assert(max_waypoints >= 2);
    }

    std::pair<ExitCode, QPVector> run(Ctrl start_pos, Ctrl end_pos) {
        auto [constraints, warm_start] = initConstraintsAndWarmStart(start_pos, end_pos);

        auto qp_solver = QPSolver{constraints.build(), problem_matrix};
        qp_solver.setWarmStart(warm_start);
        auto last_code = ExitCode::kUnknown;
        QPVector last_solution{};

        for(auto i = 0; i < 30; i++) {
            auto [exit_code, solution] = qp_solver.solve();
            if (exit_code != ExitCode::kOptimal) {
                // // There are no solutions.
                last_solution = solution;
                last_code = exit_code;
                break;
            }

            last_code = ExitCode::kOptimal;
            last_solution = solution;
            qp_solver.update(
                    constraints
                            .zObstacles(z_obstacles_geq, solution)
                            .build()
            );
        }

        // for (size_t t = max_waypoints; t >= 2; --t) {
        //     auto [exit_code, solution] = qp_solver.solve();
        //     if (exit_code != ExitCode::kOptimal) {
        //         if (t == max_waypoints) {
        //             // There are no solutions.
        //             last_solution = solution;
        //             last_code = exit_code;
        //         }
        //         break;
        //     }

        //     last_code = ExitCode::kOptimal;
        //     last_solution = solution;
        //     qp_solver.update(
        //             constraints
        //                     .position(t - 2, end_pos.toConstraintEq())
        //                     .velocity(t - 2, EQ_ZERO<N_DIM>)
        //                     .acceleration(t - 2, EQ_ZERO<N_DIM>)
        //                     .zObstacles(z_obstacles_geq, solution)
        //                     .build()
        //     );
        // }

        return {last_code, last_solution};
    }

    // std::pair<ExitCode, QPVector> run_binsearch(Point start_pos, Point end_pos) {
    //     auto qp_solver = QPSolver{initConstraints(start_pos, end_pos).build(), problem_matrix};
    //     auto [code, sol] = qp_solver.solve();
    //     if (code != ExitCode::kOptimal) {
    //         return {code, sol};
    //     }

    //     size_t l = 2;
    //     size_t p = max_waypoints - 1;
    //     while (l < p) {
    //         size_t m = (l + p) / 2;
    //         auto constraint_matrix = initConstraints(start_pos, end_pos)
    //                 .positions(m - 1, max_waypoints - 1, end_pos.toConstraintEq())
    //                 .velocities(m - 1, max_waypoints - 1, EQ_ZERO<N_DIM>)
    //                 .accelerations(m - 1, max_waypoints - 2, EQ_ZERO<N_DIM>)
    //                 .build();

    //         qp_solver.update(constraint_matrix);
    //         auto [exit_code, solution] = qp_solver.solve();

    //         if (exit_code == ExitCode::kOptimal) {
    //             p = m;
    //             sol = solution;
    //             code = ExitCode::kOptimal;
    //         } else {
    //             l = m + 1;
    //         }
    //     };
    //     return {code, sol};

    // }

private:

    const double time_step;
    const size_t max_waypoints;
    const Constraint<N_DIM> pos_con;
    const Constraint<N_DIM> vel_con;
    const Constraint<N_DIM> acc_con;
    const QPMatrixSparse problem_matrix;
    const std::vector<HorizontalLine> z_obstacles_geq;

    std::pair<ConstraintBuilder<N_DIM>, QPVector> initConstraintsAndWarmStart(const Ctrl& start_pos, const Ctrl& end_pos) {
        // Warm start as described in paper
        // Propertes = 2
        QPVector warm_start = linspace<N_DIM, 2>(start_pos, end_pos, max_waypoints);

        // Constrain all positions, velocities, accelerations with
        // pos_con, vel_con, acc_con respectively.
        // Set start and end positions.
        // Set start and end velocities/accelerations to zero.
        return {ConstraintBuilder<N_DIM>{max_waypoints, time_step}
                .positions(0, max_waypoints - 1, pos_con)
                .velocities(0, max_waypoints - 1, vel_con)
                .accelerations(0, max_waypoints - 2, acc_con)
                .position(0, start_pos.toConstraintEq())
                .velocity(0, EQ_ZERO<N_DIM>)
                .acceleration(0, EQ_ZERO<N_DIM>)
                .position(max_waypoints - 1, end_pos.toConstraintEq())
                .velocity(max_waypoints - 1, EQ_ZERO<N_DIM>)
                .acceleration(max_waypoints - 2, EQ_ZERO<N_DIM>)
                .zObstacles(z_obstacles_geq, warm_start),
                warm_start};
    }

};

#endif //GOMP_SOLVER_H
