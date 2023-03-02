#ifndef GOMP_SOLVER_H
#define GOMP_SOLVER_H

#include "constraints/constraint-builder.h"
#include "osqp-wrapper.h"
#include "ik/analytical_ik.h"

template<size_t N_DIM>
class GOMPSolver {

    using Position = std::array<double, N_DIM>;
    using Coordinates = std::tuple<double, double, double>;

public:

    GOMPSolver(size_t waypoints, double time_step,
               Constraint<N_DIM> pos_con,
               Constraint<N_DIM> vel_con,
               Constraint<N_DIM> acc_con,
               QPMatrix &&P)
            : max_waypoints(waypoints),
              time_step(time_step),
              pos_con(pos_con),
              vel_con(vel_con),
              acc_con(acc_con),
              problem_matrix(P) {
        assert(max_waypoints >= 2);
    }

    std::pair<ExitCode, QPVector> run(Position start_pos, Position end_pos) {
        auto constraints = initConstraints(start_pos, end_pos);

        auto qp_solver = QPSolver{constraints.build(), problem_matrix};

        auto last_code = ExitCode::kUnknown;
        QPVector last_solution{};

        for (size_t t = max_waypoints; t >= 2; --t) {
            auto [exit_code, solution] = qp_solver.solve();

            if (exit_code != ExitCode::kOptimal) {
                if (t == max_waypoints) {
                    // There are no solutions.
                    last_code = exit_code;
                }
                break;
            }

            last_code = exit_code;
            last_solution = std::move(solution);
            qp_solver.update(
                    constraints
                            .position(t - 2, equal<N_DIM>(end_pos))
                            .velocity(t - 2, EQ_ZERO<N_DIM>)
                            .acceleration(t - 2, EQ_ZERO<N_DIM>)
                            .build()
            );
        }

        return {last_code, last_solution};
    }

    std::pair<ExitCode, QPVector> run(Coordinates start_pos, Coordinates end_pos) {
        double q_sols_start[8 * 6];
        double q_sols_end[8 * 6];
        int num_sols_start = inverse_kinematics(q_sols_start, std::get<0>(start_pos), std::get<1>(start_pos), std::get<2>(start_pos));
        int num_sols_end = inverse_kinematics(q_sols_end, std::get<0>(end_pos), std::get<1>(end_pos), std::get<2>(end_pos));

        Position start = choose_solution(q_sols_start);
        Position end = choose_solution(q_sols_end);

        return run(start, end);
    }


private:

    const double time_step;
    const size_t max_waypoints;
    const Constraint<N_DIM> pos_con;
    const Constraint<N_DIM> vel_con;
    const Constraint<N_DIM> acc_con;
    const QPMatrix problem_matrix;

    Position choose_solution(double *q_sols) {
        std::array<double, N_DIM> = 
    }

    ConstraintBuilder<N_DIM> initConstraints(Position start_pos, Position end_pos) {
        // Constrain all positions, velocities, accelerations with
        // pos_con, vel_con, acc_con respectively.
        // Set start and end positions.
        // Set start and end velocities/accelerations to zero.
        return ConstraintBuilder<N_DIM>{max_waypoints, time_step}
                .positions(0, max_waypoints - 1, pos_con)
                .velocities(0, max_waypoints - 1, vel_con)
                .accelerations(0, max_waypoints - 2, acc_con)
                .position(0, equal<N_DIM>(start_pos))
                .velocity(0, EQ_ZERO<N_DIM>)
                .acceleration(0, EQ_ZERO<N_DIM>)
                .position(max_waypoints - 1, equal<N_DIM>(end_pos))
                .velocity(max_waypoints - 1, EQ_ZERO<N_DIM>)
                .acceleration(max_waypoints - 2, EQ_ZERO<N_DIM>);
    }

};

#endif //GOMP_SOLVER_H
