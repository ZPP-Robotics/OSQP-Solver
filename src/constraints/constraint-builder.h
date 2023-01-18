#ifndef CONSTRAINT_BUILDER_H
#define CONSTRAINT_BUILDER_H

#include <cassert>

#include <armadillo>

#include "utils.h"
#include "constraints.h"
#include "osqp++.h"

using namespace constraints;

// <lower_bounds, constraint_matrix, upper_bounds>
using QPConstraints = std::tuple<QPVector, QPMatrix, QPVector>;

template<size_t N_DIM>
class ConstraintBuilder {

    using Factor = std::pair<size_t, double>;
    using matrix_cell = Eigen::Triplet<double, size_t>;

    // position + velocity + acceleration + linearized constraints for obstacles avoidance
    static constexpr const size_t DYNAMICS_DERIVATIVES = 4;

public:

    ConstraintBuilder(size_t waypoints, double timestep)
            : waypoints(waypoints), timestep(timestep) {
        linkVelocityToPosition();
        userConstraintOffset = lowerBounds.size();

        lowerBounds.resize(userConstraintOffset + N_DIM * waypoints * DYNAMICS_DERIVATIVES, -INF);
        upperBounds.resize(userConstraintOffset + N_DIM * waypoints * DYNAMICS_DERIVATIVES, INF);

        // Add default constraints -INF < var < INF.

        positions(0, waypoints - 1, ANY<N_DIM>);
        velocities(0, waypoints - 1, ANY<N_DIM>);
        accelerations(0, waypoints - 2, ANY<N_DIM>);
    }

    ConstraintBuilder &position(size_t i, const Constraint<N_DIM> &c) {
        return positions(i, i, c);
    }

    ConstraintBuilder &positions(size_t first, size_t last, const Constraint<N_DIM> &c) {
        return variablesInRange(nthPos(first), nthPos(last), c);
    }

    ConstraintBuilder &velocity(size_t i, const Constraint<N_DIM> &c) {
        return velocities(i, i, c);
    }

    ConstraintBuilder &velocities(size_t first, size_t last, const Constraint<N_DIM> &c) {
        return variablesInRange(nthVelocity(first), nthVelocity(last), c);
    }

    ConstraintBuilder &accelerations(size_t first, size_t last, const Constraint<N_DIM> &c) {
        for (size_t i = first; i <= last; ++i) {
            acceleration(i, c);
        }
        return *this;
    }

    ConstraintBuilder &acceleration(size_t i, const Constraint<N_DIM> &c) {
        assert(i < waypoints - 1);
        size_t baseA = nthAcceleration(i);
        size_t baseV = nthVelocity(i);
        size_t baseNextV = nthVelocity(i + 1);

        for (int j = 0; j < N_DIM; j++) {
            // l <= 1/timestep * (v_{t+1} - v_{t} ) <= u
            addConstraint(userConstraintOffset + baseA + j,
                          {
                                  {baseNextV + j, 1 / timestep},
                                  {baseV + j,     -1 / timestep}
                          },
                          getConstraintForNthDim(j, c));
        }
        return *this;
    }

    ConstraintBuilder &zObstacles(const std::vector<std::pair<size_t, Constraint<N_DIM>>> z_obstacles_geq, const QPVector& trajectory) {
        for(auto& [i, c] : z_obstacles_geq) {
            zObstacle(i, c, trajectory);
        }
        return *this;
    }

    ConstraintBuilder &zObstacle(size_t i, Constraint<N_DIM> c, const QPVector& trajectory) {
        // For now assume that c has only lowe bound and upper bound = INF
        assert(i < waypoints - 1);
        auto traj_offset = N_DIM * i;
        auto basePos = nthPos(i);
        auto baseZObstacle = nthZObstacle(i);

        double jacobian[3 * 6];
        double q[6]{trajectory[traj_offset], trajectory[traj_offset + 1], trajectory[traj_offset + 2],
                    trajectory[traj_offset + 3], trajectory[traj_offset + 4], trajectory[traj_offset + 5]}; 
        joint_jacobian(jacobian, q);
        arma::mat jacobian_arma = arma::mat{jacobian, 3, 6};
        arma::vec jacobian_xyz = jacobian_arma * arma::vec(q, 6);

        auto forward_xyz = forward_kinematics(q);

        c.first->at(0) = c.first->at(0) - std::get<2>(forward_xyz) + jacobian_xyz[2];
        for(int j = 0; j < N_DIM; j++) {
            addConstraint(userConstraintOffset + baseZObstacle,
                          {
                                  {basePos + j, jacobian[2 * 6 + j]}
                          },
                          getConstraintForNthDim(0, c));
        }
        return *this;
    }

    QPConstraints build() {
        QPMatrix A;
        A.resize(lowerBounds.size(), N_DIM * waypoints * 2); // 2 = one for position one for velocity

        // On duplicate, overwrite cell value with the newest Triplet.
        A.setFromTriplets(linearSystem.begin(), linearSystem.end(), [](const auto &a, const auto &b) { return b; });

        return {
                Eigen::Map<QPVector>(lowerBounds.data(), lowerBounds.size()),
                A,
                Eigen::Map<QPVector>(upperBounds.data(), upperBounds.size())
        };
    }

private:

    const size_t waypoints;
    const double timestep;
    size_t userConstraintOffset = 0;

    std::vector<matrix_cell> linearSystem{};
    std::vector<double> lowerBounds;
    std::vector<double> upperBounds;

    [[nodiscard]] size_t nthVelocity(size_t i) const {
        assert(i < waypoints);
        return waypoints * N_DIM + nthPos(i);
    }

    [[nodiscard]] size_t nthPos(size_t i) const {
        assert(i < waypoints);
        return i * N_DIM;
    }

    [[nodiscard]] size_t nthAcceleration(size_t i) const {
        assert(i < waypoints - 1);
        return waypoints * N_DIM * 2 + i * N_DIM;
    }

    [[nodiscard]] size_t nthZObstacle(size_t i) const {
        assert(i < waypoints - 1);
        return waypoints * N_DIM * 3 + i * N_DIM;
    }

    std::pair<std::optional<double>, std::optional<double>>
    getConstraintForNthDim(size_t n, const Constraint<N_DIM> &c) {
        auto &[l_opt, u_opt] = c;
        std::optional<double> l{};
        std::optional<double> u{};
        if (l_opt) l = (*l_opt)[n];
        if (u_opt) u = (*u_opt)[n];
        return {l, u};
    }

    void addConstraint(size_t constraint_idx,
                       std::vector<Factor> &&equation,
                       std::pair<std::optional<double>, std::optional<double>> b) {
        auto &[l, u] = b;
        for (const auto &[variable_idx, coeff]: equation) {
            linearSystem.emplace_back(constraint_idx, variable_idx, coeff);
        }
        if (l) lowerBounds[constraint_idx] = *l;
        if (u) upperBounds[constraint_idx] = *u;
        assert(lowerBounds[constraint_idx] <= upperBounds[constraint_idx]);
    }

    ConstraintBuilder &constrainVariable(size_t n_dim_var_start, const Constraint<N_DIM> &c) {
        for (size_t j = 0; j < N_DIM; ++j) {
            addConstraint(userConstraintOffset + n_dim_var_start + j,
                          {
                                  {n_dim_var_start + j, 1}
                          },
                          getConstraintForNthDim(j, c));
        }
        return *this;
    }

    ConstraintBuilder &variablesInRange(size_t first_start, size_t last_start, const Constraint<N_DIM> &c) {
        for (size_t i = first_start; i <= last_start; i += N_DIM) {
            constrainVariable(i, c);
        }
        return *this;
    }

    void linkVelocityToPosition() {
        for (size_t i = 0; i + 1 < waypoints; ++i) {
            size_t baseV = nthVelocity(i);
            size_t baseP = nthPos(i);
            size_t baseNextP = nthPos(i + 1);
            for (size_t j = 0; j < N_DIM; ++j) {
                lowerBounds.push_back(-INF);
                upperBounds.push_back(INF);
                addConstraint(lowerBounds.size() - 1,
                              {
                                      {baseV + j,     timestep},
                                      {baseNextP + j, -1},
                                      {baseP + j,     1}
                              }, {0, 0});
            }
        }
    }

};

#endif //CONSTRAINT_BUILDER_H
