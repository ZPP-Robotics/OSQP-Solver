#ifndef CONSTRAINT_BUILDER_H
#define CONSTRAINT_BUILDER_H

#include <cassert>

#include "constraints.h"

using namespace constraints;

template<size_t N_DIM>
class ConstraintBuilder {

    using constraint_matrix = Eigen::SparseMatrix<double, Eigen::ColMajor, long long>;
    using bound_vector = Eigen::VectorXd;
    using factor_t = std::pair<size_t, double>;
    using matrix_cell_t = Eigen::Triplet<double, size_t>;

    static constexpr const size_t DYNAMICS_DERIVATIVES = 3;

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

    ConstraintBuilder &position(size_t i, const constraint_t<N_DIM> &c) {
        return positions(i, i, c);
    }

    ConstraintBuilder &positions(size_t first, size_t last, const constraint_t<N_DIM> &c) {
        return variablesInRange(nthPos(first), nthPos(last), c);
    }

    ConstraintBuilder &velocity(size_t i, const constraint_t<N_DIM> &c) {
        return velocities(i, i, c);
    }

    ConstraintBuilder &velocities(size_t first, size_t last, const constraint_t<N_DIM> &c) {
        return variablesInRange(nthVelocity(first), nthVelocity(last), c);
    }

    ConstraintBuilder &accelerations(size_t first, size_t last, const constraint_t<N_DIM> &c) {
        for (size_t i = first; i <= last; ++i) {
            acceleration(i, c);
        }
        return *this;
    }

    ConstraintBuilder &acceleration(size_t i, const constraint_t<N_DIM> &c) {
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
                          }, c[j]);
        }
        return *this;
    }

    std::tuple<
            bound_vector,
            constraint_matrix,
            bound_vector
    >
    build() {
        constraint_matrix A;
        A.resize(lowerBounds.size(), N_DIM * waypoints * 2); // 2 = one for position one for velocity

        // On duplicate, overwrite cell value with the newest Triplet.
        A.setFromTriplets(linearSystem.begin(), linearSystem.end(), [](const auto &a, const auto &b) { return b; });

        return {
                Eigen::Map<bound_vector>(lowerBounds.data(), lowerBounds.size()),
                A,
                Eigen::Map<bound_vector>(upperBounds.data(), upperBounds.size())
        };
    }

private:

    const size_t waypoints;
    const double timestep;
    size_t userConstraintOffset = 0;

    std::vector<matrix_cell_t> linearSystem{};
    std::vector<double> lowerBounds;
    std::vector<double> upperBounds;

    [[nodiscard]] size_t nthVelocity(size_t i) const {
        return waypoints * N_DIM + nthPos(i);
    }

    [[nodiscard]] size_t nthPos(size_t i) const {
        return i * N_DIM;
    }

    [[nodiscard]] size_t nthAcceleration(size_t i) const {
        return waypoints * N_DIM * 2 + i * N_DIM;
    }

    void addConstraint(size_t constraint_idx,
                       std::vector<factor_t> &&equation,
                       bound_t b) {
        auto &[l, u] = b;
        for (const auto &[variable_idx, coeff]: equation) {
            linearSystem.emplace_back(constraint_idx, variable_idx, coeff);
        }
        if (l) lowerBounds[constraint_idx] = *l;
        if (u) upperBounds[constraint_idx] = *u;
        assert(lowerBounds[constraint_idx] <= upperBounds[constraint_idx]);
    }

    ConstraintBuilder &constrainVariable(size_t n_dim_var_start, const constraint_t<N_DIM> &c) {
        for (size_t j = 0; j < N_DIM; ++j) {
            addConstraint(userConstraintOffset + n_dim_var_start + j,
                          {
                                  {n_dim_var_start + j, 1}
                          },
                          c[j]);
        }
        return *this;
    }

    ConstraintBuilder &variablesInRange(size_t first_start, size_t last_start, const constraint_t<N_DIM> &c) {
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
