#ifndef CONSTRAINT_BUILDER_H
#define CONSTRAINT_BUILDER_H

#include <cassert>

#include "utils.h"

template<size_t N_DIM>
class ConstraintBuilder {

    using constraint_matrix = Eigen::SparseMatrix<double, Eigen::ColMajor, long long>;
    using bound_vector = Eigen::VectorXd;
    using constraint_t = std::array<std::optional<double>, N_DIM>;
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

        positionsInRange(0, waypoints - 1, NEG_INF<N_DIM>, POS_INF<N_DIM>);
        velocitiesInRange(0, waypoints - 1, NEG_INF<N_DIM>, POS_INF<N_DIM>);
        accelerationsInRange(0, waypoints - 2, NEG_INF<N_DIM>, POS_INF<N_DIM>);
    }

    ConstraintBuilder &positionInRange(size_t i,
                                       constraint_t l,
                                       constraint_t u) {
        return positionsInRange(i, i, l, u);
    }

    ConstraintBuilder &positionsInRange(size_t first, size_t last,
                                        constraint_t l,
                                        constraint_t u) {
        return variablesInRange(nthPos(first), nthPos(last), l, u);
    }

    ConstraintBuilder &velocityInRange(size_t i,
                                       constraint_t l,
                                       constraint_t u) {
        return velocitiesInRange(i, i, l, u);
    }

    ConstraintBuilder &velocitiesInRange(size_t first, size_t last,
                                         constraint_t l,
                                         constraint_t u) {
        return variablesInRange(nthVelocity(first), nthVelocity(last), l, u);
    }

    ConstraintBuilder &accelerationsInRange(size_t first, size_t last,
                                            constraint_t l,
                                            constraint_t u) {
        for (size_t i = first; i <= last; ++i) {
            accelerationInRange(i, l, u);
        }
        return *this;
    }

    ConstraintBuilder &accelerationInRange(size_t i,
                                           constraint_t l,
                                           constraint_t u) {
        assert(i < waypoints);
        size_t baseA = nthAcceleration(i);
        size_t baseV = nthVelocity(i);
        size_t baseNextV = nthVelocity(i + 1);
        for (int j = 0; j < N_DIM; j++) {
            // l <= 1/timestep * (v_{t+1} - v_{t} ) <= u
            addConstraint(userConstraintOffset + baseA + j,
                          {
                                  {baseNextV + j, 1 / timestep},
                                  {baseV + j,     -1 / timestep}
                          }, l[j], u[j]);
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
                       std::optional<double> l,
                       std::optional<double> u) {
        for (const auto &[variable_idx, coeff]: equation) {
            linearSystem.emplace_back(constraint_idx, variable_idx, coeff);
        }
        if (l) lowerBounds[constraint_idx] = l.value();
        if (u) upperBounds[constraint_idx] = u.value();
    }

    ConstraintBuilder &variableInRange(size_t n_dim_var_start,
                                       constraint_t l,
                                       constraint_t u) {
        for (size_t j = 0; j < N_DIM; ++j) {
            addConstraint(userConstraintOffset + n_dim_var_start + j,
                          {
                                  {n_dim_var_start + j, 1}
                          },
                          l[j], u[j]);
        }
        return *this;
    }

    ConstraintBuilder &variablesInRange(size_t first_start, size_t last_start,
                                        constraint_t l,
                                        constraint_t u) {
        for (size_t i = first_start; i <= last_start; i += N_DIM) {
            variableInRange(i, l, u);
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
                              }, 0, 0);
            }
        }
    }

};

#endif //CONSTRAINT_BUILDER_H
