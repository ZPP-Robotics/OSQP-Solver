//
// Created by mat on 20/11/2022.
//

#ifndef TEST3_CONSTRAINT_BUILDER_H
#define TEST3_CONSTRAINT_BUILDER_H

#include "utils.h"


template<size_t N_DIM>
class ConstraintBuilder {

    using constraint_matrix = Eigen::SparseMatrix<double, Eigen::ColMajor, long long>;
    using bound_vector = Eigen::VectorXd;
    using constraint_t = std::array<double, N_DIM>;
    using factor_t = std::pair<size_t, double>;
    using matrix_cell_t = Eigen::Triplet<double, size_t>;

    static constexpr const size_t DYNAMICS_DERIVATIVES = 2;

public:

    ConstraintBuilder(size_t waypoints, double timestep)
            : waypoints(waypoints), timestep(timestep) {
        linkVelocityToPosition();
        userConstraintOffset = lowerBounds.size();

        lowerBounds.resize(userConstraintOffset + N_DIM * waypoints * DYNAMICS_DERIVATIVES, -INF);
        upperBounds.resize(userConstraintOffset + N_DIM * waypoints * DYNAMICS_DERIVATIVES, INF);
    }

    ConstraintBuilder &posGreaterEq(size_t i, constraint_t v) {
        return positionInRange(i, v);
    }

    ConstraintBuilder &posLessEq(size_t i, constraint_t v) {
        return positionInRange(i, NEG_INF<N_DIM>, v);
    }

    ConstraintBuilder &posEq(size_t i, constraint_t v) {
        return positionInRange(i, v, v);
    }

    ConstraintBuilder &velocityLessEq(size_t i, constraint_t v) {
        return velocityInRange(i, NEG_INF<N_DIM>, v);
    }

    ConstraintBuilder &velocityGreaterEq(size_t i, constraint_t v) {
        return velocityInRange(i, v);
    }

    ConstraintBuilder &velocityEq(size_t i, constraint_t v) {
        return velocityInRange(i, v, v);
    }

    ConstraintBuilder &positionInRange(size_t i,
                                       constraint_t l = NEG_INF<N_DIM>,
                                       constraint_t u = POS_INF<N_DIM>) {
        return variableInRange(nthPos(i), l, u);
    }

    ConstraintBuilder &velocityInRange(size_t i,
                                       constraint_t l = NEG_INF<N_DIM>,
                                       constraint_t u = POS_INF<N_DIM>) {
        return variableInRange(nthVelocity(i), l, u);
    }

    std::tuple<
            bound_vector,
            constraint_matrix,
            bound_vector
    >
    build() {
        constraint_matrix A;
        A.resize(lowerBounds.size(), N_DIM * waypoints * DYNAMICS_DERIVATIVES);
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

    void linkVelocityToPosition() {
        for (size_t i = 0; i + 1 < waypoints; ++i) {
            size_t baseV = nthVelocity(i);
            size_t baseP = nthPos(i);
            for (size_t j = 0; j < N_DIM; ++j) {
                lowerBounds.push_back(-INF);
                upperBounds.push_back(INF);
                addConstraint(lowerBounds.size() - 1,
                              {
                                      factor_t{baseV + j, timestep},
                                      factor_t{baseP + j + 1, -1},
                                      factor_t{baseP + j, 1}
                              }, 0, 0);
            }
        }
    }

    [[nodiscard]] size_t nthVelocity(size_t i) const {
        return waypoints * N_DIM + nthPos(i);
    }

    [[nodiscard]] size_t nthPos(size_t i) const {
        return i * N_DIM;
    }

    void addConstraint(size_t constraint_idx,
                       std::vector<factor_t> &&equation,
                       double l = -INF,
                       double u = INF) {
        for (const auto &[variable_idx, coeff]: equation) {
            linearSystem.emplace_back(constraint_idx, variable_idx, coeff);
        }
        lowerBounds[constraint_idx] = l;
        upperBounds[constraint_idx] = u;
    }

    ConstraintBuilder &variableInRange(size_t n_dim_var_start,
                                       constraint_t l = NEG_INF<N_DIM>,
                                       constraint_t u = POS_INF<N_DIM>) {
        for (int j = 0; j < N_DIM; ++j) {
            addConstraint(userConstraintOffset + n_dim_var_start + j,
                          {factor_t{n_dim_var_start + j, 1}},
                          l[j], u[j]);
        }
        return *this;
    }

};


#endif //TEST3_CONSTRAINT_BUILDER_H
