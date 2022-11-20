//
// Created by mat on 20/11/2022.
//

#ifndef TEST3_CONSTRAINT_BUILDER_H
#define TEST3_CONSTRAINT_BUILDER_H

#include "utils.h"

template<size_t N_DIM>
class ConstraintBuilder {

    using constraint_t = std::array<double, N_DIM>;
    using factor_t = std::pair<size_t, double>;
    using matrix_cell_t = Eigen::Triplet<double, size_t>;
    static constexpr const size_t DYNAMICS_DERIVATIVES = 2;

public:

    ConstraintBuilder(size_t waypoints, double timestep)
        : waypoints(waypoints), timestep(timestep) {
        linkVelocityToPosition();
    }

    ConstraintBuilder &posGreaterEq(size_t i, constraint_t v) {
        addPositionConstraint(i, v);
        return *this;
    }

    ConstraintBuilder &posLessEq(size_t i, constraint_t v) {
        addPositionConstraint(i, NEG_INF<N_DIM>, v);
        return *this;
    }

    ConstraintBuilder &posEq(size_t i, constraint_t v) {
        addPositionConstraint(i, v, v);
        return *this;
    }

    ConstraintBuilder &velocityLessEq(size_t i, constraint_t v) {
        addVelocityConstraint(i, NEG_INF<N_DIM>, v);
        return *this;
    }

    ConstraintBuilder &velocityGreaterEq(size_t i, constraint_t v) {
        addVelocityConstraint(i, v);
        return *this;
    }

    ConstraintBuilder &velocityEq(size_t i, constraint_t v) {
        addVelocityConstraint(i, v, v);
        return *this;
    }

    std::tuple<
            Eigen::VectorXd,
            Eigen::SparseMatrix<double, Eigen::ColMajor>,
            Eigen::VectorXd
    > build() {
        Eigen::SparseMatrix<double, Eigen::ColMajor> A;
        A.resize(lowerBounds.size(), N_DIM * waypoints * DYNAMICS_DERIVATIVES);
        A.template setFromTriplets(linearSystem.begin(), linearSystem.end());

        return {
                Eigen::Map<Eigen::VectorXd>(lowerBounds.data(), lowerBounds.size()),
                A,
                Eigen::Map<Eigen::VectorXd>(upperBounds.data(), upperBounds.size())
        };
    }

private:
    
    const size_t waypoints;
    const double timestep;

    std::vector<matrix_cell_t> linearSystem{};
    std::vector<double> lowerBounds;
    std::vector<double> upperBounds;

    size_t linkVelocityToPosition() {
        for (size_t i = 0; i + 1 < waypoints; ++i) {
            size_t baseV = nthVelocity(i);
            size_t baseP = nthPos(i);
            for (size_t j = 0; j < N_DIM; ++j) {
                addConstraint({
                        factor_t{baseV + j, timestep},
                        factor_t{baseP + j + 1, -1},
                        factor_t{baseP + j, 1}
                }, 0, 0);
            }
        }
        return waypoints * N_DIM;
    }

    [[nodiscard]] size_t nthVelocity(size_t i) const {
        return waypoints * N_DIM + nthPos(i);
    }

    [[nodiscard]] size_t nthPos(size_t i) const {
        return i * N_DIM;
    }

    void addConstraint(std::vector<factor_t> &&equation,
                       double l = -INF,
                       double u = INF) {
        size_t constraints = lowerBounds.size();
        for (const auto &[variable, value]: equation) {
            linearSystem.emplace_back(constraints, variable, value);
        }
        lowerBounds.emplace_back(l);
        upperBounds.emplace_back(u);
    }
    
    void addPositionConstraint(size_t i,
                               constraint_t l = NEG_INF<N_DIM>,
                               constraint_t u = POS_INF<N_DIM>) {
        size_t base = nthPos(i);
        for (int j = 0; j < N_DIM; ++j) {
            addConstraint({factor_t{base + j, 1}}, l[j], u[j]);
        }
    }

    void addVelocityConstraint(size_t i,
                               constraint_t l = NEG_INF<N_DIM>,
                               constraint_t u = POS_INF<N_DIM>) {
        size_t base = nthVelocity(i);
        for (int j = 0; j < N_DIM; ++j) {
            addConstraint({factor_t{base + j, 1}}, l[j], u[j]);
        }
    }
    
    
};


#endif //TEST3_CONSTRAINT_BUILDER_H
