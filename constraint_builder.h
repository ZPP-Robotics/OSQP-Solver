//
// Created by mat on 20/11/2022.
//

#ifndef TEST3_CONSTRAINT_BUILDER_H
#define TEST3_CONSTRAINT_BUILDER_H

#include "utils.h"

#include <cassert>
#include <optional>


template<size_t N_DIM>
class ConstraintBuilder {

    using constraint_matrix = Eigen::SparseMatrix<double, Eigen::ColMajor, long long>;
    using bound_vector = Eigen::VectorXd;
    using constraint_t = std::array<double, N_DIM>;
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
        variablesInRange(nthPos(0), nthPos(waypoints - 1));
        for(auto i = 0; i < waypoints - 1; i++)
            accelerationInRange(i);
    }

    ConstraintBuilder &velocityEq(
        size_t at, 
        std::optional<constraint_t> to) {
        return velocityEq(at, at, to);
    }

    ConstraintBuilder &velocityEq(
        size_t first, 
        size_t last, 
        std::optional<constraint_t> to) {
        return velocityInRangeFromTo(first, last, to, to);
    }

    ConstraintBuilder &velocityGeq(
        size_t at, 
        std::optional<constraint_t> than) {
        return velocityGeq(at, at, than);
    }

    ConstraintBuilder &velocityGeq(
        size_t first, 
        size_t last, 
        std::optional<constraint_t> than) {
        return velocityInRangeFromTo(first, last, than);
    }

    ConstraintBuilder &velocityLeq(
        size_t at, 
        std::optional<constraint_t> than) {
        return velocityLeq(at, at, std::nullopt ,than);
    }

    ConstraintBuilder &velocityLeq(
        size_t first, 
        size_t last, 
        std::optional<constraint_t> than) {
        return velocityInRangeFromTo(first, last, std::nullopt, than);
    }

    ConstraintBuilder &posEq(
        size_t at, 
        std::optional<constraint_t> to) {
        return posEq(at, at, to);
    }

    ConstraintBuilder &posEq(
        size_t first, 
        size_t last, 
        std::optional<constraint_t> to) {
        return posInRangeFromTo(first, last, to, to);
    }

    ConstraintBuilder &posGeq(
        size_t at, 
        std::optional<constraint_t> than) {
        return posGeq(at, at, than);
    }

    ConstraintBuilder &posGeq(
        size_t first, 
        size_t last, 
        std::optional<constraint_t> than) {
        return posInRangeFromTo(first, last, than);
    }

    ConstraintBuilder &posLeq(
        size_t at, 
        std::optional<constraint_t> than) {
        return posLeq(at, at, than);
    }

    ConstraintBuilder &posLeq(
        size_t first, 
        size_t last, 
        std::optional<constraint_t> than) {
        return posInRangeFromTo(first, last, std::nullopt, than);
    }

    ConstraintBuilder &accEq(
        size_t at, 
        std::optional<constraint_t> to) {
        return accEq(at, at, to);
    }

    ConstraintBuilder &accEq(
        size_t first, 
        size_t last, 
        std::optional<constraint_t> to) {
        return accInRangeFromTo(first, last, to, to);
    }

    ConstraintBuilder &accGeq(
        size_t at, 
        std::optional<constraint_t> than) {
        return accGeq(at, at, than);
    }

    ConstraintBuilder &accGeq(
        size_t first, 
        size_t last, 
        std::optional<constraint_t> than) {
        return accInRangeFromTo(first, last, than);
    }

    ConstraintBuilder &accLeq(
        size_t at, 
        std::optional<constraint_t> than) {
        return accLeq(at, at, std::nullopt ,than);
    }

    ConstraintBuilder &accLeq(
        size_t first, 
        size_t last, 
        std::optional<constraint_t> than) {
        return accInRangeFromTo(first, last, std::nullopt, than);
    }

    ConstraintBuilder &posInRange(
        size_t at, 
        std::optional<constraint_t> lower = std::nullopt,
        std::optional<constraint_t> upper = std::nullopt) {
        return posInRange(at, at, lower, upper);
    }

    ConstraintBuilder &velocityInRange(
        size_t at,  
        std::optional<constraint_t> lower = std::nullopt,
        std::optional<constraint_t> upper = std::nullopt) {
        return velocityInRange(at, at, lower, upper);
    }
    
    ConstraintBuilder &accInRange(
        size_t at, 
        std::optional<constraint_t> lower = std::nullopt,
        std::optional<constraint_t> upper = std::nullopt) {
        return velocityInRange(at, at, lower, upper);
    }

    ConstraintBuilder &posInRange(
        size_t first, 
        size_t last, 
        std::optional<constraint_t> lower = std::nullopt,
        std::optional<constraint_t> upper = std::nullopt) {
        return posInRangeFromTo(first, last, lower, upper);
    }

    ConstraintBuilder &velocityInRange(
        size_t first, 
        size_t last, 
        std::optional<constraint_t> lower = std::nullopt,
        std::optional<constraint_t> upper = std::nullopt) {
        return velocityInRangeFromTo(first, last, lower, upper);
    }
    
    ConstraintBuilder &accInRange(
        size_t first, 
        size_t last, 
        std::optional<constraint_t> lower = std::nullopt,
        std::optional<constraint_t> upper = std::nullopt) {
        return posInRangeFromTo(first, last, lower, upper);
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
                                      factor_t{baseV + j, timestep},
                                      factor_t{baseNextP + j, -1},
                                      factor_t{baseP + j, 1}
                              }, std::optional<double>{0}, std::optional<double>{0});
            }
        }
    }

    [[nodiscard]] size_t nthVelocity(size_t i) const {
        return waypoints * N_DIM + nthPos(i);
    }

    [[nodiscard]] size_t nthPos(size_t i) const {
        return i * N_DIM;
    }

    [[nodiscard]] size_t nthAcceleration(size_t i) const {
        return waypoints * N_DIM * 2 + i * N_DIM;
    }

    void addConstraint(
        size_t constraint_idx,
        std::vector<factor_t> &&equation,
        std::optional<double> l = std::nullopt,
        std::optional<double> u = std::nullopt) {

        for (const auto &[variable_idx, coeff]: equation) {
            linearSystem.emplace_back(constraint_idx, variable_idx, coeff);
        }
        if(l.has_value())
            lowerBounds[constraint_idx] = l.value();
        if(u.has_value())
            upperBounds[constraint_idx] = u.value();
    }

    ConstraintBuilder &variablesInRange(
        size_t first, 
        size_t last, 
        std::optional<constraint_t> lower = std::nullopt,
        std::optional<constraint_t> upper = std::nullopt) {

        for (size_t i = first; i <= last; i += N_DIM) {
            variableInRange(i, lower, upper);
        }
        return *this;
    }

    ConstraintBuilder &variableInRange(
        size_t n_dim_var_start,
        std::optional<constraint_t> l = std::nullopt,
        std::optional<constraint_t> u = std::nullopt) {

        for (size_t j = 0; j < N_DIM; ++j) {
            addConstraint(userConstraintOffset + n_dim_var_start + j,
                          {factor_t{n_dim_var_start + j, 1}},
                          l.has_value() ? std::optional<double>{l.value()[j]} : std::nullopt,
                          u.has_value() ? std::optional<double>{u.value()[j]} : std::nullopt);
        }
        return *this;
    }

    ConstraintBuilder &posInRangeFromTo(
        size_t first, 
        size_t last, 
        std::optional<constraint_t> lower = std::nullopt,
        std::optional<constraint_t> upper = std::nullopt) {
        return variablesInRange(nthPos(first), nthPos(last), lower, upper);
    }

    ConstraintBuilder &velocityInRangeFromTo(
        size_t first, 
        size_t last, 
        std::optional<constraint_t> lower = std::nullopt,
        std::optional<constraint_t> upper = std::nullopt) {
        return variablesInRange(nthVelocity(first), nthVelocity(last), lower, upper);
    }

    ConstraintBuilder &accInRangeFromTo(
        size_t first, 
        size_t last, 
        std::optional<constraint_t> lower = std::nullopt,
        std::optional<constraint_t> upper = std::nullopt) {
        
        for(auto i =first; i < last; i++)
            accelerationInRange(i, lower, upper);
        return *this;
    }

    ConstraintBuilder &accelerationInRange(
        size_t i,
        std::optional<constraint_t> l = std::nullopt,
        std::optional<constraint_t> u = std::nullopt) {

        assert(i < waypoints);
        size_t baseA = nthAcceleration(i);
        size_t baseV = nthVelocity(i);
        size_t baseNextV = nthVelocity(i) + N_DIM;
        for(int j = 0; j < N_DIM; j++) {
            // l <= 1/timestep * (v_{t+1} - t_{t} ) <= u
            addConstraint(userConstraintOffset + baseA + j,
                            {
                            factor_t{baseNextV + j,  1},
                            factor_t{baseV     + j, -1}
                            }, 
                            l.has_value() ? std::optional<double>{l.value()[j] / timestep} : std::nullopt,
                            u.has_value() ? std::optional<double>{u.value()[j] / timestep} : std::nullopt);
        }
        return *this;
    }

};


#endif //TEST3_CONSTRAINT_BUILDER_H
