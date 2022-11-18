#ifndef TEST3_CONSTRAINT_BUILDER_H
#define TEST3_CONSTRAINT_BUILDER_H

#include <iostream>
#include <osqp++.h>

using namespace std;
using namespace Eigen;

namespace constraints {

    constexpr const double INF = std::numeric_limits<double>::infinity();

    template<size_t N>
    static constexpr std::array<double, N> fill(double val) {
        std::array<double, N> res{};
        for (int i = 0; i < N; ++i) {
            res[i] = val;
        }
        return res;
    }

    template<size_t N_DIM>
    class Builder {

        using constraint_t = array<double, N_DIM>;
        using matrix_cell_t = Triplet<double, size_t>;
        using factor_t = pair<size_t, double>;

    public:

        static constexpr const std::array<double, N_DIM> N_DIM_INF = fill<N_DIM>(INF);
        static constexpr const std::array<double, N_DIM> N_DIM_NEG_INF = fill<N_DIM>(-INF);

        Builder(size_t waypoints, double timestep)
                : waypoints(waypoints),
                  timestep(timestep) {
        }

        Builder &posGreaterEq(size_t i, constraint_t v) {
            addPositionConstraint(i, v);
            return *this;
        }

        Builder &posLessEq(size_t i, constraint_t v) {
            addPositionConstraint(i, N_DIM_NEG_INF, v);
            return *this;
        }

        Builder &posEq(size_t i, constraint_t v) {
            addPositionConstraint(i, v, v);
            return *this;
        }

        Builder &velocityLessEq(size_t i, constraint_t v) {
            addVelocityConstraint(i, N_DIM_NEG_INF, v);
            return *this;
        }

        Builder &velocityGreaterEq(size_t i, constraint_t v) {
            addVelocityConstraint(i, v);
            return *this;
        }

        Builder &velocityEq(size_t i, constraint_t v) {
            addVelocityConstraint(i, v, v);
            return *this;
        }

        tuple<vector<double>, vector<matrix_cell_t>, vector<double>> build() {
            return {lowerBounds, linearSystem, upperBounds};
        }

    private:

        size_t waypoints;
        double timestep;
        vector<matrix_cell_t> linearSystem{};
        vector<double> lowerBounds{};
        vector<double> upperBounds{};

        [[nodiscard]] size_t nthVelocity(size_t i) const {
            return waypoints * N_DIM + nthPos(i);
        }

        [[nodiscard]] size_t nthPos(size_t i) const {
            return i * N_DIM;
        }

        template<size_t N_FACTORS>
        void addConstraint(array<factor_t, N_FACTORS> &&equation, double l = -INF, double u = INF) {
            size_t constraints = lowerBounds.size();
            for (const auto &[variable, value]: equation) {
                linearSystem.emplace_back(constraints, variable, value);
            }
            lowerBounds.emplace_back(l);
            upperBounds.emplace_back(u);
        }

        void addPositionConstraint(size_t i,
                                   constraint_t l = -N_DIM_NEG_INF,
                                   constraint_t u = N_DIM_INF) {
            size_t base = nthPos(i);
            for (int j = 0; j < N_DIM; ++j) {
                addConstraint(array<factor_t, 1>{factor_t{base + j, 1}}, l[j], u[j]);
            }
        }

        void addVelocityConstraint(size_t i,
                                   constraint_t l = -N_DIM_NEG_INF,
                                   constraint_t u = N_DIM_INF) {
            size_t base = nthVelocity(i);
            for (int j = 0; j < N_DIM; ++j) {
                addConstraint(array<factor_t, 1>{factor_t{base + j, 1}}, l[j], u[j]);
            }
        }

        size_t linkVelocityToPosition() {
            for (size_t i = 0; i + 1 < waypoints; ++i) {
                size_t baseV = nthVelocity(i);
                size_t baseP = nthPos(i);
                for (size_t j = 0; j < N_DIM; ++j) {
                    addConstraint(array<factor_t, 3>{
                            factor_t{baseV + j, timestep},
                            factor_t{baseP + j + 1, -1},
                            factor_t{baseP + j, 1}
                    }, 0, 0);
                }
            }
            return waypoints * N_DIM;
        }
    };
}




#endif //TEST3_CONSTRAINT_BUILDER_H
