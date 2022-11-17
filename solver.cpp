#include <iostream>
#include <osqp++.h>
#include <vector>
#include <tuple>

namespace {

    using namespace Eigen;
    using namespace osqp;
    using namespace std;

    constexpr const size_t WAYPOINTS = 2000;
    constexpr const int PROPERTIES = 2;
    constexpr const int VARS = WAYPOINTS * PROPERTIES;
    constexpr const double TIME_STEP = 1;
    constexpr const double INF = std::numeric_limits<double>::infinity();

    SparseMatrix<double>
    tridiagonalMatrix(double a, double b, int n, int offset = 0) {
        SparseMatrix<double> m(n, n);
        std::vector<Triplet<double>> nonZeroValues;
        for (int i = offset; i < n; ++i) {
            m.coeffRef(i, i) = 2;
            if (i + 1 < n) {
                m.coeffRef(i, i + 1) = -1;
            }
            if (i - 1 >= offset) {
                m.coeffRef(i, i - 1) = -1;
            }
        }
        return m;
    }

    template<size_t N>
    static constexpr std::array<double, N> fill(double val) {
        std::array<double, N> res{};
        for (int i = 0; i < N; ++i) {
            res[i] = val;
        }
        return res;
    }

    template<size_t N_DIM>
    class ConstraintBuilder {

        using constraint_t = array<double, N_DIM>;
        using matrix_cell_t = Triplet<double, size_t>;
        using factor_t = pair<size_t, double>;

    public:

        static constexpr const std::array<double, N_DIM> N_DIM_INF = fill<N_DIM>(INF);
        static constexpr const std::array<double, N_DIM> N_DIM_NEG_INF = fill<N_DIM>(-INF);

        ConstraintBuilder(size_t waypoints, double timestep)
                : waypoints(waypoints), timestep(timestep) {
            linkVelocityToPosition();
        }

        ConstraintBuilder &posGreaterEq(size_t i, constraint_t v) {
            addPositionConstraint(i, v);
            return *this;
        }

        ConstraintBuilder &posLessEq(size_t i, constraint_t v) {
            addPositionConstraint(i, N_DIM_NEG_INF, v);
            return *this;
        }

        ConstraintBuilder &posEq(size_t i, constraint_t v) {
            addPositionConstraint(i, v, v);
            return *this;
        }

        ConstraintBuilder &velocityLessEq(size_t i, constraint_t v) {
            addVelocityConstraint(i, N_DIM_NEG_INF, v);
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

        void linkVelocityToPosition() {
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
        }
    };
}

int main() {
    constexpr const size_t DIMS = 6;
    SparseMatrix<double> objective_matrix(VARS * DIMS, VARS * DIMS);

    // minimize accelerations;
    objective_matrix = tridiagonalMatrix(2, -1, VARS * DIMS, WAYPOINTS * DIMS);


    auto [low, A, upp] = ConstraintBuilder<DIMS>{WAYPOINTS, TIME_STEP}
            .velocityEq(0, fill<DIMS>(0))
            .velocityEq(WAYPOINTS - 1, fill<DIMS>(0))
            .posGreaterEq(WAYPOINTS / 3, fill<DIMS>(100))
            .posLessEq(2 * WAYPOINTS / 3, fill<DIMS>(-200))
            .posEq(0, fill<DIMS>(0))
            .posEq(WAYPOINTS - 1, fill<DIMS>(0))
            .build();

    std::vector<double> warmstart(VARS * DIMS, 0);

    OsqpInstance instance;

    instance.objective_matrix = objective_matrix;
    instance.constraint_matrix.resize(low.size(), VARS * DIMS);
    instance.constraint_matrix.setFromTriplets(A.begin(), A.end());
    instance.objective_vector = Eigen::Map<Eigen::VectorXd>(warmstart.data(), warmstart.size());
    instance.lower_bounds = Eigen::Map<Eigen::VectorXd>(low.data(), low.size());
    instance.upper_bounds = Eigen::Map<Eigen::VectorXd>(upp.data(), upp.size());

    OsqpSolver solver;
    OsqpSettings settings;
// Edit settings if appropriate.
    auto status = solver.Init(instance, settings);
// Assuming status.ok().
    OsqpExitCode exit_code = solver.Solve();
// Assuming exit_code == OsqpExitCode::kOptimal.
    double optimal_objective = solver.objective_value();
    Eigen::VectorXd optimal_solution = solver.primal_solution();

  /*  cout << instance.constraint_matrix << endl;

    for (int i = 0; i < WAYPOINTS; ++i) {
        cout << optimal_solution[3 * i] << ", ";
    }
    cout << endl;
    for (int i = WAYPOINTS; i < VARS; ++i) {
        cout << optimal_solution[i] << ", ";
    }

    cout << endl;

    for (auto v: ConstraintBuilder<5>::N_DIM_INF) {
        cout << v << endl;
    }*/

}
