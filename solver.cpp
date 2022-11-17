//
// Created by mat on 16/11/2022.
//
#include <iostream>
#include <osqp++.h>
#include <vector>
#include <tuple>

namespace {
    using namespace Eigen;
    using namespace osqp;
    using namespace std;

    constexpr const int WAYPOINTS = 25;
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

    class ConstraintBuilder {
    public:
        ConstraintBuilder(size_t waypoints, double timestep)
                : waypoints(waypoints),
                  timestep(timestep) {
            linkVelocityToPosition();
        }

        ConstraintBuilder &posGreaterEq(size_t nth, double l) {
            addConstraint({{nth, 1}}, l);
            return *this;
        }

        ConstraintBuilder &posLessEq(size_t nth, double u) {
            addConstraint({{nth, 1}}, -INF, u);
            return *this;
        }

        ConstraintBuilder &posEq(size_t nth, double l = -INF, double u = INF) {
            addConstraint({{nth, 1}}, l, u);
            return *this;
        }

        ConstraintBuilder &velocityLessEq(size_t nth, double l) {
            addConstraint({{nthVelocity(nth), 1}}, l);
            return *this;
        }

        ConstraintBuilder &velocityGreaterEq(size_t nth, double u) {
            addConstraint({{nthVelocity(nth), 1}}, -INF, u);
            return *this;
        }


        ConstraintBuilder &velocityEq(size_t nth, double l = -INF, double u = INF) {
            addConstraint({{nthVelocity(nth), 1}}, l, u);
            return *this;
        }

        tuple<vector<double>, vector<Triplet<double>>, vector<double>> build() {
            return {lowerBounds, linearSystem, upperBounds};
        }

    private:
        void linkVelocityToPosition() {
            for (size_t i = 0; i + 1 < waypoints; ++i) {
                addConstraint({
                                      {nthVelocity(i), timestep},
                                      {i + 1,          -1},
                                      {i,              1}
                              },
                              0, 0);
            }
        }

        void addConstraint(vector<pair<long, double>> &&equation, double l = -INF, double u = INF) {
            size_t constraints = lowerBounds.size();
            for (const auto &[variable, value]: equation) {
                linearSystem.emplace_back(constraints, variable, value);
            }
            lowerBounds.emplace_back(l);
            upperBounds.emplace_back(u);
        }

        [[nodiscard]] size_t nthVelocity(size_t nthPos) const {
            return waypoints + nthPos;
        }


    private:
        size_t waypoints;
        double timestep;
        vector<Triplet<double>> linearSystem{};
        vector<double> lowerBounds{};
        vector<double> upperBounds{};
    };
}

int main() {
    SparseMatrix<double> objective_matrix(VARS, VARS);

    // minimize accelerations;
    objective_matrix = tridiagonalMatrix(2, -1, VARS, WAYPOINTS);

    auto [low, A, upp] = ConstraintBuilder{WAYPOINTS, TIME_STEP}
            .velocityEq(0, 0, 0)
            .velocityEq(WAYPOINTS - 1, 0, 0)
            .posGreaterEq(WAYPOINTS / 3, 100)
            .posLessEq(2 * WAYPOINTS / 3, -200)
            .posEq(0, 0, 0)
            .posEq(WAYPOINTS - 1, 0, 0)
            .build();

    std::vector<double> warmstart(VARS, 0);

    OsqpInstance instance;

    instance.objective_matrix = objective_matrix;
    instance.constraint_matrix.resize(low.size(), WAYPOINTS * 2);
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

    cout << instance.constraint_matrix << endl;

    for (int i = 0; i < WAYPOINTS; ++i) {
        cout << optimal_solution[i] << ", ";
    }
    cout << endl;
    for (int i = WAYPOINTS; i < VARS; ++i) {
        cout << optimal_solution[i] << ", ";
    }
    cout << endl;
}
