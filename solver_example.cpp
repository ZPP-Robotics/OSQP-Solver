#include <iostream>

#include <osqp++.h>
#include <vector>
#include <tuple>
#include "constraint_builder.h"

constexpr const size_t WAYPOINTS = 2000;
constexpr const int PROPERTIES = 2;
constexpr const int VARS = WAYPOINTS * PROPERTIES;
constexpr const double TIME_STEP = 1;

using namespace Eigen;
using namespace osqp;
using namespace std;
using namespace constraints;

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

int main() {
    constexpr const size_t DIMS = 6;
    SparseMatrix<double> objective_matrix(VARS * DIMS, VARS * DIMS);

    // minimize accelerations;
    objective_matrix = tridiagonalMatrix(2, -1, VARS * DIMS, WAYPOINTS * DIMS);


    auto [low, A, upp] = Builder<DIMS>{WAYPOINTS, TIME_STEP}
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
