//
// Created by mat on 20/11/2022.
//

#ifndef TEST3_QP_SOLVER_H
#define TEST3_QP_SOLVER_H

#include <iostream>

#include <osqp++.h>

using namespace std;
using namespace Eigen;
using namespace osqp;

template<size_t N_DIM>
class QPSolver {

    using matrix_cell_t = Triplet<double, size_t>;

public:

    QPSolver(
            Eigen::VectorXd low,
            Eigen::SparseMatrix<double, Eigen::ColMajor> A,
            Eigen::VectorXd upp) {
        OsqpInstance instance;

        instance.constraint_matrix = A;
        instance.objective_matrix = tridiagonalMatrix(2, -1, A.cols(), A.cols() / 2);

        instance.objective_vector.resize(A.cols());

        instance.lower_bounds = low;
        instance.upper_bounds = upp;

        OsqpSettings settings;
        auto status = solver.Init(instance, settings);
    }

    pair<OsqpExitCode, Eigen::VectorXd> solve() {
        OsqpExitCode exit_code = solver.Solve();
        return {exit_code, solver.primal_solution()};
    }

private:

    OsqpSolver solver;
    size_t WAYPOINTS = 2000;
    size_t VARS = WAYPOINTS * 2;
    size_t TIME_STEP = 1;

};

#endif //TEST3_QP_SOLVER_H
