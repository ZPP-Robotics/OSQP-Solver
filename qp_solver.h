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

class QPSolver {

public:

    QPSolver(
            Eigen::VectorXd low,
            Eigen::SparseMatrix<double, Eigen::ColMajor, c_int> A,
            Eigen::VectorXd upp,
            Eigen::SparseMatrix<double, Eigen::ColMajor, c_int> P
    ) {
        OsqpInstance instance;

        instance.constraint_matrix = A;
        instance.objective_matrix = P;

        instance.objective_vector.resize(A.cols());

        instance.lower_bounds = low;
        instance.upper_bounds = upp;

        cout << A << endl;
        cout << instance.objective_matrix << endl;

        OsqpSettings settings;
        auto status = solver.Init(instance, settings);
    }

    void update(
            Eigen::VectorXd low,
            Eigen::SparseMatrix<double, Eigen::ColMajor, c_int> A,
            Eigen::VectorXd upp
    ) {
        solver.UpdateConstraintMatrix(A);
        solver.SetBounds(low, upp);
    }

    pair<OsqpExitCode, Eigen::VectorXd> solve() {
        OsqpExitCode exit_code = solver.Solve();
        return {exit_code, solver.primal_solution()};
    }

private:

    OsqpSolver solver;

};

#endif //TEST3_QP_SOLVER_H
