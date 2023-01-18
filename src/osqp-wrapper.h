#ifndef QP_SOLVER_H
#define QP_SOLVER_H

#include <iostream>

#include <osqp++.h>

using namespace std;
using namespace Eigen;
using namespace osqp;

class QPSolver {

public:

    QPSolver(const QPConstraints &c, const QPMatrix &P) {
        auto &[l, A, u] = c;
        OsqpInstance instance;

        instance.constraint_matrix = A;
        instance.objective_matrix = P;

        instance.objective_vector.resize(A.cols());

        instance.lower_bounds = l;
        instance.upper_bounds = u;

        OsqpSettings settings;
        auto status = solver.Init(instance, settings);
        assert(status.ok());
    }

    void update(const QPConstraints &qp_constraints) {
        absl::Status status;
        auto [low, A, upp] = qp_constraints;
        if (!(status = solver.UpdateConstraintMatrix(A)).ok()) {
            throw std::invalid_argument(status.ToString());
        }

        if (!(status = solver.SetBounds(low, upp)).ok()) {
            throw std::invalid_argument(status.ToString());
        }
    }

    void setWarmStart(const QPVector& primal_vector) {
        auto status = solver.SetPrimalWarmStart(primal_vector);
        std::cout << "STATUS: " << status.ToString() << std::endl;
        assert(status.ok());
    }

    pair<OsqpExitCode, Eigen::VectorXd> solve() {
        OsqpExitCode exit_code = solver.Solve();
        return {exit_code, solver.primal_solution()};
    }

private:

    OsqpSolver solver;

};

#endif //QP_SOLVER_H
