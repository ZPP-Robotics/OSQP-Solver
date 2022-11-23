#include <iostream>

#include <tuple>
#include "constraint_builder.h"
#include "qp_solver.h"

constexpr const size_t WAYPOINTS = 30;
constexpr const int PROPERTIES = 2;
constexpr const int VARS = WAYPOINTS * PROPERTIES;
constexpr const double TIME_STEP = 1;

using namespace Eigen;
using namespace osqp;
using namespace std;

int main() {
    constexpr const size_t DIMS = 1;

    auto P = tridiagonalMatrix(2, -1, VARS, WAYPOINTS);

    auto constraints = ConstraintBuilder<DIMS>{WAYPOINTS, TIME_STEP}
            .velocityEq(0, fill<DIMS>(0))
            .velocityEq(WAYPOINTS - 1, fill<DIMS>(0))
            .posEq(0, fill<DIMS>(1000))
            .posEq(WAYPOINTS - 1, fill<DIMS>(0));

    for (int i = 0; i < WAYPOINTS - 1; ++i) {
        constraints.accelerationInRange(i, fill<DIMS>(-100), fill<DIMS>(100));
    }


    Eigen::VectorXd solv;
    OsqpExitCode code = osqp::OsqpExitCode::kPrimalInfeasible;
    auto [l, A, u] = constraints.build();
    QPSolver s{l, A, u, P};
    for (int h = WAYPOINTS - 1; h > 10; --h) {
        auto [a1, b1] = s.solve();

        if (a1 != OsqpExitCode::kOptimal) {
            break;
        }
        solv = b1;
        code = a1;
        constraints.posEq(h - 1, fill<DIMS>(0));
        constraints.velocityEq(h - 1, fill<DIMS>(0));
        auto [l1, A1, u1] = constraints.build();
        s.update(l1, A1, u1);
    }

    if (code == OsqpExitCode::kOptimal) {
        for (int i = 0; i < WAYPOINTS; ++i) {
            cout << solv[i] << ", ";
        }
        cout << endl;
        for (int i = WAYPOINTS; i < VARS; ++i) {
            cout << solv[i] << ", ";
        }
    } else {
        cout << "Path does not exist" << endl;
    }

}
