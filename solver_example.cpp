#include <iostream>

#include <osqp++.h>
#include <tuple>
#include "constraint_builder.h"
#include "qp_solver.h"

constexpr const size_t WAYPOINTS = 20;
constexpr const int PROPERTIES = 2;
constexpr const int VARS = WAYPOINTS * PROPERTIES;
constexpr const double TIME_STEP = 1;

using namespace Eigen;
using namespace osqp;
using namespace std;

/*
 * TEST TEST TEST
 */

int main() {
    constexpr const size_t DIMS = 1;

    auto P = tridiagonalMatrix(2, -1, VARS, WAYPOINTS);

    auto constraints = ConstraintBuilder<DIMS>{WAYPOINTS, TIME_STEP}
            .velocityEq(0, fill<DIMS>(0))
            .velocityEq(WAYPOINTS - 1, fill<DIMS>(0))
            .posGreaterEq(WAYPOINTS / 3, fill<DIMS>(100))
            .posLessEq(2 * WAYPOINTS / 3, fill<DIMS>(-200))
            .posEq(0, fill<DIMS>(0))
            .posEq(WAYPOINTS - 1, fill<DIMS>(0));

    auto [l, A, u] = constraints.build();
    QPSolver s{l, A, u, P};

    constraints.posGreaterEq(4, fill<DIMS>(50));

    auto [l1, A1, u1] = constraints.build();
    s.update(l1, A1, u1);

    auto [a1, b1] = s.solve();

    for (int i = 0; i < WAYPOINTS; ++i) {
        cout << b1[i] << ", ";
    }
    cout << endl;
    for (int i = WAYPOINTS; i < VARS; ++i) {
        cout << b1[i] << ", ";
    }



}
