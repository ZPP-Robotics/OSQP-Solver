#include <iostream>

#include <tuple>
#include "constraints/constraint_builder.h"
#include "utils.h"
#include "qp_solver.h"

constexpr const size_t WAYPOINTS = 100;
constexpr const int PROPERTIES = 2;
constexpr const int VARS = WAYPOINTS * PROPERTIES;
constexpr const double TIME_STEP = 1;

using namespace Eigen;
using namespace osqp;
using namespace std;

int main() {

    constexpr const size_t DIMS = 1;

    auto P = triDiagonalMatrix(2, -1, VARS, WAYPOINTS);

    auto constraints = ConstraintBuilder<DIMS>{WAYPOINTS, TIME_STEP}
            .velocity(0, constraints::equal<DIMS>(0))
            .velocity(WAYPOINTS - 1, constraints::equal<DIMS>(0))
            .position(WAYPOINTS / 3, constraints::greaterEq<DIMS>(100))
            .position(2 * WAYPOINTS / 3, constraints::lessEq<DIMS>(-200))
            .position(0, constraints::equal<DIMS>(0))
            .position(WAYPOINTS - 1, constraints::equal<DIMS>(0));

    auto [l, A, u] = constraints.build();

    QPSolver s{l, A, u, P};

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
