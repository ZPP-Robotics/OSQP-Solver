#include <iostream>

#include <osqp++.h>
#include <tuple>
#include "constraint_builder.h"
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

    auto P = tridiagonalMatrix(2, -1, VARS, WAYPOINTS);

    auto constraints = ConstraintBuilder<DIMS>{WAYPOINTS, TIME_STEP};


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
