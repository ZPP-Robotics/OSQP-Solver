#include <iostream>

#include <osqp++.h>
#include <tuple>
#include <fstream>
#include <iomanip>

#include "constraint_builder.h"
#include "qp_solver.h"

constexpr const size_t WAYPOINTS = 60 * 5; // 60 fps for 5 seconds
constexpr const int PROPERTIES = 2;
constexpr const int VARS = WAYPOINTS * PROPERTIES;
constexpr const double TIME_STEP = 1./60.; // basic control once per 1/60 sec

using namespace Eigen;
using namespace osqp;
using namespace std;

/*
 * TEST TEST TEST
 */

int main() {
    constexpr const size_t DIMS = 6;

    auto P = tridiagonalMatrix(2, -1, VARS * DIMS, WAYPOINTS);

    auto constraints = ConstraintBuilder<DIMS>{WAYPOINTS, TIME_STEP}
            .velocityEq(0, fill<DIMS>(0))
            .velocityEq(WAYPOINTS - 1, fill<DIMS>(0))
            .posEq(0, fill<DIMS>(0))
            .posEq(WAYPOINTS - 1, std::array<double, DIMS>{1, 1, 1, 1, 1, 1})
            .accInRange(0, WAYPOINTS - 1, std::array<double, DIMS>{-1, -1, -1, -1, -1, -1}, std::array<double, DIMS>{1, 1, 1, 1, 1, 1});

    auto [l, A, u] = constraints.build();
    QPSolver s{l, A, u, P};

    auto [a1, b1] = s.solve();

    std::ofstream fp_pos, fp_vel;
    fp_pos.open("ctrl_pos.data");
    fp_vel.open("ctrl_vel.data");
    for(int i = 0; i < WAYPOINTS; ++i) {
        auto VEL_OFFSET = WAYPOINTS * DIMS;
        for(int j = 0; j < DIMS; j++)
        {
            fp_pos << std::setw(20) << b1[i * DIMS + j];
            fp_vel << std::setw(20) << b1[i * DIMS + j + VEL_OFFSET];
        }
        fp_pos << "\n";
        fp_vel << "\n";
    }

    fp_pos.close();
    fp_vel.close();

}
