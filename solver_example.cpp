#include <iostream>

#include <osqp++.h>
#include <vector>
#include <tuple>
#include "constraint_builder.h"
#include "qp_solver.h"

constexpr const size_t WAYPOINTS = 10;
constexpr const int PROPERTIES = 2;
constexpr const int VARS = WAYPOINTS * PROPERTIES;
constexpr const double TIME_STEP = 1;

using namespace Eigen;
using namespace osqp;
using namespace std;



int main() {
    constexpr const size_t DIMS = 2;


    auto [l, A, u] = ConstraintBuilder<DIMS>{WAYPOINTS, TIME_STEP}
            .velocityEq(0, fill<DIMS>(0))
            .velocityEq(WAYPOINTS - 1, fill<DIMS>(0))
            .posGreaterEq(WAYPOINTS / 3, fill<DIMS>(100))
            .posLessEq(2 * WAYPOINTS / 3, fill<DIMS>(-200))
            .posEq(0, fill<DIMS>(0))
            .posEq(WAYPOINTS - 1, fill<DIMS>(0))
            .build();

    QPSolver<DIMS> s{l, A, u};

    auto [a, b] = s.solve();
cout << l << endl;
cout << "dipa" << endl;
cout << u << endl;
cout << A << endl;
      for (int i = 0; i < WAYPOINTS; ++i) {
          cout << b[i] << ", ";
      }
      cout << endl;
      for (int i = WAYPOINTS; i < VARS; ++i) {
          cout << b[i] << ", ";
      }

      cout << endl;


}
