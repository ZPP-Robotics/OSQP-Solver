#include <iostream>

#include <tuple>
#include "constraints/constraint-builder.h"
#include "utils.h"
#include "osqp-wrapper.h"
#include "gomp-solver.h"

constexpr const size_t WAYPOINTS = 200;
constexpr const int PROPERTIES = 2;
constexpr const int VARS = WAYPOINTS * PROPERTIES;
constexpr const double TIME_STEP = 1;

using namespace Eigen;
using namespace osqp;
using namespace std;

int main() {

    constexpr const size_t DIMS = 1;

    GOMPSolver<DIMS> s{WAYPOINTS, TIME_STEP,
                       constraints::inRange<DIMS>(of<DIMS>(-150), of<DIMS>(150)),
                       constraints::inRange<DIMS>(of<DIMS>(-100), of<DIMS>(100)),
                       constraints::inRange<DIMS>(of<DIMS>(-10), of<DIMS>(10)),
                       triDiagonalMatrix(2, -1, VARS, WAYPOINTS)};

    auto [e, b1] = s.run({100}, {0});

    if (e == ExitCode::kOptimal) {
        cout << "POSITIONS: \n";
        for (int i = 0; i < WAYPOINTS; ++i) {
            cout << b1[i] << ", ";
        }
        cout << endl;
        cout << "VELOCITIES: \n";
        for (int i = WAYPOINTS; i < VARS; ++i) {
            cout << b1[i] << ", ";
        }
    } else {
        cout << ToString(e) << endl;
        exit(1);
    }



}
