#include <iostream>

#include <tuple>
#include "constraints/constraint-builder.h"
#include "utils.h"
#include "osqp-wrapper.h"
#include "gomp-solver.h"
#include <chrono>

constexpr const size_t WAYPOINTS = 1000;
constexpr const int PROPERTIES = 2;
constexpr const size_t DIMS = 6;
constexpr const int VARS = WAYPOINTS * PROPERTIES * DIMS;
constexpr const double TIME_STEP = 0.1;

using namespace Eigen;
using namespace osqp;
using namespace std;

// maximum and minimum values of joint angles
// in fact it is -2pi and 2pi but -pi and pi is enough to get everywhere
constexpr double Q_MIN = - M_PI;
constexpr double Q_MAX =   M_PI;

int main() {

    GOMPSolver<DIMS> s{WAYPOINTS, TIME_STEP,
                       constraints::inRange<DIMS>(of<DIMS>(Q_MIN), of<DIMS>(Q_MAX)),
                       constraints::inRange<DIMS>(of<DIMS>(-0.01), of<DIMS>(0.01)),
                       constraints::inRange<DIMS>(of<DIMS>(-0.1), of<DIMS>(0.1)),
                       triDiagonalMatrix(2, -1, VARS, WAYPOINTS * DIMS, DIMS),
                    //    {}};
                       {{500, constraints::zObstacleGeq<DIMS>(0.5)}}};


    for (int i = 0; i < 1; ++i) {
        auto start = std::chrono::high_resolution_clock::now();

        Point start_pos_gt(-0.13, 0.82, 0.063);
        Point   end_pos_gt(0.5, 0.5, 0);
        auto [e, b1] = s.run(start_pos_gt.toCtrl(), end_pos_gt.toCtrl());

        auto finish = std::chrono::high_resolution_clock::now();
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start);
        std::cout << "Time iter: " << milliseconds.count() << "ms\n";

        std::cout << "pos dim1: \n";
        for (int j = 0; j < WAYPOINTS; ++j) {
            cout << b1[DIMS * j] << ", ";
        }
        cout << endl;
        cout << endl;
        std::cout << "vel dim1: \n";
        for (int j = 0; j < WAYPOINTS; ++j) {
            cout << b1[DIMS * j + WAYPOINTS * DIMS] << ", ";
        }
        cout << endl;
        cout << endl;
        std::cout << "acc dim1: \n";
        for (int j = 0; j + 1 < WAYPOINTS; ++j) {
            cout << b1[DIMS * (j + 1) + WAYPOINTS * DIMS] - b1[DIMS * j + WAYPOINTS * DIMS] << ", ";
        }
        cout << endl;
        cout << endl;
        std::cout << "pos dim2: \n";
        for (int j = 0; j < WAYPOINTS; ++j) {
            cout << b1[DIMS * j + 1] << ", ";
        }
        cout << endl;
        cout << endl;
        std::cout << "vel dim2: \n";
        for (int j = 0; j < WAYPOINTS; ++j) {
            cout << b1[DIMS * j + WAYPOINTS * DIMS + 1] << ", ";
        }
        cout << endl;
        cout << endl;
        std::cout << "acc dim2: \n";
        for (int j = 0; j + 1 < WAYPOINTS; ++j) {
            cout << b1[DIMS * (j + 1) + WAYPOINTS * DIMS + 1] - b1[DIMS * j + WAYPOINTS * DIMS + 1] << ", ";
        }

        Point start_pos_found = Ctrl(b1[0], b1[1], b1[2], b1[3], b1[4], b1[5]).toPoint();
        auto offset = DIMS * (WAYPOINTS - 1);
        Point end_pos_found = Ctrl(b1[offset + 0], b1[offset + 1], b1[offset + 2], b1[offset + 3], b1[offset + 4], b1[offset + 5]).toPoint();
        offset = DIMS * 500;
        Point mid_pos_found = Ctrl(b1[offset + 0], b1[offset + 1], b1[offset + 2], b1[offset + 3], b1[offset + 4], b1[offset + 5]).toPoint();

        std::cout << "\n\n Ground true point: " << start_pos_gt << " point -> ctrl -> point: " << start_pos_gt.toCtrl().toPoint() << "= decoded point \n\n";
        std::cout << "\n\n";
        std::cout << "Ground true starting position: " << start_pos_gt << " starting position after optimization: " << start_pos_found << "\n";
        std::cout << "Middle position after optimization: "   << mid_pos_found << "\n";
        std::cout << "Ground true end position: "      << end_pos_gt   << " end position after optimization: "      << end_pos_found   << "\n\n";

        std::cout << ToString(e) << endl;
        std::cout << b1.size() << endl;
    }

}
