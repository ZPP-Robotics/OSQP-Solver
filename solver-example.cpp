#include <iostream>

#include <tuple>
#include "src/constraints/constraint-builder.h"
#include "utils.h"
#include "osqp-wrapper.h"
#include "gomp-solver.h"
#include <chrono>
#include <fstream>

constexpr const size_t WAYPOINTS = 20;
constexpr const int PROPERTIES = 2;
constexpr const size_t DIMS = 6;
constexpr const int VARS = WAYPOINTS * PROPERTIES * DIMS;
constexpr const double TIME_STEP = 1;

using namespace Eigen;
using namespace osqp;
using namespace std;

// maximum and minimum values of joint angles
// in fact it is -2pi and 2pi but -pi and pi is enough to get everywhere
constexpr double Q_MIN = - M_PI;
constexpr double Q_MAX =   M_PI;

int main() {

    std::map<size_t, fj_pair_t> m;
    m[0] = {&forward_kinematics, &joint_jacobian};
    m[1] = {&forward_kinematics_elbow_joint, &jacobian_elbow_joint};

    GOMPSolver<DIMS> s(WAYPOINTS, TIME_STEP,
                       constraints::inRange<DIMS>(of<DIMS>(Q_MIN), of<DIMS>(Q_MAX)),
                       constraints::inRange<DIMS>(of<DIMS>(-0.3), of<DIMS>(0.3)),
                       constraints::inRange<DIMS>(of<DIMS>(-0.3), of<DIMS>(0.3)),
                       triDiagonalMatrix(2, -1, VARS, WAYPOINTS * DIMS, DIMS),
                       { HorizontalLine{{0, 1}, {0, 0, 0.3}} },
                       m);


    for (int i = 0; i < 1; ++i) {
        auto start = std::chrono::high_resolution_clock::now();

        Point start_pos_gt = Ctrl(0,0,0,0,0,0).toPoint();
        Point   end_pos_gt = Ctrl(-M_PI,0,0,0,0,0).toPoint();
        auto [e, b1] = s.run(Ctrl(0,0,0,0,0,0), Ctrl(-M_PI,0,0,0,0,0));

        auto output_file_ctrl = ofstream("output_trajectory_ctrl.data");
        auto output_file_xyz = ofstream("output_trajectory_xyz.data");
        for(auto i = 0; i < WAYPOINTS; i++) {
            output_file_ctrl << b1[DIMS * i] << " " << b1[DIMS * i + 1] << " " << b1[DIMS * i + 2] << " " << b1[DIMS * i + 3] << " " << b1[DIMS * i + 4] << " " << b1[DIMS * i + 5] << "\n";
            Point point =  Ctrl(b1[DIMS * i + 0], b1[DIMS * i + 1], b1[DIMS * i + 2], b1[DIMS * i + 3], b1[DIMS * i + 4], b1[DIMS * i + 5]).toPoint();
            output_file_xyz << point.x << " " << point.y << " " << point.z << "\n";
        }
        output_file_ctrl.close();
        output_file_xyz.close();

        Point start_pos_found = Ctrl(b1[0], b1[1], b1[2], b1[3], b1[4], b1[5]).toPoint();
        auto offset = DIMS * (WAYPOINTS - 1);
        Point end_pos_found = Ctrl(b1[offset + 0], b1[offset + 1], b1[offset + 2], b1[offset + 3], b1[offset + 4], b1[offset + 5]).toPoint();
        offset = DIMS * 10;
        Point mid_pos_found = Ctrl(b1[offset + 0], b1[offset + 1], b1[offset + 2], b1[offset + 3], b1[offset + 4], b1[offset + 5]).toPoint();

        std::cout << "\n\nSummary:\n";
        std::cout << "Ground true starting position: " << start_pos_gt << " starting position after optimization: " << start_pos_found << "\n";
        std::cout << "Middle position after optimization: "   << mid_pos_found << "\n";
        std::cout << "Ground true end position: "      << end_pos_gt   << " end position after optimization: "      << end_pos_found   << "\n\n";

        std::cout << ToString(e) << endl;
        std::cout << b1.size() << endl;
    }

}
