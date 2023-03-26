#include <iostream>
#include "analytical_ik.h"
#include <tuple>
#include "constraints/constraint-builder.h"
#include "utils.h"
#include "osqp-wrapper.h"
#include "gomp-solver.h"
#include "horizontal-line.h"
#include <chrono>
#include <fstream>

constexpr const double TIME_STEP = 0.1;
constexpr const size_t WAYPOINTS = 180 + 2;
constexpr const int PROPERTIES = 2;
constexpr const size_t DIMS = 6;
constexpr const int VARS = WAYPOINTS * PROPERTIES * DIMS;


using namespace Eigen;
using namespace osqp;
using namespace std;

// maximum and minimum values of joint angles
// in fact it is -2pi and 2pi but -pi and pi is enough to get everywhere
constexpr double Q_MIN = - 2 * M_PI;
constexpr double Q_MAX =   2 * M_PI;

// Converts joint angles to site_xpos (x, y, z)
template<size_t N>
Point toPoint(const Ctrl<N> &c) {
    auto [x, y, z] = forward_kinematics((double *) c.data());
    return {x, y, z};
}

int main() {

    std::vector<std::pair<ForwardKinematicsFun, JacobianFun>> mappers{ 
        {&forward_kinematics_6_back, &joint_jacobian_6_back},
        {&forward_kinematics, &joint_jacobian},
    };
    
    GOMPSolver<DIMS> fff(WAYPOINTS,
                       constraints::inRange<DIMS>(of<DIMS>(Q_MIN), of<DIMS>(Q_MAX)),
                       constraints::inRange<DIMS>(of<DIMS>(-M_PI * TIME_STEP), of<DIMS>(M_PI * TIME_STEP)),
                       constraints::inRange<DIMS>(of<DIMS>(-M_PI * 800 / 180 * TIME_STEP * TIME_STEP), of<DIMS>(M_PI * 800 / 180 * TIME_STEP * TIME_STEP)),
                       constraints::inRange<3>({{-INF, -0.3, -0.1}}, {{INF, INF, 0.2}}),
                       {},
                       mappers,
                       &inverse_kinematics);

    // for (int i = 0; i < 1; ++i) {
    //     auto start = std::chrono::high_resolution_clock::now();
        
        Point start_pos_gt = toPoint<6>({0,0,0,0,0,0});
        Point   end_pos_gt = toPoint<6>({M_PI,0,0,0,0,0});
    //     // printf("FIRST: \n");
    //     // auto [et, b12] = s.run({0,0,0,0,0,0}, {M_PI,0,0,0,0,0});
    //     // printf("SECOND: \n");
    //     // auto [ett, b1t] = f.run({0,0,0,0,0,0}, {M_PI,0,0,0,0,0}, WAYPOINTS, b12);
    //     // printf("THIRD: \n");
    //     // auto [e, b1] = ff.run({0,0,0,0,0,0}, {M_PI,0,0,0,0,0}, WAYPOINTS, b1t);
    //     // printf("4th: \n");
    //     // auto [e, b1] = fff.run({0,0,0,0,0,0}, {M_PI,0,0,0,0,0}, WAYPOINTS, b1tt);

    //     printf("THIRD: \n");
        auto [e, b1] = fff.run({0,0, 0, 0, 0, 0}, {-M_PI,0, 0, 0, 0, 0});
std::cout << ToString(e) << endl;

        auto output_file_ctrl = ofstream("output_trajectory_ctrl.data");
        auto output_file_xyz = ofstream("output_trajectory_xyz.data");
        for(auto i = 0; i < b1.size() / DIMS / 2; i++) {
            output_file_ctrl << b1[DIMS * i] << " " << b1[DIMS * i + 1] << " " << b1[DIMS * i + 2] << " " << b1[DIMS * i + 3] << " " << b1[DIMS * i + 4] << " " << b1[DIMS * i + 5] << "\n";
            Point point = toPoint<6>({b1[DIMS * i + 0], b1[DIMS * i + 1], b1[DIMS * i + 2], b1[DIMS * i + 3], b1[DIMS * i + 4], b1[DIMS * i + 5]});
            output_file_xyz << "(" << point[0] << ", " << point[1] << ", " << point[2] << ")" << "\n";
        }
        output_file_ctrl.close();
        output_file_xyz.close();

        Point start_pos_found = toPoint<6>({b1[0], b1[1], b1[2], b1[3], b1[4], b1[5]});
        auto offset = DIMS * (b1.size() / DIMS / 2 - 1);
        Point end_pos_found = toPoint<6>({b1[offset + 0], b1[offset + 1], b1[offset + 2], b1[offset + 3], b1[offset + 4], b1[offset + 5]});
        offset = DIMS * 10;
        Point mid_pos_found = toPoint<6>({b1[offset + 0], b1[offset + 1], b1[offset + 2], b1[offset + 3], b1[offset + 4], b1[offset + 5]});

        std::cout << "\n\nSummary:\n";
        std::cout << "Ground true starting position: " << start_pos_gt << " starting position after optimization: " << start_pos_found << "\n";
        std::cout << "Middle position after optimization: "   << mid_pos_found << "\n";
        std::cout << "Ground true end position: "      << end_pos_gt   << " end position after optimization: "      << end_pos_found   << "\n\n";

        std::cout << ToString(e) << endl;
        std::cout << b1.size() << endl;
    
    double q[6] = {M_PI_2, M_PI_4, M_PI_2, -M_PI_4, 0, M_PI};
    auto [x, y, z] = forward_kinematics(q);
    printf("(%f, %f, %f)\n", x, y, z);

}
