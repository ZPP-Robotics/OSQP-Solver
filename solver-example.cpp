#include <iostream>

#include <tuple>
#include "constraints/constraint-builder.h"
#include "utils.h"
#include "osqp-wrapper.h"
#include "gomp-solver.h"
#include <chrono>

constexpr const size_t WAYPOINTS = 20;
constexpr const int PROPERTIES = 2;
constexpr const size_t DIMS = 6;
constexpr const int VARS = WAYPOINTS * PROPERTIES * DIMS;
constexpr const double TIME_STEP = 1;

using namespace Eigen;
using namespace osqp;
using namespace std;



int main() {

    GOMPSolver<DIMS> s{WAYPOINTS, TIME_STEP,
                       constraints::inRange<DIMS>(of<DIMS>(-1000), of<DIMS>(1000)),
                       constraints::inRange<DIMS>(of<DIMS>(-10), of<DIMS>(10)),
                       constraints::inRange<DIMS>(of<DIMS>(-10), of<DIMS>(10)),
                       triDiagonalMatrix(2, -1, VARS, WAYPOINTS * DIMS, DIMS)};


    for (int i = 0; i < 1; ++i) {
        auto start = std::chrono::high_resolution_clock::now();

        auto [e, b1] = s.run({100, 100}, {0, 100});

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

        std::cout << ToString(e) << endl;
        std::cout << b1.size() << endl;
    }

}
