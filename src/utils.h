#ifndef UTILS_H
#define UTILS_H

#include <limits>
#include <array>
#include <iostream>
#include <osqp++.h>

#include "constraints/constraints.h"

using ExitCode = osqp::OsqpExitCode;
using QPMatrixSparse = Eigen::SparseMatrix<double, Eigen::ColMajor, long long>;
template<size_t Row, size_t Col>
using QPMatrix = Eigen::Matrix<double, Row, Col, Eigen::RowMajor>;
using QPVector = Eigen::VectorXd;
using QPVector2d = Eigen::Vector2d;
using QPVector3d = Eigen::Vector3d;
using Point = Eigen::Vector3d;
template<size_t N>
using Ctrl = Eigen::Vector<double, N>;
using ForwardKinematicsFun = std::function<std::tuple<double, double, double>(double *)>;
using JacobianFun = std::function<void(double *, double *)>;

enum Axis : size_t {
    X, Y, Z
};

constexpr const std::array<Axis, 3> XYZ_AXES = {X, Y, Z};

constexpr const double CENTIMETER = 0.01;

/**
 * Creates nxn sparse matrix M so that
 * i < offset then M[i, j] = 0
 * i >= offset then M[i, i] = a, M[i, i - diagonal_num] = b, M[i, i + diagonal_num] = b.
 * @return M
 */
QPMatrixSparse triDiagonalMatrix(double a, double b, int n, int offset = 0, int diagonal_num = 1) {
    QPMatrixSparse m(n, n);
    std::vector<Eigen::Triplet<double>> nonZeroValues;
    for (int i = offset; i < n; ++i) {
        nonZeroValues.emplace_back(i, i, a);
        if (i + diagonal_num < n) {
            nonZeroValues.emplace_back(i, i + diagonal_num, b);
        }
        if (i - diagonal_num > 0) {
            nonZeroValues.emplace_back(i, i - diagonal_num, b);
        }
    }
    m.setFromTriplets(nonZeroValues.begin(), nonZeroValues.end());
    return m;
}

class NoInverseKinematicSolution : public std::runtime_error {
public:
    NoInverseKinematicSolution(Point p)
            : std::runtime_error("No inverse kinematic solution exists for point ") {}
};

template<size_t N>
QPVector linspace(const Eigen::Vector<double, N> &a, const Eigen::Vector<double, N> &b, size_t n_steps) {
    QPVector res(N * n_steps);

    Eigen::Vector<double, N> step_size = (b - a) / (n_steps - 1);;
    for (auto step = 0; step < n_steps; step++) {
        Eigen::Vector<double, N> current = step_size * step + a;
        res.segment(step * N, N) = current;
    }
    return res;
}

template<size_t N>
QPVector mapJointTrajectoryToXYZ(const QPVector &trajectory, const ForwardKinematicsFun &mapper) {
    int waypoints = trajectory.size() / 2 / N;
    QPVector trajectory_xyz(3 * waypoints);
    for (int waypoint = 0; waypoint < waypoints; ++waypoint) {
        Ctrl<N> q = trajectory.segment(waypoint * N, N);
        auto [x, y, z] = mapper((double *) q.data());
        trajectory_xyz.segment(3 * waypoint, 3) = Point{x, y, z};
    }
    return trajectory_xyz;
}

#endif //UTILS_H
