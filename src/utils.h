#ifndef UTILS_H
#define UTILS_H

#include <limits>
#include <array>
#include <iostream>
#include <osqp++.h>

#include "constraints/constraints.h"
#include "../../Kinematics-UR5e-arm/src/analytical_ik.h"

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
        if (i + 1 + diagonal_num < n) {
            nonZeroValues.emplace_back(i, i + 1 + diagonal_num, b);
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

// Converts joint angles to site_xpos (x, y, z)
template<size_t N>
Point toPoint(const Ctrl<N> &c) {
    auto [x, y, z] = forward_kinematics((double *) c.data());
    return {x, y, z};
}

template<size_t N>
constraints::Constraint<N> toConstraintEq(const Ctrl<N> &c) {
    std::array<double, N> a;
    std::copy(c.data(), c.data() + N, a.begin());
    return constraints::equal(a);
}

template<size_t N_DIM, size_t PROPERTIES>
QPVector linspace(const Ctrl<N_DIM> &a, const Ctrl<N_DIM> &b, const size_t n_steps) {
    QPVector acc;
    acc.resize(N_DIM * n_steps * PROPERTIES);
    QPVector step_size = (b - a) / (n_steps - 1);;

    // first half is for position, initialize it to linear interpolation from a to b
    for (auto step = 0; step < n_steps; step++) {
        QPVector current = step_size * step + a;
        for (auto i = 0; i < N_DIM; i++) {
            acc[step * N_DIM + i] = current[i];
        }
    }

    // second part if for velocity, initialize it to zero
    for (auto step = n_steps; step < 2 * n_steps; step++) {
        for (auto i = 0; i < N_DIM; i++) {
            acc[step * N_DIM + i] = 0.0;
        }
    }

    return acc;
}

#endif //UTILS_H
