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

#endif //UTILS_H
