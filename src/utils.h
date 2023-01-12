#ifndef UTILS_H
#define UTILS_H

#include <limits>
#include <array>
#include <osqp++.h>

using ExitCode = osqp::OsqpExitCode;
using QPMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor, long long>;
using QPVector = Eigen::VectorXd;

/**
 * Creates nxn sparse matrix M so that
 * i < offset then M[i, j] = 0
 * i >= offset then M[i, i] = a, M[i, i - diagonal_num] = b, M[i, i + diagonal_num] = b.
 * @return M
 */
QPMatrix triDiagonalMatrix(double a, double b, int n, int offset = 0, int diagonal_num = 1) {
    QPMatrix m(n, n);

    std::vector<Eigen::Triplet<double>> nonZeroValues;
    for (int i = offset; i < n; ++i) {
        nonZeroValues.emplace_back(i, i, a);
        if (i + 1 + diagonal_num < n) {
            nonZeroValues.emplace_back(i, i + 1 + diagonal_num, b);
        }
        if (i - 1 - diagonal_num < n) {
            nonZeroValues.emplace_back(i, i - 1 - diagonal_num, b);
        }
    }
    m.setFromTriplets(nonZeroValues.begin(), nonZeroValues.end());

    return m;
}



#endif //UTILS_H
