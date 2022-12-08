#ifndef UTILS_H
#define UTILS_H

#include <limits>
#include <array>
#include <osqp++.h>

using ExitCode = osqp::OsqpExitCode;
using QPMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor, long long>;
using QPVector = Eigen::VectorXd;

QPMatrix triDiagonalMatrix(double a, double b, int n, int offset = 0) {
    QPMatrix m(n, n);

    std::vector<Eigen::Triplet<double>> nonZeroValues;
    for (int i = offset; i < n; ++i) {
        nonZeroValues.emplace_back(i, i, a);
        if (i + 1 < n) {
            nonZeroValues.emplace_back(i, i + 1, b);
        }
        if (i - 1 >= offset) {
            nonZeroValues.emplace_back(i, i - 1, b);
        }
    }
    m.setFromTriplets(nonZeroValues.begin(), nonZeroValues.end());

    return m;
}



#endif //UTILS_H
