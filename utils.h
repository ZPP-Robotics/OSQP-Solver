#ifndef UTILS_H
#define UTILS_H

#include <limits>
#include <array>
#include <osqp++.h>

Eigen::SparseMatrix<double, Eigen::ColMajor>
tridiagonalMatrix(double a, double b, int n, int offset = 0) {
    Eigen::SparseMatrix<double> m(n, n);

    std::vector<Eigen::Triplet<double>> nonZeroValues;
    for (int i = offset; i < n; ++i) {
        nonZeroValues.emplace_back(i, i, 2);
        if (i + 1 < n) {
            nonZeroValues.emplace_back(i, i + 1, -1);
        }
        if (i - 1 >= offset) {
            nonZeroValues.emplace_back(i, i - 1, -1);
        }
    }
    m.setFromTriplets(nonZeroValues.begin(), nonZeroValues.end());

    return m;
}



#endif //UTILS_H
