#ifndef UTILS_H
#define UTILS_H

#include <limits>
#include <array>
#include <osqp++.h>

constexpr const double INF = std::numeric_limits<double>::infinity();

template<size_t N>
static constexpr std::array<std::optional<double>, N> fill(double val) {
    std::array<std::optional<double>, N> res{};
    for (int i = 0; i < N; ++i) {
        res[i] = val;
    }
    return res;
}

template<size_t N>
static constexpr const std::array<std::optional<double>, N> EMPTY{};

template<size_t N>
static constexpr const std::array<std::optional<double>, N> POS_INF = fill<N>(INF);

template<size_t N>
static constexpr const std::array<std::optional<double>, N> NEG_INF = fill<N>(-INF);

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
