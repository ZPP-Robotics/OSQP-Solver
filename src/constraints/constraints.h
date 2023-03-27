#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <utility>
#include <optional>
#include <limits>
#include <Eigen/Core>

namespace constraints {

    const double INF = 1e30;

    // N-dimensional bound (upper or lower).
    template<size_t N>
    using Bound = std::optional<Eigen::Vector<double, N>>;

    // N-dimensional lower and upper bounds.
    template<size_t N>
    using Constraint = std::pair<Bound<N>, Bound<N>>;

    // Returns array of length N filled with value `val`.
    template<size_t N>
    Eigen::Vector<double, N> of(double val) {
        Eigen::Vector<double, N> res;
        res.setConstant(val);
        return res;
    }

    template<size_t N>
    Constraint<N> inRange(Bound<N> low, Bound<N> upp) {
        return {low, upp};
    }

    template<size_t N>
    Constraint<N> equal(const Eigen::Vector<double, N> &vals) {
        return inRange<N>({vals}, {vals});
    }

   template<size_t N>
   Constraint<N> greaterEq(const Eigen::Vector<double, N> &vals) {
       return inRange<N>(vals, {});
   }

   template<size_t N>
   Constraint<N> lessEq(const Eigen::Vector<double, N> &vals) {
       return inRange<N>({}, vals);
   }

    template<size_t N>
    const Constraint<N> ANY = {{}, {}};

    template<size_t N>
    const Constraint<N> EQ_ZERO = equal<N>(of<N>(0));

    template<size_t N>
    const Bound<N> scaled(const Bound<N> &b, double v) {
        if (b.has_value()) {
            return (*b) * v;
        }
        return {};
    }

    template<size_t N>
    const Constraint<N> scaled(const Constraint<N> &c, double v) {
        auto &[l, u] = c;
        return {scaled<N>(l, v), scaled<N>(u, v)};
    }

}


#endif //CONSTRAINTS_H
