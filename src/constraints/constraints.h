#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <utility>
#include <optional>

namespace constraints {

    const double INF = std::numeric_limits<double>::infinity();

    // One-dimensional constraint.
    using bound_t = std::pair<std::optional<double>, std::optional<double>>;

    // Multidimensional constraint = array of one-dimensional bounds.
    template<size_t N>
    using constraint_t = std::array<bound_t, N>;

    template<size_t N>
    constraint_t<N> inRange(std::optional<double> low, std::optional<double> upp) {
        constraint_t<N> res;
        res.fill({low, upp});
        return res;
    }

    template<size_t N>
    const constraint_t<N> ANY = inRange<N>(-INF, INF);

    template<size_t N>
    constraint_t<N> greaterEq(double val) {
        return inRange<N>(val, {});
    }

    template<size_t N>
    constraint_t<N> lessEq(double val) {
        return inRange<N>({}, val);
    }

    template<size_t N>
    constraint_t<N> equal(double val) {
        return inRange<N>(val, val);
    }

}


#endif //CONSTRAINTS_H
