#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <utility>
#include <optional>
#include <limits>

namespace constraints {

    const double INF = std::numeric_limits<double>::infinity();

    // N-dimensional bound (upper or lower).
    template<size_t N>
    using Bound = std::optional<std::array<double, N>>;

    // N-dimensional lower and upper bounds.
    template<size_t N>
    using Constraint = std::pair<Bound<N>, Bound<N>>;

    // Returns array of length N filled with value `val`.
    template<size_t N>
    std::array<double, N> of(double val) {
        std::array<double, N> res;
        res.fill(val);
        return res;
    }

    template<size_t N>
    Constraint<N> inRange(Bound<N> low, Bound<N> upp) {
        return {low, upp};
    }

    template<size_t N>
    Constraint<N> equal(std::array<double, N> vals) {
        return inRange<N>({vals}, {vals});
    }

    template<size_t N>
    Constraint<N> greaterEq(std::array<double, N> vals) {
        return inRange<N>(vals, {});
    }

    template<size_t N>
    Constraint<N> lessEq(std::array<double, N> vals) {
        return inRange<N>({}, vals);
    }

    template<size_t N>
    Constraint<N> zObstacleGeq(double z) {
        auto inf_inf_z = inRange<N>({of<N>(-INF)}, {of<N>(INF)});
        inf_inf_z.first->at(0) = z;
        return inf_inf_z;
    }

    template<size_t N>
    const Constraint<N> ANY = inRange<N>({of<N>(-INF)}, {of<N>(INF)});

    template<size_t N>
    const Constraint<N> EQ_ZERO = equal<N>(of<N>(0));

}


#endif //CONSTRAINTS_H
