#ifndef GOMP_HORIZONTAL_LINE_H
#define GOMP_HORIZONTAL_LINE_H

#include "utils.h"

class HorizontalLine {
private:

    /**
     * A - point on the lines
     * D - direction of line (horizontal & unit length)
     */
    const QPVector3d D;
    const QPVector3d A;

    static QPVector3d fromXY(const QPVector2d &v) {
        return {v[0], v[1], 0};
    }

public:

    HorizontalLine(const QPVector2d &direction, const QPVector3d &point)
            : D(fromXY(direction) / direction.norm()), A(point) {}

    /**
     * P - some point
     *
     *     P
     *    /|
     *   / |
     *  /  v
     * A---X----->D
     *
     * X - base of the perpendicular line
     *
     * (P-A) dot D == |X-A|
     *
     * X == A + ((P-A) dot D)D
     * Returns perpendicular: X-P
     */
    [[nodiscard]] QPVector3d distanceVec(const QPVector3d &P) const {
        QPVector3d X = A + (P - A).dot(D) * D;
        return X - P;
    }

    /**
     * Return "horizontal" distance from P to line.
     */
    [[nodiscard]] double distanceXY(const QPVector3d &P) const {
        QPVector3d dist = distanceVec(P);
        return QPVector2d{dist[0], dist[1]}.norm();
    }

    /**
     * Return point on the line closest to P.
     */
    QPVector3d operator[](const QPVector3d &P) const {
        return P + distanceVec(P);
    };

};

#endif //GOMP_HORIZONTAL_LINE_H
