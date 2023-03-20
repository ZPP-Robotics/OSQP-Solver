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
    const QPVector3d bypassOffset;

    static QPVector3d fromXY(const QPVector2d &v) {
        return {v[0], v[1], 0};
    }

public:

    HorizontalLine(const QPVector2d &direction, const QPVector3d &point, QPVector3d bypassOffset)
            : D(fromXY(direction) / direction.norm()),
              A(point),
              bypassOffset(bypassOffset) {}

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
    QPVector3d getDistanceVec(const QPVector3d &P) const {
        QPVector3d X = A + (P - A).dot(D) * D;
        return X - P;
    }

    QPVector2d getDistanceVecXY(const QPVector3d &P) const {
        QPVector3d dist = getDistanceVec(P);
        return QPVector2d{dist[0], dist[1]};
    }

    /**
     * Return "horizontal" distance from P to line.
     */
    double getDistanceXY(const QPVector3d &P) const {
        return getDistanceVecXY(P).norm();
    }

    /**
     * Return point on the line closest to P.
     */
    QPVector3d operator[](const QPVector3d &P) const {
        return P + getDistanceVec(P);
    };

    QPVector3d getBypassOffset() const {
        return bypassOffset;
    }

    bool areOnOppositeSides(const QPVector3d &P, const QPVector3d &Q) const {
        QPVector2d distP = getDistanceVecXY(P);
        QPVector2d distQ = getDistanceVecXY(Q);
        bool res = distP.dot(distQ) < 0;
        if (res) {
            printf("points (%f, %f, %f) and (%f, %f, %f) are on opposite sides of line (dir=(%f, %f))\n", P[0], P[1], P[2], Q[0], Q[1], Q[2], D[0], D[1]);
        }
                                    
        return res;
    }

    bool isClose(const QPVector3d &P, double radius) const {
        return getDistanceXY(P) < radius;
    }


};

#endif //GOMP_HORIZONTAL_LINE_H
