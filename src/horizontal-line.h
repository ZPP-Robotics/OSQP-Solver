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
    const Point A;
    const QPVector3d bypassOffset;

    static QPVector3d fromXY(const QPVector2d &v) {
        return {v[0], v[1], 0};
    }

public:

    HorizontalLine(const QPVector2d &direction, const Point &point, QPVector3d bypassOffset)
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
    QPVector3d getDistanceVec(const Point &P) const {
        QPVector3d X = A + (P - A).dot(D) * D;
        return X - P;
    }

    QPVector2d getDistanceVecXY(const Point &P) const {
        QPVector3d dist = getDistanceVec(P);
        return QPVector2d{dist[0], dist[1]};
    }

    /**
     * Return "horizontal" distance from P to line.
     */
    double getDistanceXY(const Point &P) const {
        return getDistanceVecXY(P).norm();
    }

    /**
     * Return point on the line closest to P.
     */
    Point operator[](const Point &P) const {
        return P + getDistanceVec(P);
    };

    QPVector3d getBypassOffset() const {
        return bypassOffset;
    }

    bool areOnOppositeSides(const Point &P, const Point &Q) const {
        QPVector2d distP = getDistanceVecXY(P);
        QPVector2d distQ = getDistanceVecXY(Q);
        return distP.dot(distQ) < 0;
    }

    bool isClose(const Point &P) const {
        return getDistanceXY(P) < bypassOffset.norm();
    }

    bool hasCollision(int waypoint, const QPVector &trajectory_xyz) const {
        int waypoints = trajectory_xyz.size() / 3;        
        Point p = trajectory_xyz.segment(3 * waypoint, 3);

        if (isClose(p)) return true;
        if (waypoint > 0) {
            Point p_prev = trajectory_xyz.segment(3 * (waypoint - 1), 3);
            if (areOnOppositeSides(p_prev, p)) return true;
        }
        if (waypoint + 1 < waypoints) {
            Point p_next = trajectory_xyz.segment(3 * (waypoint + 1), 3);
            if (areOnOppositeSides(p, p_next)) return true;
        }
        return false;
    }

    bool isAbove(const Point &P) const {
        return bypassOffset[Axis::Z] > 0 
            ? (P - A - bypassOffset)[Axis::Z] >= -CENTIMETER
            : (P - A - bypassOffset)[Axis::Z] <= +CENTIMETER;
    }

};

#endif //GOMP_HORIZONTAL_LINE_H
