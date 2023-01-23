#ifndef UTILS_H
#define UTILS_H

#include <limits>
#include <array>
#include <iostream>
#include <osqp++.h>

#include "constraints/constraints.h"
#include "../../Kinematics-UR5e-arm/src/analytical_ik.h"

#include <armadillo>

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
    }
    m.setFromTriplets(nonZeroValues.begin(), nonZeroValues.end());

    return m;
}

struct Point;
struct Ctrl;

struct Point {
    double x, y, z;

    Point(double x, double y, double z) : x(x), y(y), z(z) {}

    Ctrl toCtrl() const;
};

struct Ctrl {
    double q1, q2, q3, q4, q5, q6;

    Ctrl(double q1, double q2, double q3, double q4, double q5, double q6)
            : q1(q1), q2(q2), q3(q3), q4(q4), q5(q5), q6(q6) {}

    Point toPoint() const;

    constraints::Constraint<6> toConstraintEq() const {
        return constraints::equal(std::array<double, 6>{q1, q2, q3, q4, q5, q6});
    }

    QPVector to_vec() const {
        return QPVector{{q1, q2, q3, q4, q5, q6}};
    }
};

std::ostream &operator<<(std::ostream &os, const Point &p) {
    os << "Point: (" << p.x << ", " << p.y << ", " << p.z << ")";
    return os;
}

std::ostream &operator<<(std::ostream &os, const Ctrl &p) {
    os << "Ctrl: (" << p.q1 << ", " << p.q2 << ", " << p.q3 << ", " << p.q4 << ", " << p.q5 << ", " << p.q6 << ")";
    return os;
}

std::string to_string(Point const &arg) {
    std::ostringstream ss;
    ss << arg;
    return std::move(ss).str();  // enable efficiencies in c++17
}

class NoInverseKinematicSolution : public std::runtime_error {
public:
    NoInverseKinematicSolution(Point p)
            : std::runtime_error("No inverse kinematic solution exists for point " + to_string(p)) {}
};

// Converts site_xpos to joint angles
Ctrl Point::toCtrl() const {
    double q_sols[8 * 6]{0};
    int num_sols = inverse_kinematics(q_sols, x, y, z);

    if (num_sols == 0) {
        throw NoInverseKinematicSolution(*this);
    }

    // for(auto i = 0; i < num_sols; i++) {
    //     std::cout << "\nSolution nr: " << i << "\n";
    //     std::cout << q_sols[i*6 + 0] << ", " << q_sols[i*6 + 1] << ", " << q_sols[i*6 + 2] << ", " << q_sols[i*6 + 3] << ", " << q_sols[i*6 + 4] << ", " << q_sols[i*6 + 5] << "\n";
    //     std::cout << "As point: " << Ctrl(q_sols[i*6 + 0], q_sols[i*6 + 1], q_sols[i*6 + 2], q_sols[i*6 + 3], q_sols[i*6 + 4], q_sols[i*6 + 5]).toPoint() << "\n";
    // }

    return Ctrl(q_sols[0], q_sols[1], q_sols[2], q_sols[3], q_sols[4], q_sols[5]);
}

// Converts joint angles to site_xpos (x, y, z)
Point Ctrl::toPoint() const {
    double q[6] = {q1, q2, q3, q4, q5, q6};
    std::tuple<double, double, double> fc = forward_kinematics(q);
    return Point(std::get<0>(fc), std::get<1>(fc), std::get<2>(fc));
}

template<size_t N_DIM, size_t PROPERTIES>
QPVector linspace(const Ctrl &a, const Ctrl &b, const size_t n_steps) {
    QPVector acc;
    acc.resize(N_DIM * n_steps * PROPERTIES);
    QPVector step_size = (b.to_vec() - a.to_vec()) / (n_steps - 1);
    QPVector a_vec = a.to_vec();

    // first half is for position, initialize it to linear interpolation from a to b
    for (auto step = 0; step < n_steps; step++) {
        QPVector current = step_size * step + a_vec;
        for (auto i = 0; i < N_DIM; i++) {
            acc[step * N_DIM + i] = current[i];
        }
    }

    // second part if for velocity, initialize it to zero
    for (auto step = n_steps; step < 2 * n_steps; step++) {
        for (auto i = 0; i < N_DIM; i++) {
            acc[step * N_DIM + i] = 0.0;
        }
    }

    return acc;
}

class Line {

    /**
     * A - point on the line
     * D - direction of line (unit length)
     */
    const arma::vec3 D;
    const arma::vec3 A;

public:

    Line(const arma::vec3 &direction, const arma::vec3 &point)
            : D(direction / arma::norm(direction)), A(point) {}

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
    [[nodiscard]] arma::vec3 distanceVec(const arma::vec3 &P) const {
        arma::vec3 X = A + arma::dot(P - A, D) * D;
        return X - P;
    }

    /**
     * Return "horizontal" distance from P to line.
     */
    [[nodiscard]] double distanceXY(const arma::vec3 &P) const {
        arma::vec3 dist = distanceVec(P);
        return arma::norm(arma::vec2{dist[0], dist[1]});
    }

    /**
     * Return point on the line closest to P.
     */
    arma::vec3 operator[](const arma::vec3 &P) const {
        return P + distanceVec(P);
    };

};


#endif //UTILS_H
