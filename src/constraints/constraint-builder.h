#ifndef CONSTRAINT_BUILDER_H
#define CONSTRAINT_BUILDER_H

#include <cassert>
#include <map>
#include <utility>

#include "utils.h"
#include "constraints.h"
#include "osqp++.h"
#include "horizontal-line.h"

using namespace constraints;

// <lower_bounds, constraint_matrix, upper_bounds>
using QPConstraints = std::tuple<QPVector, QPMatrixSparse, QPVector>;

template<size_t N_DIM>
class ConstraintBuilder {

    using Factor = std::pair<size_t, double>;
    using MatrixCell = Eigen::Triplet<double, size_t>;
    using Jacobian = QPMatrix<3, N_DIM>;

    // position + velocity + acceleration + linearized constraints for obstacles avoidance
    static constexpr const size_t DYNAMICS_DERIVATIVES = 4;

public:

    ConstraintBuilder(size_t waypoints,
                        double timestep,
                        const std::vector<std::pair<ForwardKinematicsFun, JacobianFun>> &m)
            : waypoints(waypoints), timestep(timestep), mappers(m) {
        linkVelocityToPosition();
        userConstraintOffset = lowerBounds.size();

        lowerBounds.resize(userConstraintOffset + N_DIM * waypoints * DYNAMICS_DERIVATIVES * mappers.size(), -INF);
        upperBounds.resize(userConstraintOffset + N_DIM * waypoints * DYNAMICS_DERIVATIVES * mappers.size(), INF);

        // Add default constraints -INF < var < INF.

        positions(0, waypoints - 1, ANY<N_DIM>);
        velocities(0, waypoints - 1, ANY<N_DIM>);
        accelerations(0, waypoints - 2, ANY<N_DIM>);
    }

    ConstraintBuilder &position(size_t i, const Constraint<N_DIM> &c) {
        return positions(i, i, c);
    }

    ConstraintBuilder &positions(size_t first, size_t last, const Constraint<N_DIM> &c) {
        return variablesInRange(nthPos(first), nthPos(last), c);
    }

    ConstraintBuilder &velocity(size_t i, const Constraint<N_DIM> &c) {
        return velocities(i, i, c);
    }

    ConstraintBuilder &velocities(size_t first, size_t last, const Constraint<N_DIM> &c) {
        return variablesInRange(nthVelocity(first), nthVelocity(last), c);
    }

    ConstraintBuilder &accelerations(size_t first, size_t last, const Constraint<N_DIM> &c) {
        for (size_t i = first; i <= last; ++i) {
            acceleration(i, c);
        }
        return *this;
    }

    ConstraintBuilder &acceleration(size_t i, const Constraint<N_DIM> &c) {
        assert(i < waypoints - 1);
        size_t baseA = nthAcceleration(i);
        size_t baseV = nthVelocity(i);
        size_t baseNextV = nthVelocity(i + 1);

        for (int j = 0; j < N_DIM; j++) {
            // l <= 1/timestep * (v_{t+1} - v_{t} ) <= u
            addConstraint(userConstraintOffset + baseA + j,
                          {
                                  {baseNextV + j, 1 / timestep},
                                  {baseV + j,-1 / timestep}
                          },
                          getConstraintForNthDim(j, c));
        }
        return *this;
    }

    ConstraintBuilder &withObstacles(const Constraint<3> &con_3d,
                                    const std::vector<HorizontalLine> &obstacles,
                                    const QPVector &trajectory) {
        size_t obstacle_constraints_base = userConstraintOffset + N_DIM * waypoints * 3;
        size_t next_obstacle_constraint_idx = obstacle_constraints_base;

        for (const auto &[forward_kinematics_fun, jacobian_fun]: mappers) {
            QPVector p_trajectory = mapJointTrajectoryToXYZ<N_DIM>(trajectory, forward_kinematics_fun);
            for (int waypoint = 0; waypoint < waypoints; ++waypoint) {
                Ctrl<N_DIM> q = trajectory.segment(waypoint * N_DIM, N_DIM);
                Point p = p_trajectory.segment(waypoint * 3, 3);
                Jacobian jac;
                jacobian_fun((double *) jac.data(), (double *) q.data());
                // for (int i = 0; i < N_DIM; ++i) {
                //     std::swap(jac(Axis::X, i), jac(Axis::Y, i));
                // }

                add3dPositionConstraint(next_obstacle_constraint_idx, con_3d, q, p, jac, waypoint);
                next_obstacle_constraint_idx += 3;

                for (const auto &obstacle : obstacles) {
                    // Make specified joint avoid a given obstacle.
                    if (obstacle.hasCollision(waypoint, p_trajectory, 5 * CENTIMETER)) {
                        addObstacleVerticalConstraint(next_obstacle_constraint_idx++, obstacle, q, p, jac, waypoint);
                    } else {
                        // Add dummy constraint so that at each iteration
                        // matrix of constraints has non-zero elements in exactly the same cells.
                        add3dPositionConstraint(next_obstacle_constraint_idx++, Axis::Z, jac, waypoint, -INF, INF);
                    }
                }
            }
        }
        
        return *this;
    }

    QPConstraints build() {
        QPMatrixSparse A;
        A.resize(lowerBounds.size(), N_DIM * waypoints * 2); // 2 = one for position one for velocity

        // On duplicate, overwrite cell value with the newest Triplet.
        A.setFromTriplets(linearSystem.begin(), linearSystem.end(), [](const auto &a, const auto &b) { return b; });

        return {
                Eigen::Map<QPVector>(lowerBounds.data(), lowerBounds.size()),
                A,
                Eigen::Map<QPVector>(upperBounds.data(), upperBounds.size())
        };
    }

private:

    const size_t waypoints;
    const double timestep;
    size_t userConstraintOffset = 0;
    std::vector<std::pair<ForwardKinematicsFun, JacobianFun>> mappers;

    std::vector<MatrixCell> linearSystem{};
    std::vector<double> lowerBounds;
    std::vector<double> upperBounds;

    [[nodiscard]] size_t nthVelocity(size_t i) const {
        assert(i < waypoints);
        return waypoints * N_DIM + nthPos(i);
    }

    [[nodiscard]] size_t nthPos(size_t i) const {
        assert(i < waypoints);
        return i * N_DIM;
    }

    [[nodiscard]] size_t nthAcceleration(size_t i) const {
        assert(i < waypoints - 1);
        return waypoints * N_DIM * 2 + i * N_DIM;
    }

    std::pair<std::optional<double>, std::optional<double>>
    getConstraintForNthDim(size_t n, const Constraint<N_DIM> &c) {
        auto &[l_opt, u_opt] = c;
        std::optional<double> l{};
        std::optional<double> u{};
        if (l_opt) l = (*l_opt)[n];
        if (u_opt) u = (*u_opt)[n];
        return {l, u};
    }

    void addConstraint(size_t constraint_idx,
                       const std::vector<Factor> &equation,
                       std::pair<std::optional<double>, std::optional<double>> b) {
        auto &[l, u] = b;
        for (const auto &[variable_idx, coeff]: equation) {
            linearSystem.emplace_back(constraint_idx, variable_idx, coeff);
        }
        if (l) lowerBounds[constraint_idx] = *l;
        if (u) upperBounds[constraint_idx] = *u;
        assert(lowerBounds[constraint_idx] <= upperBounds[constraint_idx]);
    }

    ConstraintBuilder &constrainVariable(size_t n_dim_var_start, const Constraint<N_DIM> &c) {
        for (size_t j = 0; j < N_DIM; ++j) {
            addConstraint(userConstraintOffset + n_dim_var_start + j,
                          {
                                  {n_dim_var_start + j, 1}
                          },
                          getConstraintForNthDim(j, c));
        }
        return *this;
    }

    ConstraintBuilder &variablesInRange(size_t first_start, size_t last_start, const Constraint<N_DIM> &c) {
        for (size_t i = first_start; i <= last_start; i += N_DIM) {
            constrainVariable(i, c);
        }
        return *this;
    }

    void linkVelocityToPosition() {
        for (size_t i = 0; i + 1 < waypoints; ++i) {
            size_t baseV = nthVelocity(i);
            size_t baseP = nthPos(i);
            size_t baseNextP = nthPos(i + 1);
            for (size_t j = 0; j < N_DIM; ++j) {
                lowerBounds.push_back(-INF);
                upperBounds.push_back(INF);
                addConstraint(lowerBounds.size() - 1,
                              {
                                      {baseV + j,     timestep},
                                      {baseNextP + j, -1},
                                      {baseP + j,     1}
                              }, {0, 0});
            }
        }
    }

    void add3dPositionConstraint(size_t constraint_idx,
                                const Constraint<3> &con_3d,
                                const Ctrl<N_DIM> &q,
                                const Point &p,
                                const Jacobian &jacobian,
                                size_t waypoint) {
        auto &[low, upp] = con_3d;
        for (auto axis : XYZ_AXES) {
            double axis_low = -INF;
            double axis_upp = INF;
            if (low.has_value()) {
                axis_low = (*low)[axis];
                axis_low -= p[axis];
                axis_low += jacobian.row(axis) * q;
            }
            if (upp.has_value()) {
                axis_upp = (*upp)[axis];
                axis_upp -= p[axis];
                axis_upp += jacobian.row(axis) * q;
            }
            add3dPositionConstraint(constraint_idx++, axis, jacobian, waypoint, axis_low, axis_upp);
        }
    }

    void addObstacleVerticalConstraint(size_t constraint_idx,
                                const HorizontalLine &obstacle,
                                const Ctrl<N_DIM> &q,
                                const Point &p,
                                const Jacobian &jacobian,
                                size_t waypoint) {
        double bound = obstacle[p][Axis::Z];
        bound -= p[Axis::Z];
        bound += jacobian.row(Axis::Z) * q;

        double low = -INF;
        double upp = INF;
        if (obstacle.bypassFromBelow()) {
            // Bypass from below.
            upp = bound;
        } else {
            // Bypass from above.
            low = bound;
        }
        add3dPositionConstraint(constraint_idx, Axis::Z, jacobian, waypoint, low, upp);
    }

    void add3dPositionConstraint(size_t constraint_idx,
                                Axis axis,
                                const Jacobian &jacobian,
                                size_t waypoint,
                                double low,
                                double upp) {
        std::vector<Factor> constraintFactors(N_DIM);
        for (int i = 0; i < N_DIM; ++i) {
            constraintFactors[i] = {nthPos(waypoint) + i, jacobian(axis, i)};
        }
        //printf("3D position at waypoint %d at axis %d must be in range [%f, %f]", waypoint, axis, low, upp); 
        addConstraint(constraint_idx, constraintFactors, {low, upp});
    }

};

#endif //CONSTRAINT_BUILDER_H
