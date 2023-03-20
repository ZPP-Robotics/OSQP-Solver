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

using ForwardKinematics = std::function<std::tuple<double, double, double>(double *)>;
using Jacobian = std::function<void(double *, double *)>;
using JointIndex = size_t;

template<size_t N_DIM>
class ConstraintBuilder {

    using Factor = std::pair<size_t, double>;
    using MatrixCell = Eigen::Triplet<double, size_t>;

    // position + velocity + acceleration + linearized constraints for obstacles avoidance
    static constexpr const size_t DYNAMICS_DERIVATIVES = 4;

public:

    ConstraintBuilder(size_t waypoints, double timestep,
                      std::map<JointIndex, std::pair<ForwardKinematics, Jacobian>> m)
            : waypoints(waypoints), timestep(timestep), mappers(std::move(m)) {
        for (const auto &[joint_idx, mapper]: mappers) assert(joint_idx < N_DIM);

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

    ConstraintBuilder &withObstacles(
            const std::vector<HorizontalLine> &obstacles,
            const QPVector &trajectory) {
        size_t obstacle_constraints_base = userConstraintOffset + N_DIM * waypoints * 3;
        size_t next_obstacle_constraint_idx = obstacle_constraints_base;

        for (const auto &obstacle : obstacles) {

            // Make each specified joint avoid a given obstacle.
            for (const auto &[joint_idx, fk_jac]: mappers) {
                const auto &[forward_kinematics, jacobian_calculator] = fk_jac;

                // Calculate position of the specified joint at each waypoint of the current trajectory,
                // and constrain its position if it is close to the obstacle.
                bool isAlreadyConstrained = false;
                Ctrl<N_DIM> q = trajectory.segment(nthPos(0), N_DIM);
                QPMatrix<XYZ_AXES.size(), N_DIM> jac;

                auto [x, y, z] = forward_kinematics((double *) q.data());
                QPVector3d p = {x, y, z};
                jacobian_calculator((double *) jac.data(), (double *) q.data());


                
                for (size_t waypoint = 0; waypoint < waypoints; ++waypoint) {
                    if (waypoint + 1 < waypoints) {
                        // has next waypoint.
                        Ctrl<N_DIM> q_next = trajectory.segment(nthPos(waypoint + 1), N_DIM);
                        QPMatrix<XYZ_AXES.size(), N_DIM> jac_next;
                        auto [x, y, z] = forward_kinematics((double *) q_next.data());
                        QPVector3d p_next = {x, y, z};
                        jacobian_calculator((double *) jac_next.data(), (double *) q_next.data());

                        if (obstacle.areOnOppositeSides(p, p_next)) {
                            if (!isAlreadyConstrained) {
                                addObstacleConstraint(next_obstacle_constraint_idx++, q, p, jac, obstacle, waypoint);
                            }
                            addObstacleConstraint(next_obstacle_constraint_idx++, q_next, p_next, jac_next, obstacle, waypoint + 1);
                            isAlreadyConstrained = true;
                        } else {
                            if (!isAlreadyConstrained) {
                                if (obstacle.isClose(p, CENTIMETER * 10)) {
                                    addObstacleConstraint(next_obstacle_constraint_idx++, q, p, jac, obstacle, waypoint);
                                } else {
                                    addDummyObstacleConstraint(next_obstacle_constraint_idx++, jac, waypoint);
                                }
                            }
                            isAlreadyConstrained = false;
                        }
                        q = q_next;
                        p = p_next;
                        jac = jac_next;
                    } else if (!isAlreadyConstrained) {
                        if (obstacle.isClose(p, CENTIMETER * 10)) {
                            addObstacleConstraint(next_obstacle_constraint_idx, q, p, jac, obstacle, waypoint);
                        } else {
                            addDummyObstacleConstraint(next_obstacle_constraint_idx, jac, waypoint);
                        }
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
    std::map<size_t, std::pair<ForwardKinematics, Jacobian>> mappers;

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

    [[nodiscard]] size_t nthZObstacle(size_t i) const {
        assert(i < waypoints - 1);
        return waypoints * N_DIM * 3 + i * N_DIM;
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

    void addObstacleConstraint(size_t constraint_idx,
                                const Ctrl<N_DIM> &q,
                                const QPVector3d &p,
                                const QPMatrix<XYZ_AXES.size(), N_DIM> &jacobian,
                                const HorizontalLine &obstacle,
                                size_t waypoint) {
        double low = -INF;
        double upp = INF;

        //printf("position (x=%f, y=%f, z=%f), closest on obstacle: (x=%f, y=%f, z=%f)\n", x, y, z, obstacle[tipXYZ][0], obstacle[tipXYZ][1], obstacle[tipXYZ][2]);
        double offset = obstacle.getBypassOffset()[Axis::Z];
        double bound = obstacle[p][Axis::Z] + offset;
        bound -= p[Axis::Z];
        bound += jacobian.row(Axis::Z) * q;

        if (offset > CENTIMETER) {
            //printf("position (x=%f, y=%f, z=%f) is close to obstacle, so position at axis=%d should be >= %f\n", x, y, z, Axis::Z,  obstacle[tipXYZ][Axis::Z] + offset);
            // Bypass from above.
            low = bound;
        } else if (offset < -CENTIMETER) {
            //printf("position (x=%f, y=%f, z=%f) is close to obstacle, so position at axis=%d should be <= %f\n", x, y, z, Axis::Z,  obstacle[tipXYZ][Axis::Z] + offset);
            // Bypass from below.
            upp = bound;
        }

        addObstacleConstraint(constraint_idx, jacobian, waypoint, low, upp);
    }

    void addDummyObstacleConstraint(size_t constraint_idx,
                                    const QPMatrix<XYZ_AXES.size(), N_DIM> &jacobian,
                                    size_t waypoint) {
        addObstacleConstraint(constraint_idx, jacobian, waypoint, -INF, INF);                            
    }

    void addObstacleConstraint(size_t constraint_idx,
                                const QPMatrix<XYZ_AXES.size(), N_DIM> &jacobian,
                                size_t waypoint,
                                double low,
                                double upp) {
        std::vector<Factor> constraintFactors(N_DIM);
        for (int i = 0; i < N_DIM; ++i) {
            constraintFactors[i] = {nthPos(waypoint) + i, jacobian(Axis::Z, i)};
        }
        addConstraint(constraint_idx, constraintFactors, {low, upp});
    }
};

#endif //CONSTRAINT_BUILDER_H
