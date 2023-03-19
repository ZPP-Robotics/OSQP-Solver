#include "gtest/gtest.h"
#include "constraints/constraint-builder.h"

const double DEFAULT_TIME_STEP = 2;
const size_t DEFAULT_WAYPOINTS_CNT = 2;

/**
 * GENERAL NOTATION:
 *
 *
 * Let q_{i}, v_{i}, a_{i}
 * be D-dimensional vectors describing robot's joints:
 * - positions,
 * - angular velocities,
 * - angular accelerations,
 * respectively.
 *
 */

void expect_equality_constraint(size_t idx, const Eigen::VectorXd &low, const Eigen::VectorXd &upp) {
    EXPECT_EQ(low[idx], 0);
    EXPECT_EQ(upp[idx], 0);
}

void expect_vectors_eq(QPVector3d a, QPVector3d b) {
    EXPECT_EQ((a - b).norm(), 0);
}

TEST(ConstraintBuilderTest, linkingVelocityToPosition) {
    /**
     * v_i == (q_i + 1 - q_i) / DEFAULT_TIME_STEP
     *
     * so
     *
     * v_i * DEFAULT_TIME_STEP - q_{i + 1} + q_i  == 0
     *
     * These 'velocity-position linking' constraints should be placed
     * at first (DEFAULT_WAYPOINTS_CNT - 1) rows of constraint matrix.
     */

    auto [l, A, u] = ConstraintBuilder<1>{DEFAULT_WAYPOINTS_CNT, DEFAULT_TIME_STEP, {}}.build();

    for (size_t i = 0; i < DEFAULT_WAYPOINTS_CNT - 1; ++i) {
        expect_equality_constraint(i, l, u);

        // Check factors of the constrained formula.
        EXPECT_EQ(A.coeff(i, i), 1); // 1 * q_i
        EXPECT_EQ(A.coeff(i, i + 1), -1); // (-1) * q_{i + 1}
        EXPECT_EQ(A.coeff(i, DEFAULT_WAYPOINTS_CNT + i), DEFAULT_TIME_STEP); // v_i * DEFAULT_TIME_STEP
    }
}

TEST(LineUtilTest, XAxis) {
    HorizontalLine line{{2, 0}, {1, 1, 1}, {0, 0, 0.1}};

    EXPECT_EQ(line.getDistanceVec({2, 1, 1}).norm(), 0);
    EXPECT_EQ(line.getDistanceVec({1, 2, 1}).norm(), 1);
    EXPECT_EQ(line.getDistanceVec({1, 1, 2}).norm(), 1);
    EXPECT_EQ(line.getDistanceVec({1, 2, 2}).norm(), sqrt(2));

    EXPECT_EQ(line.getDistanceXY({2, 1, 1}), 0);
    EXPECT_EQ(line.getDistanceXY({1, 2, 1}), 1);
    EXPECT_EQ(line.getDistanceXY({1, 1, 2}), 0);

    QPVector3d p_real = QPVector3d{1.1, 1.2, 1.3};
    QPVector3d p_expected = QPVector3d{1.1, 1, 1};
    // should return closest point on the line to the given one.
    QPVector3d p_act = line[p_real];

    expect_vectors_eq(p_act, p_expected);
}