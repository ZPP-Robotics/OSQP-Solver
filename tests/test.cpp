#include "gtest/gtest.h"
#include "constraints/constraint-builder.h"

const size_t DEFAULT_WAYPOINTS_CNT = 2;
using namespace Eigen;

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

void expect_vectors_eq(QPVector3d a, QPVector3d b) {
    EXPECT_EQ((a - b).norm(), 0);
}

size_t firstPositionConstraintIdx(size_t waypoints, size_t dims) {
    return (waypoints - 1) * dims;
}

size_t firstVelocityConstraintIdx(size_t waypoints, size_t dims) {
    return firstPositionConstraintIdx(waypoints, dims) + waypoints * dims;
}

size_t firstAccelerationConstraintIdx(size_t waypoints, size_t dims) {
    return firstVelocityConstraintIdx(waypoints, dims) + (waypoints - 1) * dims;
}

size_t variablesCount(size_t waypoints, size_t dims) {
    return 2 * waypoints * dims;
}

TEST(ConstraintBuilderTest, linkingVelocityToPosition) {
    /**
     * v_i == (q_i + 1 - q_i)
     *
     * so
     *
     * v_i - q_{i + 1} + q_i  == 0
     *
     * These 'velocity-position linking' constraints should be placed
     * at first (DEFAULT_WAYPOINTS_CNT - 1) rows of constraint matrix.
     */
    constexpr size_t dims = 2;
    size_t waypoints = 3;

    auto [l, A, u] = ConstraintBuilder<dims>{waypoints, {}, {}}.build();

    int start_row = 0;
    int rows_count = (waypoints - 1) * dims;
    MatrixXd expected_A(rows_count, variablesCount(waypoints, dims));
    VectorXd expected_l(rows_count);
    VectorXd expected_u(rows_count);

    expected_A << 
        1, 0, -1, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 1, 0, -1, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, -1, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, -1, 0, 0, 0, 1, 0, 0;
  
    expected_l << 0, 0, 0, 0;

    expected_u << 0, 0, 0, 0;

    EXPECT_TRUE(A.middleRows(start_row, rows_count).isApprox(expected_A));
    EXPECT_TRUE(l.segment(start_row, rows_count).isApprox(expected_l));
    EXPECT_TRUE(u.segment(start_row, rows_count).isApprox(expected_u));
}

TEST(LineUtilTest, XAxis) {
    HorizontalLine line{{2, 0}, {1, 1, 1}};

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

TEST(IndicesTest, position) {
    auto builder = ConstraintBuilder<2>{3, {}, {}};

    EXPECT_EQ(builder.nthPos(0), 0);
    EXPECT_EQ(builder.nthPos(1), 2);
    EXPECT_EQ(builder.nthPos(2), 4);
}

TEST(IndicesTest, velocity) {
    auto builder = ConstraintBuilder<2>{3, {}, {}};

    EXPECT_EQ(builder.nthVelocity(0), 6);
    EXPECT_EQ(builder.nthVelocity(1), 8);
}

TEST(IndicesTest, acceleration) {
    auto builder = ConstraintBuilder<2>{4, {}, {}};

    EXPECT_EQ(builder.nthAcceleration(0), 14);
    EXPECT_EQ(builder.nthAcceleration(1), 16);
}

TEST(ConstraintsTest, jointPosition) {
    constexpr size_t dims = 2;
    size_t waypoints = 3;

    auto [l, A, u] = ConstraintBuilder<dims>{waypoints, {}, {}}
        .positions(0, waypoints - 1, constraints::inRange<2>({{1, 2}}, {{3, 4}}))
        .build();

    int start_row = firstPositionConstraintIdx(waypoints, dims);
    int rows_count = waypoints * dims;
    MatrixXd expected_A(rows_count, variablesCount(waypoints, dims));
    VectorXd expected_l(rows_count);
    VectorXd expected_u(rows_count);

    expected_A << 
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
  
    expected_l << 1, 2, 1, 2, 1, 2;

    expected_u << 3, 4, 3, 4, 3, 4;

    EXPECT_TRUE(A.middleRows(start_row, rows_count).isApprox(expected_A));
    EXPECT_TRUE(l.segment(start_row, rows_count).isApprox(expected_l));
    EXPECT_TRUE(u.segment(start_row, rows_count).isApprox(expected_u));
}

TEST(ConstraintsTest, velocity) {
    constexpr size_t dims = 2;
    size_t waypoints = 3;

    auto [l, A, u] = ConstraintBuilder<dims>{waypoints, {}, {}}
        .velocities(0, waypoints - 2, constraints::inRange<2>({{1, 2}}, {{3, 4}}))
        .build();

    int start_row = firstVelocityConstraintIdx(waypoints, dims);
    int rows_count = (waypoints - 1) * dims;
    MatrixXd expected_A(rows_count, variablesCount(waypoints, dims));
    VectorXd expected_l(rows_count);
    VectorXd expected_u(rows_count);

    expected_A << 
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
  
    expected_l << 1, 2, 1, 2;

    expected_u << 3, 4, 3, 4;

    EXPECT_TRUE(A.middleRows(start_row, rows_count).isApprox(expected_A));
    EXPECT_TRUE(l.segment(start_row, rows_count).isApprox(expected_l));
    EXPECT_TRUE(u.segment(start_row, rows_count).isApprox(expected_u));
}

TEST(ConstraintsTest, acceleration) {
    constexpr size_t dims = 2;
    size_t waypoints = 3;

    auto [l, A, u] = ConstraintBuilder<dims>{waypoints, {}, {}}
        .accelerations(0, waypoints - 3, constraints::inRange<2>({{1, 2}}, {{3, 4}}))
        .build();

    int start_row = firstAccelerationConstraintIdx(waypoints, dims);
    int rows_count = (waypoints - 2) * dims;
    MatrixXd expected_A(rows_count, variablesCount(waypoints, dims));
    VectorXd expected_l(rows_count);
    VectorXd expected_u(rows_count);

    expected_A << 
        0, 0, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, -1, 0, 1, 0, 0,
  
    expected_l << 1, 2;

    expected_u << 3, 4;

    EXPECT_TRUE(A.middleRows(start_row, rows_count).isApprox(expected_A));
    EXPECT_TRUE(l.segment(start_row, rows_count).isApprox(expected_l));
    EXPECT_TRUE(u.segment(start_row, rows_count).isApprox(expected_u));
}

TEST(ConstraintsTest, all) {
    constexpr size_t dims = 2;
    size_t waypoints = 3;

    auto [l, A, u] = ConstraintBuilder<dims>{waypoints, {}, {}}
        .positions(0, waypoints - 1, constraints::inRange<2>({{1, 2}}, {{3, 4}}))
        .velocities(0, waypoints - 2, constraints::inRange<2>({{5, 6}}, {{7, 8}}))
        .accelerations(0, waypoints - 3, constraints::inRange<2>({{9, 10}}, {{11, 12}}))
        .build();

    int start_row = firstPositionConstraintIdx(waypoints, dims);
    int rows_count = (waypoints + waypoints - 1 + waypoints - 2) * dims;
    MatrixXd expected_A(rows_count, variablesCount(waypoints, dims));
    VectorXd expected_l(rows_count);
    VectorXd expected_u(rows_count);

    expected_A << 
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, -1, 0, 1, 0, 0,
  
    expected_l << 1, 2, 1, 2, 1, 2, 5, 6, 5, 6, 9, 10;

    expected_u << 3, 4, 3, 4, 3, 4, 7, 8, 7, 8, 11, 12;

    EXPECT_TRUE(A.middleRows(start_row, rows_count).isApprox(expected_A));
    EXPECT_TRUE(l.segment(start_row, rows_count).isApprox(expected_l));
    EXPECT_TRUE(u.segment(start_row, rows_count).isApprox(expected_u));
}