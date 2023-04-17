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

size_t first3dPositionConstraintIdx(size_t waypoints, size_t dims) {
    return firstAccelerationConstraintIdx(waypoints, dims) + (waypoints - 2) * dims;
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

TEST(ConstraintsTest, position3d_2) {
    constexpr size_t dims = 3;
    size_t waypoints = 2;
    ForwardKinematicsFun pow2_fk = [](double q[3]) {
        static int cnt = 0;
        return std::tuple<double, double, double>{1 << (cnt++), 1 << (cnt++), 1 << (cnt++)};
    };

    JacobianFun const_jac = [](double *out, double q[3]) {
        // out = 3x3 matrix in row major order.
        out[0] = 0;
        out[1] = 1;
        out[2] = 2;
        out[3] = 3;
        out[4] = 4;
        out[5] = 5;
        out[6] = 6;
        out[7] = 7;
        out[8] = 8;
    };

    VectorXd trajectory = VectorXd::Constant(waypoints * dims * 2, 1);
    auto [l, A, u] = ConstraintBuilder<dims>{waypoints, {RobotBall{pow2_fk, const_jac, 0, true}}, {}}
        .withObstacles(constraints::inRange<3>({{11, 22, 33}}, {{44, 55, 66}}), trajectory)
        .build();

    int start_row = first3dPositionConstraintIdx(waypoints, dims);
    int rows_count = waypoints * 3;
    MatrixXd expected_A(rows_count, variablesCount(waypoints, dims));
    VectorXd expected_l(rows_count);
    VectorXd expected_u(rows_count);

    expected_A << 
        0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        3, 4, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 3, 4, 5, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 6, 7, 8, 0, 0, 0, 0, 0, 0;
  
    double low_x = 11 + 0 + 1 + 2;
    double low_y = 22 + 3 + 4 + 5;
    double low_z = 33 + 6 + 7 + 8;
    expected_l << low_x - 1, low_y - 2, low_z - 4, low_x - 8, low_y - 16, low_z - 32;

    double upp_x = 44 + 0 + 1 + 2;
    double upp_y = 55 + 3 + 4 + 5;
    double upp_z = 66 + 6 + 7 + 8;
    expected_u << upp_x - 1, upp_y - 2, upp_z - 4, upp_x - 8, upp_y - 16, upp_z - 32;

    EXPECT_TRUE(A.middleRows(start_row, rows_count).isApprox(expected_A));
    EXPECT_TRUE(l.segment(start_row, rows_count).isApprox(expected_l));
    EXPECT_TRUE(u.segment(start_row, rows_count).isApprox(expected_u));
}

TEST(ConstraintsTest, position3d_1) {
    constexpr size_t dims = 3;
    size_t waypoints = 2;
    ForwardKinematicsFun identity_fk = [](double q[3]) {
        return std::tuple<double, double, double>{q[0], q[1], q[2]};
    };

    JacobianFun const_jac = [](double *out, double q[3]) {
        // out = 3x3 matrix in row major order.
        out[0] = 0;
        out[1] = 1;
        out[2] = 2;
        out[3] = 3;
        out[4] = 4;
        out[5] = 5;
        out[6] = 6;
        out[7] = 7;
        out[8] = 8;
    };

    VectorXd trajectory = VectorXd::Constant(waypoints * dims * 2, 1);
    auto [l, A, u] = ConstraintBuilder<dims>{waypoints, {RobotBall{identity_fk, const_jac, 0, true}}, {}}
        .withObstacles(constraints::inRange<3>({{11, 22, 33}}, {{44, 55, 66}}), trajectory)
        .build();

    int start_row = first3dPositionConstraintIdx(waypoints, dims);
    int rows_count = waypoints * 3;
    MatrixXd expected_A(rows_count, variablesCount(waypoints, dims));
    VectorXd expected_l(rows_count);
    VectorXd expected_u(rows_count);

    expected_A << 
        0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        3, 4, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 3, 4, 5, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 6, 7, 8, 0, 0, 0, 0, 0, 0;
  
    double low_x = 11 - 1 + 0 + 1 + 2;
    double low_y = 22 - 1 + 3 + 4 + 5;
    double low_z = 33 - 1 + 6 + 7 + 8;
    expected_l << low_x, low_y, low_z, low_x, low_y, low_z;

    double upp_x = 44 - 1 + 0 + 1 + 2;
    double upp_y = 55 - 1 + 3 + 4 + 5;
    double upp_z = 66 - 1 + 6 + 7 + 8;
    expected_u << upp_x, upp_y, upp_z, upp_x, upp_y, upp_z;

    EXPECT_TRUE(A.middleRows(start_row, rows_count).isApprox(expected_A));
    EXPECT_TRUE(l.segment(start_row, rows_count).isApprox(expected_l));
    EXPECT_TRUE(u.segment(start_row, rows_count).isApprox(expected_u));
}

TEST(ConstraintsTest, position3d_jac_pow2) {
    constexpr size_t dims = 3;
    size_t waypoints = 2;
    ForwardKinematicsFun identity_fk = [](double q[3]) {
        return std::tuple<double, double, double>{q[0], q[1], q[2]};
    };

    JacobianFun const_jac = [](double *out, double q[3]) {
        // out = 3x3 matrix in row major order.
        out[0] = 0;
        out[1] = 1;
        out[2] = 2;
        out[3] = 4;
        out[4] = 8;
        out[5] = 16;
        out[6] = 32;
        out[7] = 64;
        out[8] = 128;
    };

    VectorXd trajectory = VectorXd::Constant(waypoints * dims * 2, 2);
    auto [l, A, u] = ConstraintBuilder<dims>{waypoints, {RobotBall{identity_fk, const_jac, 0, true}}, {}}
        .withObstacles(constraints::inRange<3>({{11, 22, 33}}, {{44, 55, 66}}), trajectory)
        .build();

    int start_row = first3dPositionConstraintIdx(waypoints, dims);
    int rows_count = waypoints * 3;
    VectorXd expected_l(rows_count);
    VectorXd expected_u(rows_count);
  
    double low_x = 11 - 2 + 0 + 2 + 4;
    double low_y = 22 - 2 + 8 + 16 + 32;
    double low_z = 33 - 2 + 64 + 128 + 256;
    expected_l << low_x, low_y, low_z, low_x, low_y, low_z;

    double upp_x = 44 - 2 + 0 + 2 + 4;
    double upp_y = 55 - 2 + 8 + 16 + 32;
    double upp_z = 66 - 2 + 64 + 128 + 256;
    expected_u << upp_x, upp_y, upp_z, upp_x, upp_y, upp_z;

    EXPECT_TRUE(l.segment(start_row, rows_count).isApprox(expected_l));
    EXPECT_TRUE(u.segment(start_row, rows_count).isApprox(expected_u));
}

TEST(ConstraintsTest, ignore_velocity_trajectory) {
    constexpr size_t dims = 3;
    size_t waypoints = 2;
    ForwardKinematicsFun identity_fk = [](double q[3]) {
        return std::tuple<double, double, double>{q[0], q[1], q[2]};
    };

    JacobianFun const_jac = [](double *out, double q[3]) {
        // out = 3x3 matrix in row major order.
        out[0] = 0;
        out[1] = 1;
        out[2] = 2;
        out[3] = 4;
        out[4] = 8;
        out[5] = 16;
        out[6] = 32;
        out[7] = 64;
        out[8] = 128;
    };

    VectorXd trajectory(waypoints * dims * 2);
    trajectory <<
        2, 2, 2, 2, 2, 2, 1024, 1024, 1024, 1024, 1024, 1024;

    auto [l, A, u] = ConstraintBuilder<dims>{waypoints, {RobotBall{identity_fk, const_jac, 0, true}}, {}}
        .withObstacles(constraints::inRange<3>({{11, 22, 33}}, {{44, 55, 66}}), trajectory)
        .build();

    int start_row = first3dPositionConstraintIdx(waypoints, dims);
    int rows_count = waypoints * 3;
    VectorXd expected_l(rows_count);
    VectorXd expected_u(rows_count);
  
    double low_x = 11 - 2 + 0 + 2 + 4;
    double low_y = 22 - 2 + 8 + 16 + 32;
    double low_z = 33 - 2 + 64 + 128 + 256;
    expected_l << low_x, low_y, low_z, low_x, low_y, low_z;

    double upp_x = 44 - 2 + 0 + 2 + 4;
    double upp_y = 55 - 2 + 8 + 16 + 32;
    double upp_z = 66 - 2 + 64 + 128 + 256;
    expected_u << upp_x, upp_y, upp_z, upp_x, upp_y, upp_z;

    EXPECT_TRUE(l.segment(start_row, rows_count).isApprox(expected_l));
    EXPECT_TRUE(u.segment(start_row, rows_count).isApprox(expected_u));
}

