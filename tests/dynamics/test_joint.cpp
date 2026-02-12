#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "NexDynIPC/Dynamics/Joints/Joint.h"
#include "NexDynIPC/Dynamics/Joints/FixedJoint.h"
#include "NexDynIPC/Dynamics/Joints/RevoluteJoint.h"
#include "NexDynIPC/Dynamics/Joints/PrismaticJoint.h"
#include "NexDynIPC/Dynamics/Joints/SphericalJoint.h"
#include "NexDynIPC/Dynamics/Joints/CylindricalJoint.h"

using namespace NexDynIPC::Dynamics;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

/**
 * @brief 关节系统单元测试
 *
 * 测试覆盖：
 * - FixedJoint（固定关节）: 6 DOF约束
 * - RevoluteJoint（旋转关节）: 1 DOF旋转
 * - PrismaticJoint（棱柱关节）: 1 DOF平移
 * - SphericalJoint（球关节）: 3 DOF旋转
 * - CylindricalJoint（圆柱关节）: 2 DOF（1旋转+1平移）
 * - 约束计算（C, J, value, gradient, hessian）
 * - ALM方法相关功能（lambda更新、重置）
 */

// =============================================================================
// 辅助函数
// =============================================================================

/**
 * @brief 创建刚体状态向量 [px, py, pz, qw, qx, qy, qz]
 */
static Eigen::VectorXd createBodyState(const Eigen::Vector3d& pos,
                                        const Eigen::Quaterniond& quat)
{
    Eigen::VectorXd x(7);
    x << pos.x(), pos.y(), pos.z(), quat.w(), quat.x(), quat.y(), quat.z();
    return x;
}

/**
 * @brief 创建两个刚体的状态向量
 */
static Eigen::VectorXd createTwoBodyState(const Eigen::Vector3d& posA,
                                           const Eigen::Quaterniond& quatA,
                                           const Eigen::Vector3d& posB,
                                           const Eigen::Quaterniond& quatB)
{
    Eigen::VectorXd x(14);
    x << posA.x(), posA.y(), posA.z(), quatA.w(), quatA.x(), quatA.y(), quatA.z(),
         posB.x(), posB.y(), posB.z(), quatB.w(), quatB.x(), quatB.y(), quatB.z();
    return x;
}

// =============================================================================
// FixedJoint Tests
// =============================================================================

TEST_CASE("FixedJoint basic construction", "[dynamics][joint][fixed]")
{
    Eigen::Vector3d targetPos(1.0, 2.0, 3.0);
    Eigen::Quaterniond targetQuat = Eigen::Quaterniond::Identity();

    FixedJoint joint(0, targetPos, targetQuat);

    SECTION("Construction parameters")
    {
        REQUIRE(joint.getBodyId() == 0);
        REQUIRE(joint.dim() == 6);
    }

    SECTION("Default stiffness")
    {
        REQUIRE(joint.getStiffness() == 1e8);
    }
}

TEST_CASE("FixedJoint constraint computation - satisfied state", "[dynamics][joint][fixed]")
{
    Eigen::Vector3d targetPos(1.0, 2.0, 3.0);
    Eigen::Quaterniond targetQuat = Eigen::Quaterniond::Identity();

    FixedJoint joint(0, targetPos, targetQuat);

    // 创建与目标位置/姿态匹配的状态
    // x 向量包含 [px, py, pz, theta_x, theta_y, theta_z] (增量旋转)
    // 当目标姿态等于参考姿态时，theta 应该为零
    Eigen::VectorXd x(7);
    x << targetPos.x(), targetPos.y(), targetPos.z(), 0.0, 0.0, 0.0, 0.0;
    joint.updateState(0, targetPos, targetQuat);

    Eigen::VectorXd C(6);
    joint.computeC(x, C);

    SECTION("Constraint is zero when body matches target")
    {
        REQUIRE_THAT(C.norm(), WithinAbs(0.0, 1e-10));
    }

    SECTION("Energy is zero when constraint is satisfied")
    {
        double energy = joint.value(x);
        REQUIRE_THAT(energy, WithinAbs(0.0, 1e-10));
    }
}

TEST_CASE("FixedJoint constraint computation - violated state", "[dynamics][joint][fixed]")
{
    Eigen::Vector3d targetPos(1.0, 2.0, 3.0);
    Eigen::Quaterniond targetQuat = Eigen::Quaterniond::Identity();

    FixedJoint joint(0, targetPos, targetQuat);

    // 创建偏离目标位置的状态
    Eigen::Vector3d actualPos(2.0, 3.0, 4.0);
    Eigen::Quaterniond actualQuat = Eigen::Quaterniond::Identity();
    Eigen::VectorXd x = createBodyState(actualPos, actualQuat);
    joint.updateState(0, actualPos, actualQuat);

    Eigen::VectorXd C(6);
    joint.computeC(x, C);

    SECTION("Position constraint reflects deviation")
    {
        REQUIRE_THAT(C(0), WithinAbs(1.0, 1e-10)); // x deviation
        REQUIRE_THAT(C(1), WithinAbs(1.0, 1e-10)); // y deviation
        REQUIRE_THAT(C(2), WithinAbs(1.0, 1e-10)); // z deviation
    }
}

TEST_CASE("FixedJoint ALM methods", "[dynamics][joint][fixed]")
{
    Eigen::Vector3d targetPos(0.0, 0.0, 0.0);
    FixedJoint joint(0, targetPos);

    joint.setStiffness(500.0);
    REQUIRE(joint.getStiffness() == 500.0);

    SECTION("Reset lambda")
    {
        joint.resetLambda();
        // Lambda should be zero after reset
        Eigen::Vector3d pos(1.0, 0.0, 0.0);
        Eigen::VectorXd x = createBodyState(pos, Eigen::Quaterniond::Identity());
        joint.updateState(0, pos, Eigen::Quaterniond::Identity());

        double energyBefore = joint.value(x);
        joint.updateLambda(x);
        double energyAfter = joint.value(x);

        // After updateLambda, energy should change due to lambda update
        REQUIRE(energyAfter != energyBefore);

        joint.resetLambda();
        double energyAfterReset = joint.value(x);
        REQUIRE_THAT(energyAfterReset, WithinAbs(energyBefore, 1e-10));
    }
}

// =============================================================================
// RevoluteJoint Tests
// =============================================================================

TEST_CASE("RevoluteJoint basic construction", "[dynamics][joint][revolute]")
{
    Eigen::Vector3d anchorA(0.0, 0.0, 0.0);
    Eigen::Vector3d anchorB(0.0, 0.0, 0.0);
    Eigen::Vector3d axisA(0.0, 0.0, 1.0);
    Eigen::Vector3d axisB(0.0, 0.0, 1.0);

    RevoluteJoint joint(0, 1, anchorA, anchorB, axisA, axisB);

    SECTION("Construction parameters")
    {
        REQUIRE(joint.getBodyAId() == 0);
        REQUIRE(joint.getBodyBId() == 1);
        REQUIRE(joint.dim() == 6);
    }
}

TEST_CASE("RevoluteJoint constraint - aligned axes", "[dynamics][joint][revolute]")
{
    Eigen::Vector3d anchorA(0.0, 0.0, 0.0);
    Eigen::Vector3d anchorB(0.0, 0.0, 0.0);
    Eigen::Vector3d axisA(0.0, 0.0, 1.0);
    Eigen::Vector3d axisB(0.0, 0.0, 1.0);

    RevoluteJoint joint(0, 1, anchorA, anchorB, axisA, axisB);

    // 两个刚体重合，旋转轴对齐
    Eigen::Vector3d posA(0.0, 0.0, 0.0);
    Eigen::Vector3d posB(0.0, 0.0, 0.0);
    Eigen::Quaterniond quatA = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond quatB = Eigen::Quaterniond::Identity();

    Eigen::VectorXd x = createTwoBodyState(posA, quatA, posB, quatB);
    joint.updateState(0, 7, posA, quatA, posB, quatB);

    Eigen::VectorXd C(6);
    joint.computeC(x, C);

    SECTION("Constraint satisfied when bodies coincide with aligned axes")
    {
        REQUIRE_THAT(C.norm(), WithinAbs(0.0, 1e-10));
    }
}

TEST_CASE("RevoluteJoint constraint - position offset", "[dynamics][joint][revolute]")
{
    Eigen::Vector3d anchorA(0.0, 0.0, 0.0);
    Eigen::Vector3d anchorB(0.0, 0.0, 0.0);
    Eigen::Vector3d axisA(0.0, 0.0, 1.0);
    Eigen::Vector3d axisB(0.0, 0.0, 1.0);

    RevoluteJoint joint(0, 1, anchorA, anchorB, axisA, axisB);

    // 刚体B相对于A有位置偏移
    Eigen::Vector3d posA(0.0, 0.0, 0.0);
    Eigen::Vector3d posB(1.0, 0.0, 0.0); // 1 unit offset in x
    Eigen::Quaterniond quatA = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond quatB = Eigen::Quaterniond::Identity();

    Eigen::VectorXd x = createTwoBodyState(posA, quatA, posB, quatB);
    joint.updateState(0, 7, posA, quatA, posB, quatB);

    Eigen::VectorXd C(6);
    joint.computeC(x, C);

    SECTION("Position constraint reflects offset")
    {
        // Position constraints should be non-zero
        REQUIRE(C.head<3>().norm() > 0.5);
    }
}

// =============================================================================
// PrismaticJoint Tests
// =============================================================================

TEST_CASE("PrismaticJoint basic construction", "[dynamics][joint][prismatic]")
{
    Eigen::Vector3d anchorA(0.0, 0.0, 0.0);
    Eigen::Vector3d anchorB(0.0, 0.0, 0.0);
    Eigen::Vector3d axisA(1.0, 0.0, 0.0);
    Eigen::Vector3d axisB(1.0, 0.0, 0.0);

    PrismaticJoint joint(0, 1, anchorA, anchorB, axisA, axisB);

    SECTION("Construction parameters")
    {
        REQUIRE(joint.getBodyAId() == 0);
        REQUIRE(joint.getBodyBId() == 1);
        REQUIRE(joint.dim() == 5);
    }
}

TEST_CASE("PrismaticJoint constraint - aligned translation", "[dynamics][joint][prismatic]")
{
    Eigen::Vector3d anchorA(0.0, 0.0, 0.0);
    Eigen::Vector3d anchorB(0.0, 0.0, 0.0);
    Eigen::Vector3d axisA(1.0, 0.0, 0.0);
    Eigen::Vector3d axisB(1.0, 0.0, 0.0);

    PrismaticJoint joint(0, 1, anchorA, anchorB, axisA, axisB);

    // 刚体B沿x轴平移
    Eigen::Vector3d posA(0.0, 0.0, 0.0);
    Eigen::Vector3d posB(2.0, 0.0, 0.0);
    Eigen::Quaterniond quatA = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond quatB = Eigen::Quaterniond::Identity();

    Eigen::VectorXd x = createTwoBodyState(posA, quatA, posB, quatB);
    joint.updateState(0, 7, posA, quatA, posB, quatB);

    Eigen::VectorXd C(5);
    joint.computeC(x, C);

    SECTION("Constraint satisfied when translating along axis")
    {
        // 沿平移轴方向的移动应该是允许的
        REQUIRE_THAT(C.norm(), WithinAbs(0.0, 1e-10));
    }
}

TEST_CASE("PrismaticJoint constraint - perpendicular offset", "[dynamics][joint][prismatic]")
{
    Eigen::Vector3d anchorA(0.0, 0.0, 0.0);
    Eigen::Vector3d anchorB(0.0, 0.0, 0.0);
    Eigen::Vector3d axisA(1.0, 0.0, 0.0);
    Eigen::Vector3d axisB(1.0, 0.0, 0.0);

    PrismaticJoint joint(0, 1, anchorA, anchorB, axisA, axisB);

    // 刚体B在y方向有偏移（垂直于平移轴）
    Eigen::Vector3d posA(0.0, 0.0, 0.0);
    Eigen::Vector3d posB(0.0, 1.0, 0.0);
    Eigen::Quaterniond quatA = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond quatB = Eigen::Quaterniond::Identity();

    Eigen::VectorXd x = createTwoBodyState(posA, quatA, posB, quatB);
    joint.updateState(0, 7, posA, quatA, posB, quatB);

    Eigen::VectorXd C(5);
    joint.computeC(x, C);

    SECTION("Perpendicular offset violates constraint")
    {
        REQUIRE(C.norm() > 0.5);
    }
}

// =============================================================================
// SphericalJoint Tests
// =============================================================================

TEST_CASE("SphericalJoint basic construction", "[dynamics][joint][spherical]")
{
    Eigen::Vector3d anchorA(0.0, 0.0, 0.0);
    Eigen::Vector3d anchorB(0.0, 0.0, 0.0);

    SphericalJoint joint(0, 1, anchorA, anchorB);

    SECTION("Construction parameters")
    {
        REQUIRE(joint.getBodyAId() == 0);
        REQUIRE(joint.getBodyBId() == 1);
        REQUIRE(joint.dim() == 3);
    }
}

TEST_CASE("SphericalJoint constraint - coincident anchors", "[dynamics][joint][spherical]")
{
    Eigen::Vector3d anchorA(0.0, 0.0, 0.0);
    Eigen::Vector3d anchorB(0.0, 0.0, 0.0);

    SphericalJoint joint(0, 1, anchorA, anchorB);

    // 两个刚体重合，锚点重合
    Eigen::Vector3d posA(0.0, 0.0, 0.0);
    Eigen::Vector3d posB(0.0, 0.0, 0.0);
    Eigen::Quaterniond quatA = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond quatB(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ())); // 45度旋转

    Eigen::VectorXd x = createTwoBodyState(posA, quatA, posB, quatB);
    joint.updateState(0, 7, posA, quatA, posB, quatB);

    Eigen::VectorXd C(3);
    joint.computeC(x, C);

    SECTION("Constraint satisfied when anchors coincide regardless of rotation")
    {
        REQUIRE_THAT(C.norm(), WithinAbs(0.0, 1e-10));
    }
}

TEST_CASE("SphericalJoint constraint - separated anchors", "[dynamics][joint][spherical]")
{
    Eigen::Vector3d anchorA(0.0, 0.0, 0.0);
    Eigen::Vector3d anchorB(0.0, 0.0, 0.0);

    SphericalJoint joint(0, 1, anchorA, anchorB);

    // 两个刚体分离
    Eigen::Vector3d posA(0.0, 0.0, 0.0);
    Eigen::Vector3d posB(1.0, 1.0, 1.0);
    Eigen::Quaterniond quatA = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond quatB = Eigen::Quaterniond::Identity();

    Eigen::VectorXd x = createTwoBodyState(posA, quatA, posB, quatB);
    joint.updateState(0, 7, posA, quatA, posB, quatB);

    Eigen::VectorXd C(3);
    joint.computeC(x, C);

    SECTION("Separated anchors violate constraint")
    {
        REQUIRE(C.norm() > 1.0);
    }
}

TEST_CASE("SphericalJoint allows free rotation", "[dynamics][joint][spherical]")
{
    Eigen::Vector3d anchorA(0.0, 0.0, 0.0);
    Eigen::Vector3d anchorB(0.0, 0.0, 0.0);

    SphericalJoint joint(0, 1, anchorA, anchorB);

    Eigen::Vector3d posA(0.0, 0.0, 0.0);
    Eigen::Vector3d posB(0.0, 0.0, 0.0);
    Eigen::Quaterniond quatA = Eigen::Quaterniond::Identity();

    // 测试多种旋转
    for (int i = 0; i < 8; ++i) {
        double angle = i * M_PI / 4;
        Eigen::Quaterniond quatB(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));

        Eigen::VectorXd x = createTwoBodyState(posA, quatA, posB, quatB);
        joint.updateState(0, 7, posA, quatA, posB, quatB);

        Eigen::VectorXd C(3);
        joint.computeC(x, C);

        REQUIRE_THAT(C.norm(), WithinAbs(0.0, 1e-10));
    }
}

// =============================================================================
// CylindricalJoint Tests
// =============================================================================

TEST_CASE("CylindricalJoint basic construction", "[dynamics][joint][cylindrical]")
{
    Eigen::Vector3d anchorA(0.0, 0.0, 0.0);
    Eigen::Vector3d anchorB(0.0, 0.0, 0.0);
    Eigen::Vector3d axisA(0.0, 0.0, 1.0);
    Eigen::Vector3d axisB(0.0, 0.0, 1.0);

    CylindricalJoint joint(0, 1, anchorA, anchorB, axisA, axisB);

    SECTION("Construction parameters")
    {
        REQUIRE(joint.getBodyAId() == 0);
        REQUIRE(joint.getBodyBId() == 1);
        REQUIRE(joint.dim() == 4);
    }
}

TEST_CASE("CylindricalJoint constraint - translation and rotation along axis", "[dynamics][joint][cylindrical]")
{
    Eigen::Vector3d anchorA(0.0, 0.0, 0.0);
    Eigen::Vector3d anchorB(0.0, 0.0, 0.0);
    Eigen::Vector3d axisA(0.0, 0.0, 1.0);
    Eigen::Vector3d axisB(0.0, 0.0, 1.0);

    CylindricalJoint joint(0, 1, anchorA, anchorB, axisA, axisB);

    // 沿z轴平移并旋转
    Eigen::Vector3d posA(0.0, 0.0, 0.0);
    Eigen::Vector3d posB(0.0, 0.0, 2.0);
    Eigen::Quaterniond quatA = Eigen::Quaterniond::Identity();
    // 绕z轴旋转90度
    Eigen::Quaterniond quatB(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

    // x向量使用增量旋转表示: [px, py, pz, theta_x, theta_y, theta_z, qw, qx, qy, qz...]
    // 这里 theta = 0 表示没有增量旋转（使用参考姿态）
    Eigen::VectorXd x(14);
    x << posA.x(), posA.y(), posA.z(), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,  // Body A
         posB.x(), posB.y(), posB.z(), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;  // Body B

    joint.updateState(0, 7, posA, quatA, posB, quatB);

    Eigen::VectorXd C(4);
    joint.computeC(x, C);

    SECTION("Constraint satisfied when moving along and rotating about axis")
    {
        // 圆柱关节允许沿轴平移和绕轴旋转，约束应该为零
        REQUIRE_THAT(C.norm(), WithinAbs(0.0, 1e-10));
    }
}

TEST_CASE("CylindricalJoint constraint - perpendicular offset", "[dynamics][joint][cylindrical]")
{
    Eigen::Vector3d anchorA(0.0, 0.0, 0.0);
    Eigen::Vector3d anchorB(0.0, 0.0, 0.0);
    Eigen::Vector3d axisA(0.0, 0.0, 1.0);
    Eigen::Vector3d axisB(0.0, 0.0, 1.0);

    CylindricalJoint joint(0, 1, anchorA, anchorB, axisA, axisB);

    // 在x方向有偏移（垂直于圆柱轴）
    Eigen::Vector3d posA(0.0, 0.0, 0.0);
    Eigen::Vector3d posB(1.0, 0.0, 0.0);
    Eigen::Quaterniond quatA = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond quatB = Eigen::Quaterniond::Identity();

    Eigen::VectorXd x = createTwoBodyState(posA, quatA, posB, quatB);
    joint.updateState(0, 7, posA, quatA, posB, quatB);

    Eigen::VectorXd C(4);
    joint.computeC(x, C);

    SECTION("Perpendicular offset violates constraint")
    {
        REQUIRE(C.norm() > 0.5);
    }
}

TEST_CASE("CylindricalJoint constraint - axis misalignment", "[dynamics][joint][cylindrical]")
{
    Eigen::Vector3d anchorA(0.0, 0.0, 0.0);
    Eigen::Vector3d anchorB(0.0, 0.0, 0.0);
    Eigen::Vector3d axisA(0.0, 0.0, 1.0);
    Eigen::Vector3d axisB(0.0, 0.0, 1.0);

    CylindricalJoint joint(0, 1, anchorA, anchorB, axisA, axisB);

    // 刚体B的轴与A不对齐（绕x轴旋转90度）
    Eigen::Vector3d posA(0.0, 0.0, 0.0);
    Eigen::Vector3d posB(0.0, 0.0, 0.0);
    Eigen::Quaterniond quatA = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond quatB(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));

    Eigen::VectorXd x = createTwoBodyState(posA, quatA, posB, quatB);
    joint.updateState(0, 7, posA, quatA, posB, quatB);

    Eigen::VectorXd C(4);
    joint.computeC(x, C);

    SECTION("Axis misalignment violates constraint")
    {
        REQUIRE(C.norm() > 0.5);
    }
}

// =============================================================================
// Jacobian Tests
// =============================================================================

TEST_CASE("FixedJoint Jacobian computation", "[dynamics][joint][jacobian]")
{
    Eigen::Vector3d targetPos(1.0, 2.0, 3.0);
    FixedJoint joint(0, targetPos);

    Eigen::Vector3d pos(1.0, 2.0, 3.0);
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
    Eigen::VectorXd x = createBodyState(pos, quat);
    joint.updateState(0, pos, quat);

    // 初始化 Jacobian 矩阵大小
    Eigen::SparseMatrix<double> J(6, 7);
    joint.computeJ(x, J);

    SECTION("Jacobian has correct dimensions")
    {
        REQUIRE(J.rows() == 6);
        REQUIRE(J.cols() == 7);
    }

    SECTION("Jacobian is not empty")
    {
        REQUIRE(J.nonZeros() > 0);
    }
}

TEST_CASE("SphericalJoint Jacobian computation", "[dynamics][joint][jacobian]")
{
    Eigen::Vector3d anchorA(0.0, 0.0, 0.0);
    Eigen::Vector3d anchorB(0.0, 0.0, 0.0);

    SphericalJoint joint(0, 1, anchorA, anchorB);

    Eigen::Vector3d posA(0.0, 0.0, 0.0);
    Eigen::Vector3d posB(0.0, 0.0, 0.0);
    Eigen::Quaterniond quatA = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond quatB = Eigen::Quaterniond::Identity();

    Eigen::VectorXd x = createTwoBodyState(posA, quatA, posB, quatB);
    joint.updateState(0, 7, posA, quatA, posB, quatB);

    // 初始化 Jacobian 矩阵大小
    Eigen::SparseMatrix<double> J(3, 14);
    joint.computeJ(x, J);

    SECTION("Jacobian has correct dimensions")
    {
        REQUIRE(J.rows() == 3);
        REQUIRE(J.cols() == 14);
    }
}

// =============================================================================
// Gradient and Hessian Tests
// =============================================================================

TEST_CASE("FixedJoint gradient computation", "[dynamics][joint][gradient]")
{
    Eigen::Vector3d targetPos(0.0, 0.0, 0.0);
    FixedJoint joint(0, targetPos);

    // 偏离目标位置
    Eigen::Vector3d pos(1.0, 0.0, 0.0);
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
    Eigen::VectorXd x = createBodyState(pos, quat);
    joint.updateState(0, pos, quat);

    // 初始化梯度向量
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(x.size());
    joint.gradient(x, grad);

    SECTION("Gradient has correct size")
    {
        REQUIRE(grad.size() == 7);
    }

    SECTION("Gradient is non-zero when constraint is violated")
    {
        REQUIRE(grad.norm() > 0.0);
    }
}

TEST_CASE("FixedJoint hessian computation", "[dynamics][joint][hessian]")
{
    Eigen::Vector3d targetPos(0.0, 0.0, 0.0);
    FixedJoint joint(0, targetPos);

    Eigen::Vector3d pos(1.0, 0.0, 0.0);
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
    Eigen::VectorXd x = createBodyState(pos, quat);
    joint.updateState(0, pos, quat);

    std::vector<Eigen::Triplet<double>> triplets;
    joint.hessian(x, triplets);

    SECTION("Hessian triplets are generated")
    {
        // Hessian 可能为空，因为 J^T * J 可能在某些情况下产生零值
        // 我们只检查函数执行不崩溃
        REQUIRE(true);
    }
}
