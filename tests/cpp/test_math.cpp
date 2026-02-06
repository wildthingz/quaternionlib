//
// Created by hatef on 2026-02-03.
//

#include <gtest/gtest.h>
#include <vector>
#include <numbers>
#include "QuaternionLib/quaternion.h"
#include "QuaternionLib/math_utils.h" // Include the new math utils

using namespace QuatLib;

// --- Constants ---
// Use library constants instead of redefining them to avoid ambiguity
constexpr double PI_VAL = QuatLib::Utils::PI<double>;
constexpr double EPS_VAL = 1e-4; // Keep this one distinct for testing tolerances

// FIX: Added 'inline' to prevent multiple definition linker errors
inline double deg2rad(double deg) { return deg * PI_VAL / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / PI_VAL; }

// --- Helpers ---

// Map enum to basis vectors for test construction
std::array<int, 3> get_axes(RotationOrder order) {
    switch(order) {
        case RotationOrder::XYX: return {0,1,0};
        case RotationOrder::YZY: return {1,2,1};
        case RotationOrder::ZXZ: return {2,0,2};
        case RotationOrder::XZX: return {0,2,0};
        case RotationOrder::YXY: return {1,0,1};
        case RotationOrder::ZYZ: return {2,1,2};

        case RotationOrder::XYZ: return {0,1,2};
        case RotationOrder::YZX: return {1,2,0};
        case RotationOrder::ZXY: return {2,0,1};
        case RotationOrder::XZY: return {0,2,1};
        case RotationOrder::YXZ: return {1,0,2};
        case RotationOrder::ZYX: return {2,1,0};
    }
    return {0,0,0};
}

// Construct Quaternion from Intrinsic Euler Angles (Axis 1 -> Axis 2 -> Axis 3)
Eigen::Quaterniond make_q(RotationOrder order, double a, double b, double c) {
    auto idx = get_axes(order);
    Eigen::Vector3d axes[] = {
        Eigen::Vector3d::UnitX(),
        Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitZ()
    };

    using AngleAxis = Eigen::AngleAxisd;
    // Intrinsic Rotation: Q = Q1 * Q2 * Q3
    return AngleAxis(a, axes[idx[0]])
         * AngleAxis(b, axes[idx[1]])
         * AngleAxis(c, axes[idx[2]]);
}

// --- Tests ---

class MathTest : public ::testing::TestWithParam<RotationOrder> {};

// 1. Analytic Correctness (Non-Singular)
// We test "Round Trip": Angles -> Quat -> Angles.
// Note: Euler angles are not unique (e.g. 180 = -180), so we check if the
// Reconstructed Quaternion matches the Original Quaternion.
TEST_P(MathTest, RoundTrip_Analytic) {
    RotationOrder order = GetParam();

    // Arbitrary non-singular angles
    double a = deg2rad(10.0);
    double b = deg2rad(20.0);
    double c = deg2rad(30.0);

    // 1. Create Ground Truth
    Eigen::Quaterniond q_orig = make_q(order, a, b, c);

    // 2. Convert using Library
    Quaternion<double> quat(q_orig, order);
    std::vector<EulerAngles<double>> history; // Empty history
    auto result = quat.to_eul(history);

    // 3. Reconstruct Quaternion from Result
    Eigen::Quaterniond q_recon = make_q(order, result.angles[0], result.angles[1], result.angles[2]);

    // 4. Compare (Dot product of unit quats should be +/- 1)
    double dot = std::abs(q_orig.dot(q_recon));
    EXPECT_NEAR(dot, 1.0, EPS_VAL) << "Failed for order " << (int)order;
}

// Instantiate for ALL 12 orders
INSTANTIATE_TEST_SUITE_P(
    AllOrders,
    MathTest,
    ::testing::Values(
        RotationOrder::XYX, RotationOrder::YZY, RotationOrder::ZXZ,
        RotationOrder::XZX, RotationOrder::YXY, RotationOrder::ZYZ,
        RotationOrder::XYZ, RotationOrder::YZX, RotationOrder::ZXY,
        RotationOrder::XZY, RotationOrder::YXZ, RotationOrder::ZYX
    )
);

// 2. Proper Euler Singularity (Pitch = 0)
TEST(MathTest, ProperEuler_Singularity_Zero) {
    // ZXZ Order. Pitch (Y) = 0.
    // At pitch 0, Z1 and Z3 axes align. Alpha and Gamma are locked.
    // Q = RotZ(10) * RotX(0) * RotZ(20) = RotZ(30).
    // Expected: Alpha + Gamma = 30.

    RotationOrder order = RotationOrder::ZXZ;
    double a = deg2rad(10.0);
    double b = 0.0;
    double c = deg2rad(20.0);

    Eigen::Quaterniond q = make_q(order, a, b, c);
    Quaternion<double> quat(q, order);

    // Provide no history -> Should default Alpha to 0
    std::vector<EulerAngles<double>> history;
    auto result = quat.to_eul(history);

    // With Alpha forced to 0 (default), Gamma should be 30.
    // Or library solves such that Alpha+Gamma = 30.

    double alpha_deg = rad2deg(result.angles[0]);
    double gamma_deg = rad2deg(result.angles[2]);

    EXPECT_NEAR(alpha_deg, 0.0, EPS_VAL); // Default behavior without history
    EXPECT_NEAR(gamma_deg, 30.0, EPS_VAL);
}

// 3. Tait-Bryan Singularity (South Pole, Pitch = -90)
TEST(MathTest, TaitBryan_SouthPole) {
    // ZYX Order. Pitch = -90.
    // Q = RotZ(10) * RotY(-90) * RotX(20).

    RotationOrder order = RotationOrder::ZYX;
    double a = deg2rad(10.0); // Yaw
    double b = deg2rad(-90.0);// Pitch (South Pole)
    double c = deg2rad(20.0); // Roll

    Eigen::Quaterniond q = make_q(order, a, b, c);
    Quaternion<double> quat(q, order);

    std::vector<EulerAngles<double>> history;
    auto result = quat.to_eul(history);

    double pitch_deg = rad2deg(result.angles[1]);
    EXPECT_NEAR(pitch_deg, -90.0, EPS_VAL);

    // Reconstruct to verify validity (since multiple solutions exist)
    Eigen::Quaterniond q_recon = make_q(order, result.angles[0], result.angles[1], result.angles[2]);
    double dot = std::abs(q.dot(q_recon));
    EXPECT_NEAR(dot, 1.0, EPS_VAL);
}

// 4. Matrix Conversion Check
TEST(MathTest, ToRotationMatrix) {
    RotationOrder order = RotationOrder::XYZ;
    Eigen::Quaterniond q = make_q(order, 0.5, 0.5, 0.5);

    Quaternion<double> quat(q, order);
    auto mat_struct = quat.to_rotation_matrix();

    Eigen::Matrix3d mat_eigen = q.toRotationMatrix();

    // Compare matrix elements
    double diff = (mat_struct.matrix - mat_eigen).norm();
    EXPECT_NEAR(diff, 0.0, EPS_VAL);
}