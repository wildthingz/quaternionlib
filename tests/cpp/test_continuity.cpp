#include <gtest/gtest.h>
#include <vector>
#include <numbers>
#include "QuaternionLib/quaternion.h"
#include "QuaternionLib/math_utils.h"
#include "QuaternionLib/stateful_converter.h" // Phase 2: Stateful API

using namespace QuatLib;

// Use library constants
constexpr double PI_VAL = QuatLib::Utils::PI<double>;

// FIX: Added 'inline' to prevent multiple definition linker errors
inline double deg2rad(double deg) { return deg * PI_VAL / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / PI_VAL; }

// --- Helper: Construct Quaternion Correctly ---
// Standard ZYX (Yaw-Pitch-Roll) means: Rotate Z, then Y, then X.
// In Eigen (and standard math), q_combined = q_z * q_y * q_x
Eigen::Quaterniond from_euler_zyx(double roll_deg, double pitch_deg, double yaw_deg) {
    using Vector3 = Eigen::Vector3d;
    using AngleAxis = Eigen::AngleAxisd;

    return AngleAxis(deg2rad(yaw_deg),   Vector3::UnitZ())
         * AngleAxis(deg2rad(pitch_deg), Vector3::UnitY())
         * AngleAxis(deg2rad(roll_deg),  Vector3::UnitX());
}

TEST(ContinuityTest, SolvesGimbalLock_NorthPole) {
    // SCENARIO: Pitch approaches 90 degrees (North Pole Singularity).
    // We expect the solver to use history to keep Roll/Yaw stable.

    std::vector<EulerAngles<double>> history;
    RotationOrder order = RotationOrder::ZYX;

    // Target state we want to preserve:
    double steady_roll = 45.0;
    double steady_yaw  = 10.0;

    // 1. Warm up history (t = -2, -1)
    // Pitch: 88 -> 89
    std::vector<double> pitch_sequence = {88.0, 89.0};

    for (double p : pitch_sequence) {
        auto q = from_euler_zyx(steady_roll, p, steady_yaw);
        Quaternion<double> quat(q, order);

        auto result = quat.to_eul(history);
        history.push_back(result);
    }

    // 2. The Singularity (t = 0)
    // Pitch = 90.0.
    // At this point, Roll and Yaw are aliased (Gimbal Lock).
    // Standard solvers usually set Yaw=0, which forces Roll = (ExpectedRoll + ExpectedYaw) = 55.
    // Our solver should use history to keep Roll ~ 45 and Yaw ~ 10.
    {
        auto q_singular = from_euler_zyx(steady_roll, 90.0, steady_yaw);
        Quaternion<double> quat(q_singular, order);

        // ACT
        auto result = quat.to_eul(history);

        // ASSERT
        // For ZYX order: [0]=Yaw(Z), [1]=Pitch(Y), [2]=Roll(X)
        double yaw_out   = rad2deg(result.angles[0]);
        double pitch_out = rad2deg(result.angles[1]);
        double roll_out  = rad2deg(result.angles[2]);

        // Pitch should be exactly 90
        EXPECT_NEAR(pitch_out, 90.0, 0.01);

        // Continuity Check:
        // A stateless solver would likely return Roll ~ 55.0 and Yaw = 0.0.
        // We expect Roll ~ 45.0 and Yaw ~ 10.0.
        EXPECT_NEAR(roll_out, steady_roll, 0.5)
            << "Continuity Failure! Roll snapped to " << roll_out << " (Expected ~" << steady_roll << ")";

        EXPECT_NEAR(yaw_out, steady_yaw, 0.5)
            << "Continuity Failure! Yaw snapped to " << yaw_out << " (Expected ~" << steady_yaw << ")";
    }
}

TEST(ContinuityTest, HandlesNoise_LagrangeStability) {
    // SCENARIO: Noisy trajectory passing through singularity.
    // We check if the Lagrange extrapolator is stable enough.

    std::vector<EulerAngles<double>> history;
    // We use XYZ order here for variety.
    // XYZ Singularity is at Pitch(Y) = 90.
    RotationOrder order = RotationOrder::XYZ;
    // XYZ Output: [0]=Roll(X), [1]=Pitch(Y), [2]=Yaw(Z)

    // History: Pitch approaching 90, Roll stable at 10.
    // Frame 1
    EulerAngles<double> f1;
    f1.angles = Eigen::Vector3d(deg2rad(10.1), deg2rad(88.1), 0);
    history.push_back(f1);

    // Frame 2
    EulerAngles<double> f2;
    f2.angles = Eigen::Vector3d(deg2rad(9.9), deg2rad(88.9), 0);
    history.push_back(f2);

    // Frame 3 (Singularity)
    // Pitch = 90.0
    using AngleAxis = Eigen::AngleAxisd;
    using Vector3 = Eigen::Vector3d;
    Eigen::Quaterniond q = AngleAxis(deg2rad(10.0), Vector3::UnitX()) // Roll first
                         * AngleAxis(deg2rad(90.0), Vector3::UnitY()) // Then Pitch
                         * AngleAxis(deg2rad(0.0),  Vector3::UnitZ());// Then Yaw

    Quaternion<double> quat(q, order);

    auto result = quat.to_eul(history);

    double roll_out = rad2deg(result.angles[0]); // Index 0 is X (Roll) for XYZ order

    EXPECT_NEAR(roll_out, 10.0, 0.5)
        << "Noise Stability Failure! Roll was " << roll_out;
}

TEST(StatefulConverterTest, ManagesHistoryAndContinuity) {
    // SCENARIO: Verify StatefulConverter produces smooth output through a singularity
    // without manual history management.

    // Setup: ZYX Order, steady Yaw=10, Roll=45, Pitch ramping 88 -> 90 -> 92
    StatefulConverter<double> converter(RotationOrder::ZYX);
    double steady_roll = 45.0;
    double steady_yaw = 10.0;

    // 1. Frame 1: Pitch 88
    auto q1 = from_euler_zyx(steady_roll, 88.0, steady_yaw);
    auto eul1 = converter.update(q1);
    EXPECT_NEAR(rad2deg(eul1.angles[1]), 88.0, 0.01);

    // 2. Frame 2: Pitch 89
    auto q2 = from_euler_zyx(steady_roll, 89.0, steady_yaw);
    auto eul2 = converter.update(q2);
    EXPECT_NEAR(rad2deg(eul2.angles[1]), 89.0, 0.01);

    // 3. Frame 3: Singularity (Pitch 90)
    auto q3 = from_euler_zyx(steady_roll, 90.0, steady_yaw);
    auto eul3 = converter.update(q3);

    // Verify Continuity
    double pitch_out = rad2deg(eul3.angles[1]);
    double roll_out  = rad2deg(eul3.angles[2]); // ZYX: [0]=Yaw, [1]=Pitch, [2]=Roll
    double yaw_out   = rad2deg(eul3.angles[0]);

    EXPECT_NEAR(pitch_out, 90.0, 0.01);
    EXPECT_NEAR(roll_out, steady_roll, 0.5) << "Stateful Converter failed continuity on Roll";
    EXPECT_NEAR(yaw_out, steady_yaw, 0.5) << "Stateful Converter failed continuity on Yaw";

    // 4. Reset Logic Check
    converter.reset();
    // After reset, history is empty. A singularity input might default to 0 if no extrapolation possible.
    // Ideally we re-prime it, but this just checks the API exists.
}