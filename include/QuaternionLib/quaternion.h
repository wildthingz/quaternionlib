#ifndef QUATLIB_QUATERNION_H
#define QUATLIB_QUATERNION_H

#include <vector>
#include <array>
#include <concepts>
#include <span>       // C++20: Zero-copy views
#include <cmath>
#include <numbers>    // C++20: Standard constants
#include <type_traits> // Required for is_same_v
#include <Eigen/Dense>

namespace QuatLib {

    // --- Concepts (Type Safety) ---
    template<typename T>
    concept Scalar = std::floating_point<T>;

    // --- Constants ---
    template<Scalar T>
    constexpr T PI = std::numbers::pi_v<T>;

    // Robust epsilon/tolerance handling
    template<Scalar T>
    struct Constants {
        static constexpr T GimbalLockTolerance = std::is_same_v<T, float> ? static_cast<T>(1e-4) : static_cast<T>(1e-6);
    };

    // --- Enums ---
    enum class RotationOrder {
        XYX, YZY, ZXZ, XZX, YXY, ZYZ, // Proper Euler
        XYZ, YZX, ZXY, XZY, YXZ, ZYX  // Tait-Bryan
    };

    // --- Data Structures ---
    template<Scalar T>
    struct EulerAngles {
        Eigen::Matrix<T, 3, 1> angles; // [0]=Roll/Alpha, [1]=Pitch/Beta, [2]=Yaw/Gamma
    };

    template<Scalar T>
    struct RotationMatrix {
        Eigen::Matrix<T, 3, 3> matrix;
    };

    // --- The Main Class ---
    template<Scalar T>
    class Quaternion {
    public:
        using Vec3 = Eigen::Matrix<T, 3, 1>;
        using Quat = Eigen::Quaternion<T>;

        Quaternion(const Quat& q, RotationOrder order)
            : m_quat(q.normalized()), m_order(order) {}

        /**
         * @brief Continuity-Aware Conversion
         * Uses history to extrapolate and select the best solution at singularities.
         * * @param history A view of previous Euler Angles.
         * @param dts (Optional) A view of previous time steps (seconds).
         * @param current_dt (Optional) The current frame's time step (seconds).
         * @return Continuity-corrected Euler Angles
         */
        EulerAngles<T> to_eul(std::span<const EulerAngles<T>> history,
                              std::span<const T> dts = {},
                              T current_dt = T(0)) const;

        RotationMatrix<T> to_rotation_matrix() const;

    private:
        Quat m_quat;
        RotationOrder m_order;
    };

} // namespace QuatLib

#include "details/quaternion_impl.h"

#endif // QUATLIB_QUATERNION_H