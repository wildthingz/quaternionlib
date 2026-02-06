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
    // This ensures T is a valid floating point type (float, double, long double)
    template<typename T>
    concept Scalar = std::floating_point<T>;

    // --- Constants ---
    template<Scalar T>
    constexpr T PI = std::numbers::pi_v<T>;

    // Robust epsilon/tolerance handling
    template<Scalar T>
    struct Constants {
        // Tolerance for Gimbal Lock detection.
        // Uses 1e-4 for float (approx 7 significant digits)
        // Uses 1e-6 for double (approx 15 significant digits)
        // This prevents false positives in physics engines using double precision.
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
        // Type Aliases for clean code
        using Vec3 = Eigen::Matrix<T, 3, 1>;
        using Quat = Eigen::Quaternion<T>;

        /**
         * @brief Constructor
         * @param q The raw Eigen quaternion (will be normalized)
         * @param order The desired Euler rotation order
         */
        Quaternion(const Quat& q, RotationOrder order)
            : m_quat(q.normalized()), m_order(order) {}

        /**
         * @brief Continuity-Aware Conversion
         * Uses history to extrapolate and select the best solution at singularities.
         * * @param history A view of previous Euler Angles.
         * Can pass std::vector, std::array, or raw pointer.
         * @return Continuity-corrected Euler Angles
         */
        EulerAngles<T> to_eul(std::span<const EulerAngles<T>> history) const;

        /**
         * @brief Standard Matrix Conversion
         */
        RotationMatrix<T> to_rotation_matrix() const;

    private:
        Quat m_quat;
        RotationOrder m_order;

        // Internal Helpers (Implemented in MathOps.h or below)
        EulerAngles<T> solve_singular(const Vec3& axis_1, const Vec3& axis_3, T theta_1) const;
    };

} // namespace QuatLib

// Since this is a template library, we must include the implementation here
// so the compiler can see it when the user instantiates Quaternion<double>.
#include "details/quaternion_impl.h"

#endif // QUATLIB_QUATERNION_H