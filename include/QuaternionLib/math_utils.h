#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <cmath>
#include <concepts>
#include <numbers>
#include <span>

namespace QuatLib::Utils {

    // Concepts
    template<typename T>
    concept Float = std::floating_point<T>;

    // Constants
    template<Float T>
    constexpr T PI = std::numbers::pi_v<T>;

    template<Float T>
    constexpr T TWO_PI = 2.0 * PI<T>;

    /**
     * @brief Unwraps 'val' such that it is continuous relative to 'ref'.
     * Resolves the +/- 360 degree (2pi) ambiguity / winding problem.
     * * Example: If ref is 179 and val is -179, the diff is -358.
     * We shift val by +360 to result in 181, preserving the +2 delta.
     * * @param val The new angle (in radians)
     * @param ref The reference angle (previous frame)
     * @return The unwrapped angle
     */
    template<Float T>
    T unwrap(T val, T ref) {
        T diff = val - ref;
        // round(diff / 2pi) gives the closest number of full rotations
        T wraps = std::round(diff / TWO_PI<T>);
        return val - (wraps * TWO_PI<T>);
    }

    /**
     * @brief Extrapolates a single axis from a history of EulerAngles structs.
     * Handles the angular wrapping to ensure velocity is calculated correctly.
     * * @param history The history buffer of EulerAngles
     * @param axis_index The index (0, 1, or 2) of the axis to extrapolate
     * @return The predicted angle for the current frame
     */
    template<Float T, typename EulerType>
    T extrapolate_axis(std::span<const EulerType> history, int axis_index) {
        if (history.empty()) return T(0);
        if (history.size() == 1) return history.back().angles[axis_index];

        // Linear Extrapolation: y_next = y_last + (y_last - y_prev)
        // (Assumes uniform time steps for Phase 1)
        T y_last = history.back().angles[axis_index];
        T y_prev = history[history.size() - 2].angles[axis_index];

        // Calculate diff carefully to respect wrapping (shortest path velocity)
        T unwrapped_last = unwrap(y_last, y_prev);
        T velocity = unwrapped_last - y_prev;

        return y_last + velocity;
    }

    /**
     * @brief Linear extrapolation for non-uniform or uniform time steps (assuming uniform here for Phase 1)
     */
    template<Float T>
    T extrapolate_linear(std::span<const T> history) {
        if (history.size() < 2) return history.back();

        // Simple slope calculation: (y_final - y_initial) / steps
        // Note: This assumes uniform dt. Phase 2 will add dt support.
        T velocity = (history.back() - history.front()) / static_cast<T>(history.size() - 1);
        return history.back() + velocity;
    }

    /**
     * @brief Lagrange polynomial extrapolation.
     * WARNING: High order polynomials oscillate (Runge's phenomenon).
     * Recommend limiting history size to 3-4 points for this method.
     */
    template<Float T>
    T extrapolate_lagrange(std::span<const T> data) {
        size_t n = data.size();
        if (n < 2) return data.back();

        // Target x is n (the next step, assuming indices 0..n-1)
        T x_target = static_cast<T>(n);
        T result = 0;

        for (size_t j = 0; j < n; ++j) {
            T term = data[j];
            T x_j = static_cast<T>(j);

            // Compute Lagrange Basis L_j(x_target)
            for (size_t m = 0; m < n; ++m) {
                if (m != j) {
                    T x_m = static_cast<T>(m);
                    term *= (x_target - x_m) / (x_j - x_m);
                }
            }
            result += term;
        }
        return result;
    }
}

#endif