#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <cmath>
#include <concepts>
#include <numbers>
#include <span>
#include <vector>

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
     * Supports variable time steps if 'dts' and 'current_dt' are provided.
     *
     * @param history The history buffer of EulerAngles
     * @param dts (Optional) History of time deltas corresponding to the history frames.
     * @param current_dt (Optional) The delta time for the frame we are predicting.
     * @param axis_index The index (0, 1, or 2) of the axis to extrapolate
     * @return The predicted angle for the current frame
     */
    template<Float T, typename EulerType>
    T extrapolate_axis(std::span<const EulerType> history,
                       std::span<const T> dts,
                       T current_dt,
                       int axis_index) {

        if (history.empty()) return T(0);
        if (history.size() == 1) return history.back().angles[axis_index];

        // 1. Get previous two points
        T y_last = history.back().angles[axis_index];
        T y_prev = history[history.size() - 2].angles[axis_index];

        // 2. Unwrap to ensure continuous velocity calculation
        T unwrapped_last = unwrap(y_last, y_prev);
        T delta_angle = unwrapped_last - y_prev;

        // 3. Calculate Velocity
        T velocity;
        if (!dts.empty() && dts.size() >= history.size() && current_dt > 0) {
            // Variable Time Step: v = d_theta / dt_last
            // dts buffer structure matches history. dts.back() is the time step that led to history.back()
            T dt_last = dts.back();
            if (dt_last > 1e-6) {
                 velocity = delta_angle / dt_last;
                 // Prediction: y_new = y_last + v * dt_current
                 return y_last + (velocity * current_dt);
            }
        }

        // Fallback: Uniform Time Step Assumption
        // v_per_frame = delta_angle
        velocity = delta_angle;
        return y_last + velocity;
    }
}

#endif