#ifndef QUATLIB_QUATERNION_IMPL_H
#define QUATLIB_QUATERNION_IMPL_H

#include <cmath>
#include <vector>
#include <numeric>
#include <stdexcept>
#include <algorithm>
#include "../math_utils.h"
#include "QuaternionLib/quaternion.h"

namespace QuatLib {

    namespace details {
        struct OrderMeta {
            int i, j, k;      // Axis indices
            bool is_proper;   // Repeating axes
            bool is_even;     // Parity (Cyclic vs Anticyclic)
        };

        /**
         * @brief Metadata for all 12 rotation orders.
         * Logic: i is the first axis. j is (i+1)%3 if even, else (i+2)%3.
         */
        static constexpr OrderMeta META[] = {
            {0, 1, 2, true,  true},  // XYX
            {1, 2, 0, true,  true},  // YZY
            {2, 0, 1, true,  true},  // ZXZ
            {0, 2, 1, true,  false}, // XZX
            {1, 0, 2, true,  false}, // YXY
            {2, 1, 0, true,  false}, // ZYZ
            {0, 1, 2, false, true},  // XYZ
            {1, 2, 0, false, true},  // YZX
            {2, 0, 1, false, true},  // ZXY
            {0, 2, 1, false, false}, // XZY
            {1, 0, 2, false, false}, // YXZ
            {2, 1, 0, false, false}  // ZYX
        };
    }

    template<Scalar T>
    RotationMatrix<T> Quaternion<T>::to_rotation_matrix() const {
        RotationMatrix<T> result;
        result.matrix = m_quat.toRotationMatrix();
        return result;
    }

    template<Scalar T>
    EulerAngles<T> Quaternion<T>::to_eul(std::span<const EulerAngles<T>> history) const {
        const auto& m = details::META[static_cast<int>(m_order)];
        auto R = m_quat.toRotationMatrix();
        EulerAngles<T> result;

        // Extract indices and parity
        int i = m.i, j = m.j, k = m.k;
        T parity = m.is_even ? 1.0 : -1.0;

        if (!m.is_proper) {
            // --- TAIT-BRYAN GENERAL FORMULA ---
            // Beta is always on the corner: R(i, k). s_beta is signed sin(beta).
            T s_beta = parity * R(i, k);
            T c_beta = std::sqrt(R(i, i) * R(i, i) + R(i, j) * R(i, j));

            // Use strictly typed tolerance
            if (c_beta > Constants<T>::GimbalLockTolerance) {
                // Normal Case: Standard Solution
                result.angles[0] = std::atan2(parity * -R(j, k), R(k, k));
                result.angles[1] = std::asin(std::clamp<T>(s_beta, -1.0, 1.0));
                result.angles[2] = std::atan2(parity * -R(i, j), R(i, i));

                // Continuity: Check Alternative Domain if history exists
                if (!history.empty()) {
                    const auto& prev = history.back();

                    // Construct Alternative: [a+pi, pi-b, c+pi]
                    EulerAngles<T> alt;
                    alt.angles[0] = result.angles[0] + PI<T>;
                    alt.angles[1] = PI<T> - result.angles[1];
                    alt.angles[2] = result.angles[2] + PI<T>;

                    // Unwrap both candidates to handle 360 wraps
                    for (int idx = 0; idx < 3; ++idx) {
                        result.angles[idx] = Utils::unwrap(result.angles[idx], prev.angles[idx]);
                        alt.angles[idx] = Utils::unwrap(alt.angles[idx], prev.angles[idx]);
                    }

                    // Pick closer one
                    if ((alt.angles - prev.angles).squaredNorm() < (result.angles - prev.angles).squaredNorm()) {
                        result = alt;
                    }
                }
            } else {
                // GIMBAL LOCK: Beta is +/- 90 degrees
                T pole_sign = (s_beta > 0) ? 1.0 : -1.0;
                result.angles[1] = pole_sign * (PI<T> / 2);

                // 1. Predict Alpha for continuity using extrapolation
                T a_pred = T(0);
                if (!history.empty()) {
                    a_pred = Utils::extrapolate_axis<T>(history, 0);
                    a_pred = Utils::unwrap(a_pred, history.back().angles[0]);
                }
                result.angles[0] = a_pred;

                // 2. Solve for Gamma (Roll)
                T phi = std::atan2(parity * R(j, i), R(j, j));
                T phi_coeff = (m.is_even && pole_sign < 0) ? -1.0 : 1.0;
                T alpha_coeff = -(parity * pole_sign);

                result.angles[2] = (phi_coeff * phi) + (alpha_coeff * result.angles[0]);

                if (!history.empty()) {
                    result.angles[2] = Utils::unwrap(result.angles[2], history.back().angles[2]);
                }
            }
        } else {
            // --- PROPER EULER GENERAL FORMULA ---
            // Beta is always on the diagonal: R(i, i). c_beta is cos(beta).
            T c_beta = R(i, i);
            T s_beta = std::sqrt(R(j, i) * R(j, i) + R(k, i) * R(k, i));

            // Use strictly typed tolerance
            if (s_beta > Constants<T>::GimbalLockTolerance) {
                // Normal Case: Standard Solution
                result.angles[0] = std::atan2(R(j, i), parity * -R(k, i));
                result.angles[1] = std::acos(std::clamp<T>(c_beta, -1.0, 1.0));
                result.angles[2] = std::atan2(R(i, j), parity * R(i, k));

                // Continuity: Check Alternative Domain if history exists
                if (!history.empty()) {
                    const auto& prev = history.back();

                    // Construct Alternative: [a+pi, -b, c+pi]
                    EulerAngles<T> alt;
                    alt.angles[0] = result.angles[0] + PI<T>;
                    alt.angles[1] = -result.angles[1];
                    alt.angles[2] = result.angles[2] + PI<T>;

                    // Unwrap both candidates
                    for (int idx = 0; idx < 3; ++idx) {
                        result.angles[idx] = Utils::unwrap(result.angles[idx], prev.angles[idx]);
                        alt.angles[idx] = Utils::unwrap(alt.angles[idx], prev.angles[idx]);
                    }

                    // Pick closer one
                    if ((alt.angles - prev.angles).squaredNorm() < (result.angles - prev.angles).squaredNorm()) {
                        result = alt;
                    }
                }
            } else {
                // GIMBAL LOCK: Beta is 0 or 180 degrees
                bool is_zero = (c_beta > 0);
                result.angles[1] = is_zero ? 0.0 : PI<T>;

                // 1. Predict Alpha for continuity using extrapolation
                T a_pred = T(0);
                if (!history.empty()) {
                    a_pred = Utils::extrapolate_axis<T>(history, 0);
                    a_pred = Utils::unwrap(a_pred, history.back().angles[0]);
                }
                result.angles[0] = a_pred;

                // 2. Solve for Gamma
                T phi = is_zero ? std::atan2(parity * -R(j, k), R(j, j))
                                : std::atan2(parity * -R(j, k), -R(j, j));

                result.angles[2] = is_zero ? (phi - result.angles[0])
                                           : (result.angles[0] - phi);

                if (!history.empty()) {
                    result.angles[2] = Utils::unwrap(result.angles[2], history.back().angles[2]);
                }
            }
        }

        return result;
    }
}

#endif
