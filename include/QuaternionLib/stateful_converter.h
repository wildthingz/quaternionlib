//
// Created by hatef on 2026-02-03.
//

#ifndef QUATERNIONLIB_STATEFUL_CONVERTER_H
#define QUATERNIONLIB_STATEFUL_CONVERTER_H

#include <vector>
#include <algorithm> // Required for std::rotate
#include "quaternion.h"

namespace QuatLib {

    template<Scalar T>
    class StatefulConverter {
    public:
        using Quat = Eigen::Quaternion<T>;
        using Angles = EulerAngles<T>;

        /**
         * @brief Configures the converter.
         * @param order The desired Euler rotation order.
         * @param history_size Number of previous frames to keep for extrapolation (default 5).
         * Higher values smooth out noise but might lag during rapid acceleration changes.
         * Lower values (2-3) are more responsive but sensitive to jitter.
         */
        explicit StatefulConverter(RotationOrder order, size_t history_size = 5)
            : m_order(order), m_max_history(history_size) {
            // Pre-allocate to avoid runtime re-allocations if using vector
            m_history_buffer.reserve(m_max_history);
        }

        /**
         * @brief Converts a quaternion to Euler angles, maintaining continuity with previous frames.
         * @param q The input orientation.
         * @return Continuous Euler Angles.
         */
        Angles update(const Quat& q) {
            // 1. Create the stateless wrapper
            Quaternion<T> stateless_quat(q, m_order);

            // 2. Convert using our internal history
            // We pass the history vector to the stateless solver
            Angles result = stateless_quat.to_eul(m_history_buffer);

            // 3. Update history
            update_history(result);

            return result;
        }

        /**
         * @brief Resets the internal history.
         * Call this if the object teleports or the simulation restarts.
         */
        void reset() {
            m_history_buffer.clear();
        }

        /**
         * @brief Change the rotation order dynamically.
         * Clears history as previous angles are no longer valid in the new basis.
         */
        void set_order(RotationOrder new_order) {
            if (m_order != new_order) {
                m_order = new_order;
                reset();
            }
        }

    private:
        RotationOrder m_order;
        size_t m_max_history;
        std::vector<Angles> m_history_buffer; // Using vector as a contiguous sliding window

        void update_history(const Angles& new_angles) {
            if (m_history_buffer.size() >= m_max_history) {
                // Optimized Sliding Window (Ring Buffer Behavior):
                // We need to keep data contiguous for std::span compatibility in to_eul.
                // Instead of erase(begin) which can be slow/allocating, we use std::rotate.
                // This shifts [1..End] to [0..End-1], leaving the last slot open for overwrite.
                // For small N (history size), this is extremely fast (L1 cache friendly memmove).
                std::rotate(m_history_buffer.begin(), m_history_buffer.begin() + 1, m_history_buffer.end());
                m_history_buffer.back() = new_angles;
            } else {
                m_history_buffer.push_back(new_angles);
            }
        }
    };

} // namespace QuatLib

#endif //QUATERNIONLIB_STATEFUL_CONVERTER_H