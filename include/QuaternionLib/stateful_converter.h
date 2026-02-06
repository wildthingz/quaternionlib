//
// Created by hatef on 2026-02-03.
//

#ifndef QUATERNIONLIB_STATEFUL_CONVERTER_H
#define QUATERNIONLIB_STATEFUL_CONVERTER_H

#include <vector>
#include <algorithm>
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
         */
        explicit StatefulConverter(RotationOrder order, size_t history_size = 5)
            : m_order(order), m_max_history(history_size) {
            m_history_buffer.reserve(m_max_history);
            m_dt_buffer.reserve(m_max_history);
        }

        /**
         * @brief Converts a quaternion to Euler angles.
         * @param q The input orientation.
         * @param dt The time (seconds) elapsed since the last update. Defaults to 1.0 (Unit/Frame).
         * If providing real time, ensure units are consistent.
         * @return Continuous Euler Angles.
         */
        Angles update(const Quat& q, T dt = T(1.0)) {
            Quaternion<T> stateless_quat(q, m_order);

            // Pass both Angle history and Time history to the solver
            Angles result = stateless_quat.to_eul(m_history_buffer, m_dt_buffer, dt);

            update_history(result, dt);

            return result;
        }

        void reset() {
            m_history_buffer.clear();
            m_dt_buffer.clear();
        }

        void set_order(RotationOrder new_order) {
            if (m_order != new_order) {
                m_order = new_order;
                reset();
            }
        }

    private:
        RotationOrder m_order;
        size_t m_max_history;

        // Parallel buffers: m_history_buffer[i] corresponds to m_dt_buffer[i]
        std::vector<Angles> m_history_buffer;
        std::vector<T> m_dt_buffer;

        void update_history(const Angles& new_angles, T new_dt) {
            if (m_history_buffer.size() >= m_max_history) {
                // Sliding Window via Rotation (maintains contiguity for std::span)
                // We rotate both buffers to keep them synchronized
                std::rotate(m_history_buffer.begin(), m_history_buffer.begin() + 1, m_history_buffer.end());
                std::rotate(m_dt_buffer.begin(), m_dt_buffer.begin() + 1, m_dt_buffer.end());

                m_history_buffer.back() = new_angles;
                m_dt_buffer.back() = new_dt;
            } else {
                m_history_buffer.push_back(new_angles);
                m_dt_buffer.push_back(new_dt);
            }
        }
    };

}

#endif //QUATERNIONLIB_STATEFUL_CONVERTER_H