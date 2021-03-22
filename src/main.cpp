#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <chrono>

#include "quaternion.h"
#include "utilities.h"

int main()
{
    // Eigen::Quaterniond q(0.723317, 0.360423, 0.439679, 0.391904);
    Eigen::Quaterniond q(0.899, 0.0114, 0.419, 0.130);
    // q.normalize();
    // Eigen::Vector3d v(0, 1, 0);

    // auto v2 = q * v;

    // std::cout << v2 << std::endl;

    // I_n i_n(v);

    // std::cout << i_n.i << std::endl;

    // std::cout << i_n + 4 << std::endl;

    // std::cout << i_n + 1 << std::endl;

    // Eigen::Quaterniond q1(1, 2, 3, 4);
    // q.x() = 12;
    // std::cout << q1.x() << std::endl;
    QuatLib::euler_angles prev_eul;
    prev_eul.angles << 0, 0, 0;

    QuatLib::rotation_order order = QuatLib::eYZY;
    QuatLib::Quaternion quat(q, order);

    QuatLib::euler_angles eul = quat.toEul(prev_eul);
    eul = rad2deg(eul);

    print_eul(eul);
}