#include <iostream>
#include <cassert>

#include "quaternion.h"
#include "utilities.h"

void test_read_quat()
{
    printf("[TEST] Quaternion initialization.\n");
    QuatLib::rotation_order order = QuatLib::eZXY;
    Eigen::Quaterniond q(0.723317, 0.360423, 0.439679, 0.391904);

    QuatLib::Quaternion quat(q, order);

    assert(quat == q);
}

void test_eul_to_quat()
{
    printf("[TEST] Euler angles to quaternion conversion.\n");
    QuatLib::rotation_order order = QuatLib::eZXY;
    Eigen::Quaterniond q(0.723317, 0.360423, 0.439679, 0.391904);

    QuatLib::euler_angles eul;
    eul.angles << 30, 60, 45;
    eul = QuatLib::Utils::deg2rad(eul);

    QuatLib::Quaternion quat(eul, order);

    assert(quat == q);
}

void test_quat_to_eul()
{
    printf("[TEST] Quaternion to euler angles conversion.\n");
    QuatLib::rotation_order order = QuatLib::eZXY;
    Eigen::Quaterniond q(0.723317, 0.360423, 0.439679, 0.391904);

    QuatLib::euler_angles eul;
    eul.angles << 30, 60, 45;
    eul = QuatLib::Utils::deg2rad(eul);

    QuatLib::Quaternion quat(q, order);

    QuatLib::euler_angles prev_eul;
    prev_eul.angles << 29, 59, 44;
    prev_eul = QuatLib::Utils::deg2rad(prev_eul);
    std::vector<QuatLib::euler_angles> v;

    QuatLib::euler_angles eul_res = quat.to_eul(v);

    assert((eul.angles - eul_res.angles).norm() < EPS);
}

void test_rotation_matrix_to_quat()
{
    printf("[TEST] Rotation matrix to quaternion conversion.\n");
    QuatLib::rotation_order order = QuatLib::eZXY;
    Eigen::Quaterniond q(0.723317, 0.360423, 0.439679, 0.391904);

    QuatLib::rotation_matrix rot_mat;
    rot_mat.matrix << 0.306186, 0.883884, -0.353553,
        -0.250001, 0.433012, 0.866025,
        0.918558, -0.176776, 0.353554;

    QuatLib::Quaternion quat(rot_mat, order);

    assert(quat == q);
}

void test_quaternion_to_rotation_matrix()
{
    printf("[TEST] Quaternion to rotation matrix conversion.\n");
    QuatLib::rotation_order order = QuatLib::eZXY;
    Eigen::Quaterniond q(0.723317, 0.360423, 0.439679, 0.391904);

    QuatLib::rotation_matrix rot_mat;
    rot_mat.matrix << 0.306186, 0.883884, -0.353553,
        -0.250001, 0.433012, 0.866025,
        0.918558, -0.176776, 0.353554;

    QuatLib::Quaternion quat(q, order);

    QuatLib::rotation_matrix rot_mat_res = quat.to_rotation_matrix();

    assert((rot_mat.matrix - rot_mat_res.matrix).norm() < EPS);
}

void test_quaternion_to_euler_singularity()
{
    printf("[TEST] Quaternion to euler angles conversion with singularity.\n");
    QuatLib::rotation_order order = QuatLib::eZXY;
    Eigen::Quaterniond q(0.6830127, 0.6830127, 0.1830127, 0.1830127);

    std::vector<QuatLib::euler_angles> v;
    QuatLib::euler_angles prev_eul;
    prev_eul.angles << 22, 89, 7;
    prev_eul = QuatLib::Utils::deg2rad(prev_eul);
    v.push_back(prev_eul);
    prev_eul.angles << 21, 89, 7.5;
    prev_eul = QuatLib::Utils::deg2rad(prev_eul);
    v.push_back(prev_eul);
    prev_eul.angles << 20.7, 89, 7.8;
    prev_eul = QuatLib::Utils::deg2rad(prev_eul);
    v.push_back(prev_eul);

    QuatLib::euler_angles eul;
    eul.angles << 20, 90, 10;
    eul = QuatLib::Utils::deg2rad(eul);

    QuatLib::Quaternion quat(q, order);

    QuatLib::euler_angles eul_res = quat.to_eul(v);

    assert((eul.angles - eul_res.angles).norm() < 0.5);
}

int main()
{
#ifndef NDEBUG
    test_read_quat();
    test_eul_to_quat();
    test_quat_to_eul();
    test_rotation_matrix_to_quat();
    test_quaternion_to_rotation_matrix();
    test_quaternion_to_euler_singularity();

    printf("[TEST] Tests successful.\n");
#else
    printf("[100%] Release build finished.\n");
#endif
}