#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include <vector>
#include <Eigen/Dense>

// #ifndef _VERBOSE_
// #define _VERBOSE_
// #endif

namespace QuatLib
{
    struct euler_angles
    {
        Eigen::Vector3d angles;
    };

    // direct cosine matrix (rotation matrix)
    struct rotation_matrix
    {
        Eigen::Matrix3d matrix;
    };

    struct rotation_axis
    {
        Eigen::Vector3d v1;
        Eigen::Vector3d v2;
        Eigen::Vector3d v3;
    };

    enum rotation_order
    {
        // circular, repeated
        eXYX,
        eYZY,
        eZXZ,
        // non-circular, repeated
        eXZX,
        eYXY,
        eZYZ,
        // circular, non-repeated
        eXYZ,
        eYZX,
        eZXY,
        // non-circular, non-repeated
        eXZY,
        eYXZ,
        eZYX
    };

    rotation_axis get_rotation_vector(rotation_order &rot_ord);
    euler_angles derivative_extrapolation(std::vector<euler_angles> &euls);
    euler_angles lagrange_extrapolation(std::vector<euler_angles> &euls);
    euler_angles complex_extrapolation(std::vector<euler_angles> &euls);

    class Quaternion
    {
    public:
        euler_angles to_eul(std::vector<euler_angles> &prev_euls);
        rotation_matrix to_rotation_matrix();

        Eigen::Quaterniond quat;
        rotation_order rot_ord;
        rotation_axis rot_vecs;

        Quaternion(euler_angles &eul, rotation_order &rot_ord);
        Quaternion(rotation_matrix &rotation_matrix, rotation_order &rot_ord);
        Quaternion(Eigen::Quaterniond &q, rotation_order &rot_ord);

        bool operator==(const Quaternion &quat) const;
        bool operator==(const Eigen::Quaterniond &quat) const;
    };

} // namespace QuatLib

#endif