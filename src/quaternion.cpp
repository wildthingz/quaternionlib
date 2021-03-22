#include <cmath>
#include <iostream>

#include "quaternion.h"
#include "utilities.h"

namespace QuatLib
{
    rotation_axis get_rotation_vector(rotation_order &rot_ord)
    {
        rotation_axis output;

        switch (rot_ord)
        {
        case eXYX:
            output.v1 << 1, 0, 0;
            output.v2 << 0, 1, 0;
            output.v3 << 1, 0, 0;
            break;
        case eYZY:
            output.v1 << 0, 1, 0;
            output.v2 << 0, 0, 1;
            output.v3 << 0, 1, 0;
            break;
        case eZXZ:
            output.v1 << 0, 0, 1;
            output.v2 << 1, 0, 0;
            output.v3 << 0, 0, 1;
            break;
        case eXZX:
            output.v1 << 1, 0, 0;
            output.v2 << 0, 0, 1;
            output.v3 << 1, 0, 0;
            break;
        case eYXY:
            output.v1 << 0, 1, 0;
            output.v2 << 1, 0, 0;
            output.v3 << 0, 1, 0;
            break;
        case eZYZ:
            output.v1 << 0, 0, 1;
            output.v2 << 0, 1, 0;
            output.v3 << 0, 0, 1;
            break;
        case eXYZ:
            output.v1 << 1, 0, 0;
            output.v2 << 0, 1, 0;
            output.v3 << 0, 0, 1;
            break;
        case eYZX:
            output.v1 << 0, 1, 0;
            output.v2 << 0, 0, 1;
            output.v3 << 1, 0, 0;
            break;
        case eZXY:
            output.v1 << 0, 0, 1;
            output.v2 << 1, 0, 0;
            output.v3 << 0, 1, 0;
            break;
        case eXZY:
            output.v1 << 1, 0, 0;
            output.v2 << 0, 0, 1;
            output.v3 << 0, 1, 0;
            break;
        case eYXZ:
            output.v1 << 0, 1, 0;
            output.v2 << 1, 0, 0;
            output.v3 << 0, 0, 1;
            break;
        case eZYX:
            output.v1 << 0, 0, 1;
            output.v2 << 0, 1, 0;
            output.v3 << 1, 0, 0;
            break;
        default:
            output.v1 << 0, 0, 0;
            output.v2 << 0, 0, 0;
            output.v3 << 0, 0, 0;
            break;
        }

        return (output);
    }

    euler_angles Quaternion::toEul(euler_angles &prev_eul)
    {
        euler_angles output;
        output.angles << 0, 0, 0;
        Eigen::Vector3d v3rot = this->quat * this->rot_vecs.v3;
        I_n i_1(this->rot_vecs.v1);
        I_n i_2(this->rot_vecs.v2);
        I_n i_3(this->rot_vecs.v3);

        if ((this->rot_ord == eXYX) || (this->rot_ord == eYZY) || (this->rot_ord == eYZY))
        {
            output.angles[0] = atan2(v3rot[i_1 + 1], -v3rot[i_1 + 2]);
            output.angles[1] = acos(v3rot[i_1.i]);
        }
        else if ((this->rot_ord == eXZX) || (this->rot_ord == eYXY) || (this->rot_ord == eZYZ))
        {
            output.angles[0] = atan2(v3rot[i_1 + 2], v3rot[i_1 + 1]);
            output.angles[1] = acos(v3rot[i_1.i]);
        }
        else if ((this->rot_ord == eXYZ) || (this->rot_ord == eYZX) || (this->rot_ord == eZXY))
        {
            output.angles[0] = atan2(-v3rot[i_1 + 1], v3rot[i_1 + 2]);
            output.angles[1] = asin(v3rot[i_1.i]);
        }
        else
        {
            output.angles[0] = atan2(v3rot[i_1 + 2], v3rot[i_1 + 1]);
            output.angles[1] = -asin(v3rot[i_1.i]);
        }

        // output.angles[0] = M_PI / 6.;

        Eigen::Quaterniond q1(cos(output.angles[0] / 2.), 0.0, 0.0, 0.0);
        Eigen::Vector3d vec = sin(output.angles[0] / 2.) * this->rot_vecs.v1;
        q1.vec() = vec;
        Eigen::Quaterniond q2(cos(output.angles[1] / 2.), 0.0, 0.0, 0.0);
        vec = sin(output.angles[1] / 2.) * this->rot_vecs.v2;
        q2.vec() = vec;
        Eigen::Quaterniond q12 = q1 * q2;

        Eigen::Vector3d v3rot_n(0., 0., 0.);
        v3rot_n(i_3 + 1) = 1.;

        Eigen::Vector3d v3rot_n12 = q12 * v3rot_n;
        Eigen::Vector3d v3rot_nG = this->quat * v3rot_n;

        Eigen::Vector3d v_c = v3rot_n12.cross(v3rot_nG);
        // std::cout << copysign(1, v_c.adjoint() * v3rot) << std::endl;
        output.angles[2] = copysign(1, v_c.adjoint() * v3rot) * acos(v3rot_n12.adjoint() * v3rot_nG);

        return(output);
    }

    Quaternion::Quaternion(Eigen::Quaterniond &q, rotation_order &rot_ord)
    {
        q.normalize();
        this->quat = q;
        this->rot_ord = rot_ord;
        this->rot_vecs = get_rotation_vector(rot_ord);
    }

    Quaternion::Quaternion(rotation_matrix &prev_rotation_matrix, rotation_order &rot_ord)
    {
        this->rot_ord = rot_ord;
        this->rot_vecs = get_rotation_vector(rot_ord);
        // TODO: convert `rotation_matrix` to Quaternion
    }

    Quaternion::Quaternion(euler_angles &eul, rotation_order &rot_ord)
    {
        this->rot_ord = rot_ord;
        this->rot_vecs = get_rotation_vector(rot_ord);
        // TODO: convert `rotation_matrix` to Quaternion
    }
}