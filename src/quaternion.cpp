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

    euler_angles derivative_extrapolation(std::vector<euler_angles> &euls)
    {
        euler_angles output;
        output.angles[0] = euls[euls.size() - 1].angles[0] + (euls[euls.size() - 1].angles[0] - euls[0].angles[0]) / euls.size();
        output.angles[1] = euls[euls.size() - 1].angles[1] + (euls[euls.size() - 1].angles[1] - euls[0].angles[1]) / euls.size();
        output.angles[2] = euls[euls.size() - 1].angles[2] + (euls[euls.size() - 1].angles[2] - euls[0].angles[2]) / euls.size();
        return (output);
    }

    euler_angles lagrange_extrapolation(std::vector<euler_angles> &euls)
    {
        // The lagrange extrapolation implemeneted here is based on fixed timestep, and assumes to predict the next timestep.
        // IMPORTANT: might cause overfitting, use with caution
        euler_angles output;
        double denum;
        output.angles.setZero();

        double kfact = (double)Utils::factorial((unsigned int)(euls.size()));

        for (int i = 0; i < euls.size(); i++)
        {
            denum = -1;
            for (int k = 0; k <= euls.size(); k++)
            {
                if (k != i)
                {
                    denum *= (i - k);
                }
            }
            output.angles[0] += euls[i].angles[0] * kfact / denum;
            output.angles[1] += euls[i].angles[1] * kfact / denum;
            output.angles[2] += euls[i].angles[2] * kfact / denum;
        }

        return (output);
    }

    euler_angles complex_extrapolation(std::vector<euler_angles> &euls)
    {

        euler_angles output;
        //TODO: Use polynomial regression
        return (output);
    }

    euler_angles Quaternion::to_eul(std::vector<euler_angles> &prev_euls)
    {
        euler_angles output;
        // euler_angles prev_deriv = get_eul_derivative(prev_euls);
        output.angles << 0, 0, 0;
        Eigen::Vector3d v3rot = this->quat * this->rot_vecs.v3;
        Utils::I_n i_1(this->rot_vecs.v1);
        Utils::I_n i_2(this->rot_vecs.v2);
        Utils::I_n i_3(this->rot_vecs.v3);

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

        output.angles[2] = copysign(1, v_c.adjoint() * v3rot) * acos(v3rot_n12.adjoint() * v3rot_nG);

        if ((this->rot_ord == eXYX) || (this->rot_ord == eYZY) || (this->rot_ord == eYZY) || (this->rot_ord == eXZX) || (this->rot_ord == eYXY) || (this->rot_ord == eZYZ))
        {
            if (abs(output.angles[1]) < EPS)
            {
#ifdef _VERBOSE_
                printf("[WARN ]: Singularity reached.\n");
#endif
                output.angles[1] = M_PI / 2;
                double theta02_sum = output.angles[0] + output.angles[2];
                euler_angles extrapolated = derivative_extrapolation(prev_euls);
                euler_angles extrapolatel = lagrange_extrapolation(prev_euls);
                output.angles[0] = (extrapolated.angles[0] + extrapolatel.angles[0]) / 2;
                output.angles[2] = theta02_sum - output.angles[0];
            }
        }
        else
        {
            if (abs(output.angles[1] - (M_PI / 2)) < EPS)
            {
#ifdef _VERBOSE_
                printf("[WARN ]: Singularity reached at north pole.\n");
#endif
                output.angles[1] = M_PI / 2;
                double theta02_sum = output.angles[0] + output.angles[2];
                euler_angles extrapolated = derivative_extrapolation(prev_euls);
                euler_angles extrapolatel = lagrange_extrapolation(prev_euls);
                output.angles[0] = (extrapolated.angles[0] + extrapolatel.angles[0]) / 2;
                output.angles[2] = theta02_sum - output.angles[0];
            }
            else if (abs(output.angles[1] + (M_PI / 2)) < EPS)
            {
#ifdef _VERBOSE_
                printf("[WARN ]: Singularity reached at south pole.\n");
#endif
                output.angles[1] = -M_PI / 2;
                double theta02_diff = output.angles[0] - output.angles[2];
                euler_angles extrapolated = derivative_extrapolation(prev_euls);
                euler_angles extrapolatel = lagrange_extrapolation(prev_euls);
                output.angles[0] = (extrapolated.angles[0] + extrapolatel.angles[0]) / 2;
                output.angles[2] = output.angles[0] - theta02_diff;
            }
        }

        return (output);
    }

    rotation_matrix Quaternion::to_rotation_matrix()
    {
        rotation_matrix output;

        rotation_axis xyz;
        xyz.v1 << 1, 0, 0;
        xyz.v2 << 0, 1, 0;
        xyz.v3 << 0, 0, 1;

        output.matrix.row(0) = this->quat * xyz.v1;
        output.matrix.row(1) = this->quat * xyz.v2;
        output.matrix.row(2) = this->quat * xyz.v3;

        return (output);
    }

    Quaternion::Quaternion(Eigen::Quaterniond &q, rotation_order &rot_ord)
    {
        q.normalize();
        this->quat = q;
        this->rot_ord = rot_ord;
        this->rot_vecs = get_rotation_vector(rot_ord);
    }

    Quaternion::Quaternion(rotation_matrix &rotation_matrix, rotation_order &rot_ord)
    {
        this->rot_ord = rot_ord;
        this->rot_vecs = get_rotation_vector(rot_ord);

        rotation_axis xyz;
        xyz.v1 << 1, 0, 0;
        xyz.v2 << 0, 1, 0;
        xyz.v3 << 0, 0, 1;

        Eigen::Vector3d tmp;
        tmp << rotation_matrix.matrix(0, 0), rotation_matrix.matrix(0, 1), rotation_matrix.matrix(0, 2);
        Eigen::Vector3d v = xyz.v1.cross(tmp);
        v.normalize();

        double phi = acos(xyz.v1.adjoint() * tmp);
        Eigen::Quaterniond qi1(cos(phi / 2), v[0] * sin(phi / 2), v[1] * sin(phi / 2), v[2] * sin(phi / 2));

        v = qi1 * xyz.v2;
        tmp << rotation_matrix.matrix(1, 0), rotation_matrix.matrix(1, 1), rotation_matrix.matrix(1, 2);
        phi = acos(v.adjoint() * tmp);

        Eigen::Quaterniond qi2(cos(phi / 2), xyz.v1[0] * sin(phi / 2), xyz.v1[1] * sin(phi / 2), xyz.v1[2] * sin(phi / 2));

        this->quat = qi1 * qi2;
    }

    Quaternion::Quaternion(euler_angles &eul, rotation_order &rot_ord)
    {
        this->rot_ord = rot_ord;
        this->rot_vecs = get_rotation_vector(rot_ord);

        Eigen::Quaterniond q1(cos(eul.angles[0] / 2.), 0.0, 0.0, 0.0);
        Eigen::Vector3d vec = sin(eul.angles[0] / 2.) * this->rot_vecs.v1;
        q1.vec() = vec;

        Eigen::Quaterniond q2(cos(eul.angles[1] / 2.), 0.0, 0.0, 0.0);
        vec = sin(eul.angles[1] / 2.) * this->rot_vecs.v2;
        q2.vec() = vec;

        Eigen::Quaterniond q3(cos(eul.angles[2] / 2.), 0.0, 0.0, 0.0);
        vec = sin(eul.angles[2] / 2.) * this->rot_vecs.v3;
        q3.vec() = vec;

        this->quat = q1 * q2 * q3;
    }

    bool Quaternion::operator==(const Quaternion &quat) const
    {
        if ((abs(this->quat.x() - quat.quat.x()) < EPS) || (abs(this->quat.y() - quat.quat.y()) < EPS) || (abs(this->quat.z() - quat.quat.z()) < EPS))
        {
            return (true);
        }
        else if ((abs(this->quat.x() + quat.quat.x()) < EPS) || (abs(this->quat.y() + quat.quat.y()) < EPS) || (abs(this->quat.z() + quat.quat.z()) < EPS))
        {
            return (true);
        }
        else
        {
            return (false);
        }
    }

    bool Quaternion::operator==(const Eigen::Quaterniond &quat) const
    {
        if ((abs(this->quat.x() - quat.x()) < EPS) || (abs(this->quat.y() - quat.y()) < EPS) || (abs(this->quat.z() - quat.z()) < EPS))
        {
            return (true);
        }
        else if ((abs(this->quat.x() + quat.x()) < EPS) || (abs(this->quat.y() + quat.y()) < EPS) || (abs(this->quat.z() + quat.z()) < EPS))
        {
            return (true);
        }
        else
        {
            return (false);
        }
    }
}