#include "utilities.h"

void print_quat(Eigen::Quaterniond &q)
{
    printf("W: %.3f, X: %.3f, Y: %.3f, Z: %.3f\n", q.w(), q.x(), q.y(), q.z());
}

void print_eul(QuatLib::euler_angles &eul)
{
    printf("theta0: %.3f, theta1: %.3f, theta2: %.3f\n", eul.angles[0], eul.angles[1], eul.angles[2]);
}

double deg2rad(double &deg)
{
    return (deg * M_PI / 180.0);
}

double rad2deg(double &rad)
{
    return (rad * 180.0 / M_PI);
}

QuatLib::euler_angles deg2rad(QuatLib::euler_angles &degs)
{
    QuatLib::euler_angles output;
    output.angles[0] = deg2rad(degs.angles[0]);
    output.angles[1] = deg2rad(degs.angles[1]);
    output.angles[2] = deg2rad(degs.angles[2]);
    return(output);
}
QuatLib::euler_angles rad2deg(QuatLib::euler_angles &rads)
{
    QuatLib::euler_angles output;
    output.angles[0] = rad2deg(rads.angles[0]);
    output.angles[1] = rad2deg(rads.angles[1]);
    output.angles[2] = rad2deg(rads.angles[2]);
    return(output);
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

I_n::I_n(Eigen::Vector3d &v)
{
    Eigen::Vector3d vx(1, 0, 0);
    Eigen::Vector3d vy(0, 1, 0);
    Eigen::Vector3d vz(0, 0, 1);

    if ((v != vx) && (v != vy) && (v != vz))
    {
        throw std::invalid_argument("input vector is not a pure rotation axis.");
    }
    if (v[0] == 1)
    {
        this->i = 0;
    }
    else if (v[1] == 1)
    {
        this->i = 1;
    }
    else
    {
        this->i = 2;
    }
}

int I_n::operator+(const int &num) const
{
    return ((num + this->i) % 3);
}
