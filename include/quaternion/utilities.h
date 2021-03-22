#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <cmath>

#include "quaternion.h"

void print_quat(Eigen::Quaterniond &q);

void print_eul(QuatLib::euler_angles &eul);

double deg2rad(double &deg);
double rad2deg(double &rad);

QuatLib::euler_angles deg2rad(QuatLib::euler_angles &degs);
QuatLib::euler_angles rad2deg(QuatLib::euler_angles &rads);

template <typename T> int sgn(T val);

class I_n
{
public:
    int i = 0;

    I_n(Eigen::Vector3d &v);

    int operator+(const int &num) const;
};

#endif