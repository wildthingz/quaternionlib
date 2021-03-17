#include <iostream>
#include <cmath>
#include <Eigen/Dense>

struct euler_angles
{
    double flexion;
    double abduction;
    double pronation;
};

void get_euler_angles(Eigen::Quaterniond &q, euler_angles &output)
{
    // from @url: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
    q.normalize();
    double test = q.x() * q.y() + q.z() * q.w();
    if (test > 0.499)
    { // singularity at north pole
        output.abduction = 2 * atan2(q.x(), q.w());
        output.flexion = M_PI / 2;
        output.pronation = 0;
        return;
    }
    if (test < -0.499)
    { // singularity at south pole
        output.abduction = -2 * atan2(q.x(), q.w());
        output.flexion = -M_PI / 2;
        output.pronation = 0;
        return;
    }

    output.abduction = atan2(2 * q.y() * q.w() - 2 * q.x() * q.z(), 1 - 2 * pow(q.y(), 2) - 2 * pow(q.z(), 2));
    output.flexion = asin(2 * test);
    output.pronation = atan2(2 * q.x() * q.w() - 2 * q.y() * q.z(), 1 - 2 * pow(q.x(), 2) - 2 * pow(q.z(), 2));
};

void print_quat(Eigen::Quaterniond &q)
{
    printf("w: %.2f, x: %.2f, y: %.2f, z: %.2f\n", q.w(), q.x(), q.y(), q.z());
}

void print_eul(euler_angles &eul)
{
    printf("a: %.2f, f: %.2f, p: %.2f\n", eul.abduction, eul.flexion, eul.pronation);
}

int main()
{
    Eigen::Quaterniond q(-0.934, -0.001, -0.339,  0.113);
    euler_angles eul;
    
    print_quat(q);
    get_euler_angles(q, eul);
    print_eul(eul);
    
}