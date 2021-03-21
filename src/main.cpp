#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <chrono>

#include "quat2eul.h"

struct euler_angles
{
    double psi;
    double theta;
    double phi;
};

enum rotation_order
{
    eXYX,
    eYZY,
    eZXZ,
    eXZX,
    eYXY,
    eZYZ,
    eXYZ,
    eYZX,
    eZXY,
    eXZY,
    eYXZ,
    eZYX
};

void singularity_check(double &theta)
{
    if ((M_PI - theta < M_PI / 180 || theta < M_PI / 180))
    {
        printf("singularity check failed: %f || %f < %f rad\n", M_PI - theta, theta, M_PI / 180);
    }
}

void get_euler_angles(Eigen::Quaterniond &q, euler_angles &output, rotation_order &order)
{
    double psi = 0;
    double theta = 0;
    double phi = 0;
    double test;

    q.normalize();

    // Circular repeated axis
    if (order == eXYX)
    {
        psi = atan2((q.x() * q.y() + q.z() * q.w()), (q.y() * q.w() - q.x() * q.z()));
        theta = acos(q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
        phi = atan2((q.x() * q.y() - q.z() * q.w()), (q.x() * q.z() + q.y() * q.w()));
        singularity_check(theta);
    }
    else if (order == eYZY)
    {
        psi = atan2((q.x() * q.w() + q.y() * q.z()), (q.z() * q.w() - q.x() * q.y()));
        theta = acos(q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z());
        phi = atan2((q.y() * q.z() - q.x() * q.w()), (q.x() * q.y() + q.z() * q.w()));
        singularity_check(theta);
    }
    else if (order == eZXZ)
    {
        psi = atan2((q.x() * q.z() + q.y() * q.w()), (q.x() * q.w() - q.y() * q.z()));
        theta = acos(q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
        phi = atan2((q.x() * q.z() - q.y() * q.w()), (q.x() * q.w() + q.y() * q.z()));
        singularity_check(theta);
    }

    // Non-circular repeated axis
    else if (order == eXZX)
    {
        psi = atan2((q.x() * q.z() - q.y() * q.w()), (q.x() * q.y() + q.z() * q.w()));
        theta = acos(q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
        phi = atan2((q.x() * q.z() + q.y() * q.w()), (q.z() * q.w() - q.x() * q.y()));
        singularity_check(theta);
    }
    else if (order == eYXY)
    {
        psi = atan2((q.x() * q.y() - q.z() * q.w()), (q.x() * q.w() + q.y() * q.z()));
        theta = acos(q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z());
        phi = atan2((q.x() * q.y() + q.z() * q.w()), (q.x() * q.w() - q.y() * q.z()));
        singularity_check(theta);
    }
    else if (order == eZYZ)
    {
        psi = atan2((q.y() * q.z() - q.x() * q.w()), (q.x() * q.z() + q.y() * q.w()));
        theta = acos(q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
        phi = atan2((q.x() * q.w() + q.y() * q.z()), (q.y() * q.w() - q.x() * q.z()));
        singularity_check(theta);
    }

    // Circular non-repeated axis
    else if (order == eXYZ)
    {
        test = q.x() * q.z() + q.y() * q.w();
        if (test > 0.499)
        {
            // psi = 2 * atan2(q.x(), q.w());
            theta = M_PI / 2;
            phi = 0;
        }
        else if (test < -0.499)
        {
            // psi = -2 * atan2(q.x(), q.w());
            theta = -M_PI / 2;
            phi = 0;
        }
        else
        {
            psi = atan2(2 * (q.x() * q.w() - q.y() * q.z()),
                        (q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z()));
            theta = asin(2 * (test));
            phi = atan2(2 * (q.z() * q.w() - q.x() * q.y()),
                        (q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z()));
        }
    }
    else if (order == eYZX)
    {
        test = q.x() * q.y() + q.z() * q.w();
        if (test > 0.499)
        {
            psi = 2 * atan2(q.x(), q.w());
            theta = M_PI / 2;
            phi = 0;
        }
        else if (test < -0.499)
        {
            psi = -2 * atan2(q.x(), q.w());
            theta = -M_PI / 2;
            phi = 0;
        }
        else
        {
            psi = atan2(2 * (q.y() * q.w() - q.x() * q.z()),
                        (q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z()));
            theta = asin(2 * (test));
            phi = atan2(2 * (q.x() * q.w() - q.z() * q.y()),
                        (q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z()));
        }
    }
    else if (order == eZXY)
    {
        test = q.x() * q.w() + q.y() * q.z();
        if (test > 0.499)
        {
            // psi = 2 * atan2(q.x(), q.w());
            theta = M_PI / 2;
            phi = 0;
        }
        else if (test < -0.499)
        {
            // psi = -2 * atan2(q.x(), q.w());
            theta = -M_PI / 2;
            phi = 0;
        }
        else
        {
            psi = atan2(2 * (q.z() * q.w() - q.x() * q.y()),
                        (q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z()));
            theta = asin(2 * (test));
            phi = atan2(2 * (q.y() * q.w() - q.z() * q.x()),
                        (q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z()));
        }
    }

    // Non-circular non-repeated axis
    else if (order == eXZY)
    {
        test = q.z() * q.w() - q.x() * q.y();
        if (test > 0.499)
        {
            // psi = 2 * atan2(q.x(), q.w());
            theta = M_PI / 2;
            phi = 0;
        }
        else if (test < -0.499)
        {
            // psi = -2 * atan2(q.x(), q.w());
            theta = -M_PI / 2;
            phi = 0;
        }
        else
        {
            psi = atan2(2 * (q.x() * q.w() + q.y() * q.z()),
                        (q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z()));
            theta = asin(2 * (test));
            phi = atan2(2 * (q.x() * q.z() + q.y() * q.w()),
                        (q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z()));
        }
    }
    else if (order == eYXZ)
    {
        test = q.x() * q.w() - q.y() * q.z();
        if (test > 0.499)
        {
            // psi = 2 * atan2(q.x(), q.w());
            theta = M_PI / 2;
            phi = 0;
        }
        else if (test < -0.499)
        {
            // psi = -2 * atan2(q.x(), q.w());
            theta = -M_PI / 2;
            phi = 0;
        }
        else
        {
            psi = atan2(2 * (q.x() * q.z() + q.y() * q.w()),
                        (q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z()));
            theta = asin(2 * (test));
            phi = atan2(2 * (q.x() * q.y() + q.z() * q.w()),
                        (q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z()));
        }
    }
    else if (order == eZYX)
    {
        test = q.y() * q.w() - q.x() * q.z();
        if (test > 0.499)
        {
            // psi = 2 * atan2(q.x(), q.w());
            theta = M_PI / 2;
            phi = 0;
        }
        else if (test < -0.499)
        {
            // psi = -2 * atan2(q.x(), q.w());
            theta = -M_PI / 2;
            phi = 0;
        }
        else
        {
            psi = atan2(2 * (q.x() * q.y() + q.z() * q.w()),
                        (q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z()));
            theta = asin(2 * (test));
            phi = atan2(2 * (q.x() * q.w() + q.z() * q.y()),
                        (q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z()));
        }
    }
    else
    {
        throw std::invalid_argument("[ ERR] order not supported.");
    }
    output.psi = psi;
    output.theta = theta;
    output.phi = phi;
}

void print_quat(Eigen::Quaterniond &q)
{
    printf("w: %.3f, x: %.3f, y: %.3f, z: %.3f\n", q.w(), q.x(), q.y(), q.z());
}

void print_eul(euler_angles &eul)
{
    printf("phi: %.3f, theta: %.3f, psi: %.3f\n", eul.phi, eul.theta, eul.psi);
}

int main()
{
    // auto t0 = std::chrono::high_resolution_clock::now();
    // euler_angles eul;
    // Eigen::Quaterniond q( 0.641,  0.299,  0.641, -0.299);
    // // Eigen::Quaterniond q( 0.,  0.707,  0.0, 0.707);
    // rotation_order order = eYZX;

    // // std::cout << q.toRotationMatrix() << std::endl;
    // // print_quat(q);
    // get_euler_angles(q, eul, order);
    // auto t1 = std::chrono::high_resolution_clock::now();

    // Quat q1 = {0.299,  0.641, -0.299, 0.641};
    // EulerAngles ea;

    // ea = Eul_FromQuat(q1, EulOrdYZXr);
    // auto t2 = std::chrono::high_resolution_clock::now();

    // auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
    // printf("time for 1st algorithm: %d ns\n", duration);
    // duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
    // printf("time for 2st algorithm: %d ns\n", duration);

    // print_eul(eul);
    // std::cout << ea.x << "\t" << ea.y << "\t" << ea.z << std::endl;

    unsigned int o = 2;
    int a;
    a = o & 1;
    std::cout << a << std::endl;
}