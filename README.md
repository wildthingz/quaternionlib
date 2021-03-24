# Qauternion Conversion

## Introduction

Two things need to be considered in converting quaternions to euler angles;

- Order of rotation
- and singularities

In this code base I tackle both of these problems to produce a robust and complete tool to do the conversion.

### Order of Rotation

There are a total of 12 euler angle rotation sequences. I defined them as an enum in `quaternions.h`.

```cpp
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
```

These show the order at which the rotations are done. The "repeated" orders are called proper euler angles and the "non-repeated" orders are called Tait-Bryan angles by convention.

### Singularities (Gimbal Lock Problem)

The gimbal lock is reached when the middle rotation is at 0° for proper euler angles and ±90° for Tait-Bryan angles. This essentially causes the first order and second order rotations overlap so there would be no way of telling which one caused the actual rotation.

Note that this problem only arises when one tries to convert quaternion to euler angles. Rotation matrices and quaternions are immune to this.

There is no way of solving this if there is only one data point. However, if we have a constant stream of data, which is the case in most of the robotics and animation applications, we can find a way!

## Implementation

I did a thorough review of different algorithms devised to carry out the conversion between quaternions and euler angles and rotation matrices. I came to the conclusion that the method proposed by _Hughes_ is the most elegant. However, this method does not deal with the singularities (gimbal lock problem). Also the paper is poorly written and there are bunch of mistakes in the maths!

The main portion of the code revolves around the class `QuatLib::Quaternion` which is essentially a wrapper for `Eigen::Quaterniond`. I overload the constructor so it can be initialized using either a `Eigen::Quaterniond`, a `QuatLib::euler_angles` or a `QuatLib::rotation_matrix`. I have to two methods `to_eul` and `to_rotation_matrix`.

After implementing and successfully testing the _Hughes_ method, I tackled the gimbal lock problem by introducing `prev_euls`, a vector of previous euler angles assuming constant time-step. This can be any number of previous data points. I then extrapolate the next data-point based on these if a singularity is detected.

I'm using two methods for extrapolating the next data-point. An average derivative based extrapolation implemented as `derivative_extrapolation` and Lagrange extrapolation shown in `lagrange_extrapolation` function. A drawback of Lagrange interpolation is that it might over-fit the data so I weigh it down using the average derivative based extrapolation. I essentially calculate the extrapolated data point using both methods and take the average. 

If someone is interested in making this more accurate, I have a placeholder for a more complex method. This could for instance be a sophisticated polynomial regression algorithm, refer [here](https://github.com/Kolkir/mlcpp/tree/master/polynomial_regression_eigen).

## Running the Code

I've setup the project so that it generates a static library you can easily use. You just need Eigen installed.

I have a bash script that compiles the code and runs the test automatically. If you want to run tests make sure to change `BUILD_TYPE` to "debug" in the script.

```bash
# to compile and run:                       ./run.sh -c
# to compile only:                          ./run.sh -co
# continue to run without compiling:        ./run.sh
```

Please refer to `test.cpp` in tests folder to see how to use the library.  

## Resources

### Code

- [Ken Shoemake's algorithm](https://github.com/erich666/GraphicsGems/tree/master/gemsiv/euler_angle)
- [coderlirui's algorithm](https://github.com/coderlirui/quat2eul)
- [J. Fuller's algorithm](https://github.com/drew-gross/MATLAB/blob/master/AutonomousHw1/ME597-3-SpinCalc/SpinCalc.m)

### Papers and articles

- [Slabaugh](https://www.researchgate.net/profile/Thomas-Blesgen/publication/276852668_On_rotation_deformation_zones_for_finite-strain_Cosserat_plasticity/links/55fbe44808ae07629e07c60c/On-rotation-deformation-zones-for-finite-strain-Cosserat-plasticity.pdf)
- [Kuipers](https://www.emis.de/proceedings/Varna/vol1/GEOM09.pdf)
- [Ben-Ari](https://www.weizmann.ac.il/sci-tea/benari/sites/sci-tea.benari/files/uploads/softwareAndLearningMaterials/quaternion-tutorial-2-0-1.pdf)
- [Hughes](https://www.researchgate.net/publication/318276971_Quaternion_tofrom_Euler_Angle_of_Arbitrary_Rotation_Sequence_Direction_Cosine_Matrix_Conversion_Using_Geometric_Methods)
- [Perumal](https://core.ac.uk/download/pdf/228833209.pdf)
- [euclideanspace](http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm)

### Visulaziation and Converter Tools

- [quaterions.online](https://quaternions.online/)
- [Andre Gaschler](https://www.andre-gaschler.com/rotationconverter/)

## Requirements

- [Eigen](https://gitlab.com/libeigen/eigen)
