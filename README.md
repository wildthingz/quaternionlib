# Qauternion Conversion

## Introduction

Two things need to considered in converting quaternions to euler angles and direct cosine matrix (DCM);
 - Order of rotation
 - and singularities

In this code base I tackle both of these problems to produce a robust and complete tool to do the conversion.

## Visulaziation and Converter Tools

- [quaterions.online](https://quaternions.online/)
- [Andre Gaschler](https://www.andre-gaschler.com/rotationconverter/)

## Resources

- code:
  - [Ken Shoemake's algorithm](https://github.com/erich666/GraphicsGems/tree/master/gemsiv/euler_angle)
  - [coderlirui's algorithm](https://github.com/coderlirui/quat2eul)
  - [J. Fuller's algorithm](https://github.com/drew-gross/MATLAB/blob/master/AutonomousHw1/ME597-3-SpinCalc/SpinCalc.m)

- Papers and articles:
  - [Slabaugh](https://www.researchgate.net/profile/Thomas-Blesgen/publication/276852668_On_rotation_deformation_zones_for_finite-strain_Cosserat_plasticity/links/55fbe44808ae07629e07c60c/On-rotation-deformation-zones-for-finite-strain-Cosserat-plasticity.pdf)
  - [Kuipers](https://www.emis.de/proceedings/Varna/vol1/GEOM09.pdf)
  - [Ben-Ari](https://www.weizmann.ac.il/sci-tea/benari/sites/sci-tea.benari/files/uploads/softwareAndLearningMaterials/quaternion-tutorial-2-0-1.pdf)
  - [Hughes](https://www.researchgate.net/publication/318276971_Quaternion_tofrom_Euler_Angle_of_Arbitrary_Rotation_Sequence_Direction_Cosine_Matrix_Conversion_Using_Geometric_Methods)
  - [Perumal](https://core.ac.uk/download/pdf/228833209.pdf)
  - [euclideanspace](http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm)

## Requirements

- [Eigen](https://gitlab.com/libeigen/eigen)
