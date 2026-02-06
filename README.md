# QuaternionLib: Continuity-Aware Euler Conversion

![Gimbal Lock Demo](media/output.gif)
> **Visual Comparison:** The Red plane (Standard Library) violently snaps when Pitch hits 90°, losing orientation. The Cyan plane (QuaternionLib) maintains smooth, continuous motion using stateful history extrapolation.

## 1. The Theory & The Problem

### The Topology Mismatch (The "Hairy Ball" Theorem)
Rotations exist on a hypersphere ($S^3$), while Euler angles exist on a flat coordinate system ($\mathbb{R}^3$). Topologically, it is impossible to map the smooth surface of a sphere onto a flat grid without at least one point where the map "tears" or becomes singular.

In Euler angles, this tear is known as **Gimbal Lock**. When the middle rotation aligns the two outer axes (e.g., Pitch = $\pm 90^\circ$ in a Tait-Bryan sequence), one degree of freedom is lost. Mathematically, this means there are **infinite valid solutions** for the remaining angles.
* *Example:* At the North Pole, `[Yaw: 10, Pitch: 90, Roll: 0]` describes the exact same physical orientation as `[Yaw: 20, Pitch: 90, Roll: -10]`.

### The "Glitch" in Time-Series Data
Standard math libraries (like Eigen or GLM) are **stateless**. When they encounter this singularity, they must make an arbitrary choice (e.g., "Always force Roll to 0").

In a dynamic system—like a robotic arm moving through a singularity or a camera panning straight down—this arbitrary choice creates a massive **Temporal Discontinuity**.

**The Artifact:**
* Frame $t-1$: `[Yaw: 45°, Pitch: 89°, Roll: 45°]`
* Frame $t$ (Singularity): `[Yaw: 90°, Pitch: 90°, Roll: 0°]` *(Library forces Roll to 0)*
* Frame $t+1$: `[Yaw: 46°, Pitch: 89°, Roll: 46°]`

To a PID controller or a physics engine, this looks like the object violently snapped its wrist ($0 \rightarrow 45$) in a single microsecond. This results in **Control Spikes** in motors and **Visual Jitter** in animation.

## 2. The Solution

**QuaternionLib** treats Euler conversion as a **Time-Series Problem**, not just a geometric one.

Instead of calculating the conversion in isolation, it maintains a history buffer of recent states. When the system approaches a singularity, it uses this history to enforce continuity:

1.  **Extrapolation:** The library uses **Lagrange Polynomials** and **Derivative Estimation** to predict where the angles *should* be based on the trajectory of the last $N$ frames.
2.  **Selection:** It solves the Gimbal Lock equation to find the infinite set of valid angles, then selects the specific solution that minimizes the error against the extrapolated prediction.

This acts as a **Continuity Filter**, allowing physical systems to pass smoothly through the North/South poles without numerical glitches.

## 3. Supported Rotation Sequences

There are a total of 12 Euler angle rotation sequences supported by this library. They are defined as an enum in `quaternion.h`:

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

These determine the order in which rotations are applied. The "repeated" orders are commonly referred to as Proper Euler Angles, while the "non-repeated" orders are referred to as Tait-Bryan angles.

## 4. Implementation Details

I conducted a thorough review of algorithms for converting between quaternions, Euler angles, and rotation matrices. I concluded that the method proposed by Hughes is the most elegant for general geometric conversion. However, the original method does not account for the singularity (Gimbal Lock) continuity problem, and the paper contained several mathematical errors which have been corrected here.

The core of the library is the QuatLib::Quaternion class, which serves as a wrapper around Eigen::Quaterniond. It overloads constructors to allow initialization from:

- ```Eigen::Quaterniond```
- ```QuatLib::euler_angles```
- ```QuatLib::rotation_matrix```

### Handling Singularities
After implementing the base Hughes method, I addressed the continuity problem by introducing prev_euls, a history vector of previous Euler angles (assuming a constant time-step).

When a singularity is detected:

1. Derivative Extrapolation: Uses the average derivative of previous points to project the next likely angle.
2. Lagrange Extrapolation: Uses a Lagrange polynomial to fit the previous points. 
   - Note: Lagrange interpolation can overfit noisy data, so it is weighted against the derivative method to ensure stability.

Future improvements may include a sophisticated polynomial regression algorithm (Savitzky-Golay filter) for even robust noise rejection.

## 6. Resources

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