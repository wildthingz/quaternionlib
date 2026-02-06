import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation

# Add the build directory to sys.path so we can import the .so module
# Adjust this path if your build folder is named differently
build_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../cmake-build-release'))
sys.path.append(build_dir)

try:
    import quaternionlib as ql
    print(f"Successfully imported quaternionlib from {ql.__file__}")
except ImportError as e:
    print(f"Failed to import quaternionlib: {e}")
    print(f"Search path: {sys.path}")
    sys.exit(1)

def test_basic_conversion():
    print("\n--- Testing Basic Conversion ---")
    # ZYX Order
    converter = ql.Converter(ql.RotationOrder.ZYX)

    # Identity Quaternion (w, x, y, z) -> (1, 0, 0, 0)
    # Pybind11 Eigen mapping usually expects [x, y, z, w] for Quaterniond?
    # Let's verify. Eigen::Quaterniond constructor from vector is usually coeff order.
    # Eigen stores coeffs as (x, y, z, w).
    q_identity = np.array([0.0, 0.0, 0.0, 1.0])

    result = converter.update(q_identity)
    print(f"Identity -> Euler: {result.angles}")

    # Expected: [0, 0, 0]
    if np.allclose(result.angles, [0, 0, 0]):
        print("PASS: Identity conversion correct")
    else:
        print("FAIL: Identity conversion incorrect")

def test_singularity_continuity():
    print("\n--- Testing Singularity Continuity (ZYX North Pole) ---")
    converter = ql.Converter(ql.RotationOrder.ZYX)

    # Simulate a sequence approaching North Pole (Pitch -> 90)
    # Roll = 45, Yaw = 10 (fixed), Pitch ramps 80 -> 90

    rolls = [45.0] * 10
    yaws = [10.0] * 10
    pitches = np.linspace(80, 90, 10)

    print(f"{'Pitch':>10} | {'Calc Roll':>10} | {'Calc Yaw':>10}")
    print("-" * 36)

    for r, p, y in zip(rolls, pitches, yaws):
        # Create quat using scipy (seq='zyx' means intrinsic Z, then Y, then X)
        # Note: scipy uses scalar-last (x, y, z, w) by default which matches Eigen
        rot = Rotation.from_euler('zyx', [y, p, r], degrees=True)
        q = rot.as_quat() # returns [x, y, z, w]

        # Update converter
        euler = converter.update(q)

        # Extract results (radians -> degrees)
        # ZYX output: [0]=Yaw(Z), [1]=Pitch(Y), [2]=Roll(X)
        res_yaw = np.degrees(euler.angles[0])
        res_pitch = np.degrees(euler.angles[1])
        res_roll = np.degrees(euler.angles[2])

        print(f"{p:>10.2f} | {res_roll:>10.2f} | {res_yaw:>10.2f}")

if __name__ == "__main__":
    test_basic_conversion()
    test_singularity_continuity()