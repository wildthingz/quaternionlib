import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation
import pytest

# Ensure we can import the library
build_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../cmake-build-release'))
sys.path.append(build_dir)
import quaternionlib as ql

# Map string to your Enum
# CRITICAL FIX: Use UPPERCASE keys for SciPy to ensure Intrinsic Rotations
# This matches the C++ implementation.
ORDER_MAP = {
    'XYX': ql.RotationOrder.XYX, 'YZY': ql.RotationOrder.YZY, 'ZXZ': ql.RotationOrder.ZXZ,
    'XZX': ql.RotationOrder.XZX, 'YXY': ql.RotationOrder.YXY, 'ZYZ': ql.RotationOrder.ZYZ,
    'XYZ': ql.RotationOrder.XYZ, 'YZX': ql.RotationOrder.YZX, 'ZXY': ql.RotationOrder.ZXY,
    'XZY': ql.RotationOrder.XZY, 'YXZ': ql.RotationOrder.YXZ, 'ZYX': ql.RotationOrder.ZYX
}

def check_orientation_match(order_str, angles_deg):
    """
    1. Generate Quat from Angles (Ground Truth via SciPy Intrinsic)
    2. Convert Quat -> Angles (Candidate via QuaternionLib)
    3. Convert Angles -> Quat (Verification via SciPy Intrinsic)
    4. Compare Quat1 vs Quat3
    """
    # 1. Ground Truth (Intrinsic)
    r_gt = Rotation.from_euler(order_str, angles_deg, degrees=True)
    q_input = r_gt.as_quat() # [x, y, z, w]

    # 2. Candidate Conversion
    converter = ql.Converter(ORDER_MAP[order_str])
    res = converter.update(q_input)
    # Convert radians back to degrees for inspection
    out_deg = np.degrees(res.angles)

    # 3. Reconstruct Rotation from output angles (Intrinsic)
    r_recon = Rotation.from_euler(order_str, res.angles, degrees=False)
    q_recon = r_recon.as_quat()

    # 4. Quaternion Dot Product (Orientation Distance)
    # q and -q are the same rotation, so abs(dot) should be ~1.0
    dot = np.abs(np.dot(q_input, q_recon))

    return dot, out_deg

def test_all_orders_random():
    print(f"{'Order':<5} | {'Status':<6} | {'Error (1-Dot)':<15}")
    print("-" * 35)

    failures = []

    for order_str in ORDER_MAP.keys():
        # Test 10 random orientations per order
        for _ in range(10):
            # Avoid perfect singularities to isolate non-singular math accuracy
            angles = np.random.uniform(-80, 80, size=3)

            dot, out_angles = check_orientation_match(order_str, angles)
            error = 1.0 - dot

            if error > 1e-5:
                failures.append((order_str, angles, out_angles, error))
                print(f"{order_str:<5} | FAIL   | {error:.2e}")
                break
        else:
            print(f"{order_str:<5} | PASS   | {1.0-dot:.2e}")

    if failures:
        print("\n=== FAILURES DETECTED ===")
        for f in failures:
            print(f"Order: {f[0]}")
            print(f"  Input (Deg): {f[1]}")
            print(f"  Output(Deg): {f[2]}")
            print(f"  Error: {f[3]}")
        sys.exit(1)
    else:
        print("\nSUCCESS: All orders match SciPy Intrinsic conventions.")

if __name__ == "__main__":
    test_all_orders_random()