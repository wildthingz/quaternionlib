import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation
import pytest

# --- Path Setup to find the C++ Module ---
# Automatically looks in common build directories
build_dirs = [
    os.path.abspath(os.path.join(os.path.dirname(__file__), '../../cmake-build-release')),
    os.path.abspath(os.path.join(os.path.dirname(__file__), '../build')),
    os.path.abspath(os.path.join(os.path.dirname(__file__), '../out/build/x64-Release'))
]

found = False
for d in build_dirs:
    if os.path.exists(d):
        sys.path.append(d)
        found = True
        break

try:
    import quaternionlib as ql
    print(f"Loaded quaternionlib from {ql.__file__}")
except ImportError:
    print("CRITICAL: Could not import 'quaternionlib'. Please compile the C++ project first.")
    sys.exit(1)

# --- Helpers ---

def unwrap_angle(angle, prev_angle):
    """
    Helper to unwrap angles to ensure continuous velocity calculation.
    """
    diff = angle - prev_angle
    # If jump is > pi, wrap it (standard shortest path logic)
    diff = (diff + np.pi) % (2 * np.pi) - np.pi
    return prev_angle + diff

def check_continuity(converter, pitch_seq, yaw_seq, roll_seq, order_str='zyx'):
    """
    Runs a trajectory through the converter and asserts smoothness.
    """
    prev_angles = None
    prev_velocity = None
    max_jerk = 0.0
    failures = 0

    # Warmup frames (history buffer filling)
    warmup = 5

    for i, (p, y, r) in enumerate(zip(pitch_seq, yaw_seq, roll_seq)):
        # 1. Ground Truth (Input Quaternion)
        rot = Rotation.from_euler(order_str, [y, p, r], degrees=True)
        q = rot.as_quat() # [x, y, z, w]

        # 2. Convert using Library
        res = converter.update(q)
        current = res.angles # Radians

        # 3. Check Velocity
        if i > warmup and prev_angles is not None:
            # Unwrap current against previous to get true physical delta
            unwrapped = np.zeros(3)
            for k in range(3):
                unwrapped[k] = unwrap_angle(current[k], prev_angles[k])

            velocity = np.linalg.norm(unwrapped - prev_angles)

            if prev_velocity is not None:
                # Jerk = Change in Velocity (Acceleration change)
                # A stateless solver snaps, causing infinite jerk.
                jerk = abs(velocity - prev_velocity)
                max_jerk = max(max_jerk, jerk)

                # Thresholds:
                # 0.2 rad/frame is a violent snap.
                # 0.05 rad/frame is acceptable noise/acceleration.
                if jerk > 0.15:
                    print(f"FAIL at Frame {i}: Pitch={p:.1f}, Jerk={jerk:.4f}")
                    failures += 1

            prev_velocity = velocity

        prev_angles = current

    return failures, max_jerk

# --- Tests ---

def test_north_pole_crossing():
    """
    Scenario: ZYX Order. Pitch goes 80 -> 100 (crossing 90).
    This is the classic "Gimbal Lock" singularity.
    """
    print("\n[Test] ZYX North Pole Crossing")
    converter = ql.Converter(ql.RotationOrder.ZYX)

    frames = 300
    t = np.linspace(0, 1, frames)

    # Trajectory
    pitch = np.interp(t, [0, 1], [80, 100])
    yaw = np.sin(t * 10) * 20 + 10  # Oscillating Yaw
    roll = t * 50 + 45              # Drifting Roll

    fails, jerk = check_continuity(converter, pitch, yaw, roll, 'zyx')

    print(f"Max Jerk: {jerk:.4f}")
    assert fails == 0, f"Detected {fails} velocity discontinuities during singularity."

def test_all_orders_random_smooth():
    """
    Scenario: Random smooth trajectories for ALL 12 rotation orders.
    Ensures that Tait-Bryan and Proper Euler logic are both robust.
    """
    print("\n[Test] All Orders Random Fuzzing")

    # Map string to enum
    orders = {
        'XYZ': ql.RotationOrder.XYZ, 'XZY': ql.RotationOrder.XZY,
        'YXZ': ql.RotationOrder.YXZ, 'YZX': ql.RotationOrder.YZX,
        'ZXY': ql.RotationOrder.ZXY, 'ZYX': ql.RotationOrder.ZYX,
        'XYX': ql.RotationOrder.XYX, 'XZX': ql.RotationOrder.XZX,
        'YXY': ql.RotationOrder.YXY, 'YZY': ql.RotationOrder.YZY,
        'ZXZ': ql.RotationOrder.ZXZ, 'ZYZ': ql.RotationOrder.ZYZ,
    }

    frames = 100
    t = np.linspace(0, 1, frames)

    for name, enum_val in orders.items():
        converter = ql.Converter(enum_val)

        # Generate random smooth waves
        f1, f2, f3 = np.random.uniform(0.5, 2.0, 3) # Random frequencies
        a1, a2, a3 = np.random.uniform(10, 90, 3)   # Random amplitudes

        # Angles
        ax1 = np.sin(t * f1) * a1
        ax2 = np.cos(t * f2) * a2 # This will likely cross 0 or 90
        ax3 = np.sin(t * f3) * a3

        # Run Check
        fails, jerk = check_continuity(converter, ax2, ax1, ax3, name.lower())

        status = "PASS" if fails == 0 else "FAIL"
        print(f"Order {name}: {status} (Max Jerk: {jerk:.4f})")

        assert fails == 0, f"Order {name} failed continuity test."

if __name__ == "__main__":
    # Allow running as standalone script
    try:
        test_north_pole_crossing()
        test_all_orders_random_smooth()
        print("\nAll Fuzzing Tests Passed Successfully.")
    except AssertionError as e:
        print(f"\nTEST FAILED: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\nERROR: {e}")
        sys.exit(1)