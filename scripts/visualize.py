import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation

# --- Path Setup ---
# Automatically find the build directory
current_dir = os.path.dirname(os.path.abspath(__file__))
build_dirs = [
    os.path.abspath(os.path.join(current_dir, '../cmake-build-release')),
    os.path.abspath(os.path.join(current_dir, '../build')),
    os.path.abspath(os.path.join(current_dir, '../out/build/x64-Release'))
]

sys.path.append(current_dir)

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
    # Fallback if the path setup above failed but it's installed/in path
    try:
        import quaternionlib as ql
        print(f"Loaded quaternionlib from {ql.__file__}")
    except ImportError:
        print("Error: Could not load quaternionlib. Please build the project first.")
        sys.exit(1)

# --- Simulation Constants ---
FRAMES = 360
FPS = 60
ORDER_NAME = "ZYX"
QL_ORDER = ql.RotationOrder.ZYX

# --- Trajectory Generation: Loop-the-Loop ---
# Pitch goes 0 -> 360 degrees.
# Yaw and Roll are constant (but non-zero to force complex singularity behavior)
t = np.linspace(0, 1, FRAMES)
pitch_deg = np.linspace(0, 360, FRAMES)
yaw_deg   = np.full(FRAMES, 10.0)
roll_deg  = np.full(FRAMES, 20.0)

quats = []
for p, y, r in zip(pitch_deg, yaw_deg, roll_deg):
    rot = Rotation.from_euler(ORDER_NAME, [y, p, r], degrees=True)
    quats.append(rot.as_quat())

# --- Processing ---

# 1. Stateless (SciPy default)
stateless_angles = []
for q in quats:
    r = Rotation.from_quat(q)
    # SciPy returns [z, y, x] for ZYX
    y, p, r = r.as_euler(ORDER_NAME, degrees=True)
    stateless_angles.append([y, p, r])
stateless_angles = np.array(stateless_angles)

# 2. Stateful (QuaternionLib)
stateful_angles = []
converter = ql.Converter(QL_ORDER)
for q in quats:
    res = converter.update(q)
    # Convert to degrees
    y, p, r = np.degrees(res.angles[0]), np.degrees(res.angles[1]), np.degrees(res.angles[2])
    stateful_angles.append([y, p, r])
stateful_angles = np.array(stateful_angles)

# --- Visualization ---
fig = plt.figure(figsize=(16, 9))
fig.suptitle("Gimbal Lock Singularity: The 'Loop-the-Loop' Test", fontsize=16)

# Layout:
# [ 3D Standard ] [ 3D QuaternionLib ]
# [ Graph Std   ] [ Graph QuatLib    ]

ax_3d_bad = fig.add_subplot(221, projection='3d')
ax_3d_good = fig.add_subplot(222, projection='3d')

ax_graph_bad = fig.add_subplot(223)
ax_graph_good = fig.add_subplot(224)

# --- Graph Setup ---
lines_bad = []
lines_good = []
labels = ['Yaw', 'Pitch', 'Roll']
colors = ['blue', 'orange', 'green']

for ax, data_src, title in [(ax_graph_bad, stateless_angles, "Standard Library (Stateless)"),
                            (ax_graph_good, stateful_angles, "QuaternionLib (Stateful)")]:
    ax.set_title(title)
    ax.set_ylim(-370, 370)
    ax.set_xlim(0, FRAMES)
    ax.grid(True, alpha=0.3)
    ax.axhline(90, color='red', linestyle='--', alpha=0.3)  # Singularity
    ax.axhline(270, color='red', linestyle='--', alpha=0.3) # Singularity

    current_lines = []
    for i in range(3):
        line, = ax.plot([], [], label=labels[i], color=colors[i], lw=1.5)
        current_lines.append(line)

    # Move legend outside to prevent collision
    ax.legend(loc='upper left', fontsize='small', framealpha=0.8)
    if ax == ax_graph_bad:
        lines_bad = current_lines
    else:
        lines_good = current_lines

# --- Servo Simulation State ---
# We simulate a physical servo that tries to reach the target angles.
# Discontinuities in the target will cause the servo to "whip" around.
servo_bad = np.array(stateless_angles[0])
servo_good = np.array(stateful_angles[0])
LERP_FACTOR = 0.2 # Lower = more inertia/lag, makes glitches more visible

def draw_orientation(ax, y, p, r, title, color_theme):
    ax.clear()
    ax.set_title(title, fontsize=12, fontweight='bold', color=color_theme)
    ax.set_xlim(-1, 1); ax.set_ylim(-1, 1); ax.set_zlim(-1, 1)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_box_aspect([1,1,1])

    # Reconstruct orientation for visualization
    rot = Rotation.from_euler(ORDER_NAME, [y, p, r], degrees=True)
    mat = rot.as_matrix()

    origin = np.zeros(3)
    colors_arrow = ['r', 'g', 'b']
    labels_arrow = ['x', 'y', 'z']

    for i in range(3):
        vec = mat[:, i]
        ax.quiver(0, 0, 0, vec[0], vec[1], vec[2], color=colors_arrow[i], length=1.0)
        ax.text(vec[0]*1.1, vec[1]*1.1, vec[2]*1.1, labels_arrow[i], color=colors_arrow[i])

    status_text = f"P: {p:.0f}\nY: {y:.0f}\nR: {r:.0f}"

    ax.text2D(0.05, 0.95, status_text, transform=ax.transAxes,
              bbox=dict(boxstyle='round', facecolor='white', alpha=0.5))

def update(frame):
    global servo_bad, servo_good

    # 1. Update Servo States (Simulate Physics)
    target_bad = stateless_angles[frame]
    diff_bad = target_bad - servo_bad
    servo_bad += diff_bad * LERP_FACTOR

    target_good = stateful_angles[frame]
    diff_good = target_good - servo_good
    servo_good += diff_good * LERP_FACTOR

    # 2. Update 3D Views
    # Bad View: Shows the servo state whipping around
    draw_orientation(ax_3d_bad, *servo_bad, "Standard (Servo Glitch)", 'red')

    # Good View: Shows smooth motion
    draw_orientation(ax_3d_good, *servo_good, "QuaternionLib (Smooth)", 'green')

    # 3. Update Graphs
    # We show history of the TARGETS (the control signals), not the servo state
    history_idx = range(frame + 1)

    for i in range(3):
        # Bad Graph
        lines_bad[i].set_data(history_idx, stateless_angles[:frame+1, i])
        # Good Graph
        lines_good[i].set_data(history_idx, stateful_angles[:frame+1, i])

    return lines_bad + lines_good

# Add space for title and prevent overlaps
fig.tight_layout(rect=[0, 0.03, 1, 0.95])

print("Generating animation...")
ani = animation.FuncAnimation(fig, update, frames=FRAMES, interval=20, blit=False)
plt.show()