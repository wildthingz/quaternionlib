from manim import *
import sys
import os
import numpy as np
from manim.utils.color.DVIPSNAMES import CYAN
from scipy.spatial.transform import Rotation

# --- Environment Setup ---
current_dir = os.path.dirname(os.path.abspath(__file__))
possible_build_dirs = [
    os.path.abspath(os.path.join(current_dir, '../cmake-build-release')),
    os.path.abspath(os.path.join(current_dir, '../cmake-build-debug')),
    os.path.abspath(os.path.join(current_dir, '../build')),
    os.path.abspath(os.path.join(current_dir, '../out/build/x64-Release'))
]

for d in possible_build_dirs:
    if os.path.exists(d):
        sys.path.append(d)
        try:
            import quaternionlib as ql
            print(f"SUCCESS: Loaded quaternionlib from {d}")
            break
        except ImportError:
            raise ImportError("Error: Couldn't find quaternionlib")

class GimbalLockComparison(ThreeDScene):
    def construct(self):
        # 1. Camera & Scene Setup
        # Set camera to view the vertical X-Z loop from the side/front
        self.set_camera_orientation(phi=60 * DEGREES, theta=-130 * DEGREES, zoom=0.6)

        # 2. UI Overlay
        def create_label(text, subtext=None, color=WHITE):
            g = VGroup(Text(text, font="PP Neue Machina", font_size=22, weight=BOLD, color=color, line_spacing=1.1))
            if subtext:
                g.add(Text(subtext, font="PP Supply Mono", font_size=14, color=LIGHT_GRAY))
            g.arrange(DOWN, aligned_edge=LEFT)
            bg = BackgroundRectangle(g, color=BLACK, fill_opacity=0.7, buff=0.2)
            return VGroup(bg, g)

        title = create_label("Gimbal Lock Comparison", "Vertical Loop: Pitch 0° -> 360°")
        title.to_corner(UL)

        legend_bad = create_label("■ Standard Lib", "Stateless: Glitch at 90°/270°", color=RED)
        legend_good = create_label("■ QuaternionLib", "Stateful: Smooth Motion", color=CYAN)

        legend_group = VGroup(legend_bad, legend_good).arrange(DOWN, aligned_edge=LEFT)
        legend_group.to_corner(DL)

        self.add_fixed_in_frame_mobjects(title, legend_group)

        # 3. The World
        # Sphere and Axes for context
        sphere = Sphere(radius=4.0, resolution=(24, 48))
        sphere.set_style(fill_opacity=0, stroke_opacity=0.3, stroke_color=BLUE_E)
        self.add(sphere)

        # axes = ThreeDAxes(x_range=[0,1], y_range=[0,1], z_range=[0,1], x_length=2, y_length=2, z_length=2)
        # self.add(axes)

        # 4. The Models
        def create_dart(color, opacity):
            # Fuselage: Cone pointing RIGHT (+X)
            body = Cone(base_radius=0.2, height=1.0, direction=RIGHT)
            body.set_color(color).set_opacity(opacity)

            # Wings: Flat in X-Y plane
            wings = Polygon(
                [0.2, 0, 0],     # Root (Near nose)
                [-0.4, 1.0, 0],  # Left Wingtip
                [-0.4, -1.0, 0]  # Right Wingtip
            )
            wings.set_color(color).set_opacity(opacity * 0.6)
            # No rotation needed; wings naturally lie in X-Y plane

            return VGroup(body, wings)

        plane_bad = create_dart(RED, 0.9)
        plane_good = create_dart(CYAN, 1.0)

        # Shift slightly in Y (Depth) so they don't overlap
        plane_bad.shift(LEFT * 0.15)  # Manim UP is Y (Depth in this view)
        plane_good.shift(RIGHT * 0.15)

        # 5. Trajectory Visualization
        # trail = TracedPath(plane_good.get_center, dissipating_time=2.0, stroke_opacity=[0, 1], stroke_color=YELLOW, stroke_width=4)
        self.add(plane_bad, plane_good)

        # 6. Simulation State
        self.servo_bad = np.array([10.0, 0.0, 20.0]) # Initial angles [Yaw, Pitch, Roll]
        self.servo_good = np.array([10.0, 0.0, 20.0])

        self.converter = ql.Converter(ql.RotationOrder.ZYX)

        progress = ValueTracker(0.0)

        # --- HELPER: Unwrap Angles ---
        def unwrap_to_nearest(target, current):
            """
            Finds the equivalent angle for 'target' that is closest to 'current'.
            Resolves the 360-degree jump issue (e.g., -270 vs +90).
            """
            diff = target - current
            # Wrap diff to [-180, 180]
            diff = (diff + 180.0) % 360.0 - 180.0
            return current + diff

        # 7. Update Logic
        def update_simulation(mob, dt):
            t = progress.get_value()

            # --- A. Trajectory ---
            radius = 4.0
            theta_deg = t * 360.0
            theta_rad = np.radians(theta_deg)

            pos_local = np.array([
                radius * np.sin(theta_rad),
                0,
                -radius * np.cos(theta_rad)
            ])
            pos_final = pos_local

            # --- B. Ground Truth ---
            target_pitch = -theta_deg
            fixed_yaw = 10.0
            fixed_roll = 20.0

            rot = Rotation.from_euler('ZYX', [fixed_yaw, target_pitch, fixed_roll], degrees=True)
            q = rot.as_quat()

            # --- C. Solving ---
            # 1. Standard (Bad)
            raw_bad = np.array(Rotation.from_quat(q).as_euler('ZYX', degrees=True))

            # 2. QuaternionLib (Good)
            res = self.converter.update(q)
            raw_good = np.degrees(res.angles)

            # --- D. Physics / Servo Lag (With Unwrap Fix) ---
            # We unwrap the raw target relative to the CURRENT servo state.
            # This ensures that even if the library output jumps 360 deg, the servo sees 0 change.
            target_bad = np.zeros(3)
            target_good = np.zeros(3)

            for i in range(3):
                # We do NOT unwrap 'Bad' because we WANT to see the glitch if it snaps 180!
                # But Scipy might snap 360 too, which is boring.
                # Let's only unwrap the Good one to ensure smoothness.
                target_bad[i] = raw_bad[i]
                target_good[i] = unwrap_to_nearest(raw_good[i], self.servo_good[i])

            LERP = 5.0 * dt
            self.servo_bad += (target_bad - self.servo_bad) * LERP
            self.servo_good += (target_good - self.servo_good) * LERP

            # --- E. Apply Orientation ---
            for plane, angles in [(plane_bad, self.servo_bad), (plane_good, self.servo_good)]:
                plane.restore()

                r_mat = Rotation.from_euler('ZYX', angles, degrees=True).as_matrix()
                plane.apply_matrix(r_mat)

                if plane is plane_bad:
                    plane.shift(pos_final + LEFT * 0.15)
                else:
                    plane.shift(pos_final + RIGHT * 0.15)

        plane_good.save_state()
        plane_bad.save_state()
        plane_good.add_updater(update_simulation)
        plane_bad.add_updater(update_simulation)

        # 9. Run
        self.wait(0.1)
        self.play(
            progress.animate.set_value(1.0),
            run_time=5,
            rate_func=linear
        )
        # self.wait(1)