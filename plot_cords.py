"""
Hexapod Gait Simulator
======================
Animates a full tripod gait in a single matplotlib window.
Smoothly interpolates between keyframe poses.

Usage:  python plot_cords.py
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ==========================================
# ROBOT GEOMETRY
# ==========================================
L1 = 26.0       # Coxa  (mm)
L2 = 57.0       # Femur (mm)
L3 = 122.0      # Tibia (mm)
HIP_RADIUS = 137.5

LEG_MOUNT_ANGLES = np.deg2rad([0, 60, 120, 180, 240, 300])

# ==========================================
# IK
# ==========================================
def solve_ik_3d(x, y, z, l1, l2, l3):
    base_yaw = math.atan2(y, x)
    r_global = math.hypot(x, y)
    r_target = r_global - l1
    d_sq = r_target**2 + z**2
    d = math.sqrt(d_sq)
    if d > l2 + l3 or d < abs(l2 - l3):
        return None
    cos_knee = (l2**2 + l3**2 - d_sq) / (2 * l2 * l3)
    cos_knee = max(-1.0, min(1.0, cos_knee))
    knee_interior = math.acos(cos_knee)
    angle3_geom = -(math.pi - knee_interior)
    angle3 = angle3_geom + (math.pi / 2.0)
    az = math.atan2(z, r_target)
    cos_offset = (l2**2 + d_sq - l3**2) / (2 * l2 * d)
    cos_offset = max(-1.0, min(1.0, cos_offset))
    femur_offset = math.acos(cos_offset)
    angle2 = az + femur_offset
    return base_yaw, 0.0, angle2, angle3

# ==========================================
# FK  (joint positions for plotting)
# ==========================================
def get_leg_points(leg_index, tx, ty, tz):
    mount_angle = LEG_MOUNT_ANGLES[leg_index]
    hip_x = HIP_RADIUS * math.cos(mount_angle)
    hip_y = HIP_RADIUS * math.sin(mount_angle)
    dx, dy, dz = tx - hip_x, ty - hip_y, tz
    local_x = dx * math.cos(-mount_angle) - dy * math.sin(-mount_angle)
    local_y = dx * math.sin(-mount_angle) + dy * math.cos(-mount_angle)
    result = solve_ik_3d(local_x, local_y, dz, L1, L2, L3)
    if result is None:
        return None
    base_yaw, _, angle2, angle3 = result
    x1 = L1 * math.cos(base_yaw); y1 = L1 * math.sin(base_yaw); z1 = 0
    r2 = L1 + L2 * math.cos(angle2); z2 = L2 * math.sin(angle2)
    x2 = r2 * math.cos(base_yaw); y2 = r2 * math.sin(base_yaw)
    a3g = angle3 - (math.pi / 2.0)
    r3 = L1 + L2*math.cos(angle2) + L3*math.cos(angle2 + a3g)
    z3 = L2*math.sin(angle2) + L3*math.sin(angle2 + a3g)
    x3 = r3 * math.cos(base_yaw); y3 = r3 * math.sin(base_yaw)
    def to_g(lx, ly, lz):
        return (lx*math.cos(mount_angle) - ly*math.sin(mount_angle) + hip_x,
                lx*math.sin(mount_angle) + ly*math.cos(mount_angle) + hip_y, lz)
    return [to_g(0,0,0), to_g(x1,y1,z1), to_g(x2,y2,z2), to_g(x3,y3,z3)]

# ==========================================
# INTERPOLATION
# ==========================================
def lerp_poses(pose_a, pose_b, t):
    """Linearly interpolate between two poses (t=0→pose_a, t=1→pose_b)."""
    result = []
    for (ax,ay,az), (bx,by,bz) in zip(pose_a, pose_b):
        result.append((ax + (bx-ax)*t, ay + (by-ay)*t, az + (bz-az)*t))
    return result

# ==========================================
# BODY HEXAGON (pre-computed)
# ==========================================
LEG_COLORS = ['r', 'g', 'b', 'c', 'm', 'y']
_hip_pts = []
for _i in range(6):
    _a = LEG_MOUNT_ANGLES[_i]
    _hip_pts.append((HIP_RADIUS*math.cos(_a), HIP_RADIUS*math.sin(_a), 0))
_hip_pts.append(_hip_pts[0])
_hx, _hy, _hz = zip(*_hip_pts)

# ==========================================
# ANIMATION
# ==========================================
def animate_gait(poses, steps_per_move=20, num_cycles=3, delay=0.02):
    """
    Animate the gait in a single matplotlib window.
    poses         : list of keyframe poses (each = list of 6 (x,y,z) tuples)
    steps_per_move: interpolation steps between each pair of keyframes
    num_cycles    : how many times to repeat the full gait
    delay         : seconds between frames
    """
    plt.ion()
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    print(f"Animating {num_cycles} cycle(s), {len(poses)} keyframes each, "
          f"{steps_per_move} steps per move...")
    print("Close the window to stop.\n")

    try:
        for cycle in range(num_cycles):
            for kf in range(len(poses) - 1):
                for step in range(steps_per_move):
                    t = step / float(steps_per_move)
                    frame = lerp_poses(poses[kf], poses[kf+1], t)

                    ax.clear()

                    # Body
                    ax.plot(_hx, _hy, _hz, 'k-', linewidth=2)

                    # Legs
                    for i in range(6):
                        pts = get_leg_points(i, *frame[i])
                        if pts:
                            px, py, pz = zip(*pts)
                            ax.plot(px, py, pz, '-o', color=LEG_COLORS[i],
                                    linewidth=2, markersize=4)

                    ax.set_xlim(-400, 400)
                    ax.set_ylim(-400, 400)
                    ax.set_zlim(-200, 200)
                    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
                    ax.set_title(f"Cycle {cycle+1}/{num_cycles}  —  "
                                 f"Move {kf+1}/{len(poses)-1}  "
                                 f"(step {step+1}/{steps_per_move})")

                    plt.draw()
                    plt.pause(delay)

            print(f"  Cycle {cycle+1} complete.")

    except Exception:
        pass  # window closed by user

    plt.ioff()
    print("Animation finished.")
    plt.show()

# ==========================================
# FORWARD-WALKING TRIPOD GAIT
# ==========================================
if __name__ == "__main__":

    HALF_STRIDE = 15.0   # mm each side of home along X
    LIFT        = 40.0   # mm lift height
    STEPS       = 15     # interpolation steps per move
    CYCLES      = 3      # number of gait cycles to repeat

    S  = HALF_STRIDE
    Z  = -122.00
    ZL = Z + LIFT     # -82

    POSES = [
        # # P0 : Home (all down)
        # [
        #     (220.50,    0.00,  Z),
        #     (110.25,  190.96,  Z),
        #     (-110.25, 190.96,  Z),
        #     (-220.50,   0.00,  Z),
        #     (-110.25,-190.96,  Z),
        #     (110.25, -190.96,  Z),
        # ],
        # P1 : Lift Tripod A (0,2,4)
        [
            (220.50,    0.00, ZL),
            (110.25,  190.96,  Z),
            (-110.25, 190.96, ZL),
            (-220.50,   0.00,  Z),
            (-110.25,-190.96, ZL),
            (110.25, -190.96,  Z),
        ],
        # P2 : Swing A fwd / Push B back
        [
            (220.50+S,    0.00, ZL),
            (110.25-S,  190.96,  Z),
            (-110.25+S, 190.96, ZL),
            (-220.50-S,   0.00,  Z),
            (-110.25+S,-190.96, ZL),
            (110.25-S, -190.96,  Z),
        ],
        # P3 : Plant A
        [
            (220.50+S,    0.00,  Z),
            (110.25-S,  190.96,  Z),
            (-110.25+S, 190.96,  Z),
            (-220.50-S,   0.00,  Z),
            (-110.25+S,-190.96,  Z),
            (110.25-S, -190.96,  Z),
        ],
        # P4 : Lift Tripod B (1,3,5)
        [
            (220.50+S,    0.00,  Z),
            (110.25-S,  190.96, ZL),
            (-110.25+S, 190.96,  Z),
            (-220.50-S,   0.00, ZL),
            (-110.25+S,-190.96,  Z),
            (110.25-S, -190.96, ZL),
        ],
        # P5 : Swing B fwd / Push A back
        [
            (220.50-S,    0.00,  Z),
            (110.25+S,  190.96, ZL),
            (-110.25-S, 190.96,  Z),
            (-220.50+S,   0.00, ZL),
            (-110.25-S,-190.96,  Z),
            (110.25+S, -190.96, ZL),
        ],
        # P6 : Plant B  → back to shifted home (cycle repeats from here)
        [
            (220.50-S,    0.00,  Z),
            (110.25+S,  190.96,  Z),
            (-110.25-S, 190.96,  Z),
            (-220.50+S,   0.00,  Z),
            (-110.25-S,-190.96,  Z),
            (110.25+S, -190.96,  Z),
        ],
    ]

    animate_gait(POSES, steps_per_move=STEPS, num_cycles=CYCLES, delay=0.02)
