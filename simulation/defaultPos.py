from math import acos, atan2, cos, hypot, sin
import matplotlib.pyplot as plt
import numpy as np



def solve_ik_3d(x, y, z, l1, l2, l3, foot_angle):
    base_yaw = atan2(y, x)
    r = hypot(x, y)

    r_eff = r - l3 * cos(foot_angle)
    z_eff = z - l3 * sin(foot_angle)
    planar_dist = hypot(r_eff, z_eff)
    if planar_dist > l1 + l2 or planar_dist < abs(l1 - l2):
        return None

    cos_a2 = (planar_dist**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_a2 = max(-1.0, min(1.0, cos_a2))
    angle2 = -acos(cos_a2)

    phi = atan2(z_eff, r_eff)
    psi = atan2(l2 * sin(angle2), l1 + l2 * cos(angle2))
    angle1 = phi - psi
    angle3 = foot_angle - (angle1 + angle2)
    return base_yaw, angle1, angle2, angle3, phi, psi, r_eff


def deg(rad):
    return rad * 180.0 / np.pi


def within_limits(value, min_value, max_value):
    return min_value <= value <= max_value


def compute_workspace_bounds(l1, l2, l3, hip_min, hip_max, knee_min, knee_max, ankle_min, ankle_max, samples=41):
    hip_vals = np.linspace(hip_min, hip_max, samples)
    knee_vals = np.linspace(knee_min, knee_max, samples)
    ankle_vals = np.linspace(ankle_min, ankle_max, samples)

    r_min = float("inf")
    r_max = float("-inf")
    z_min = float("inf")
    z_max = float("-inf")

    for a1 in hip_vals:
        for a2 in knee_vals:
            for a3 in ankle_vals:
                r = l1 * cos(a1) + l2 * cos(a1 + a2) + l3 * cos(a1 + a2 + a3)
                z = l1 * sin(a1) + l2 * sin(a1 + a2) + l3 * sin(a1 + a2 + a3)
                r_min = min(r_min, r)
                r_max = max(r_max, r)
                z_min = min(z_min, z)
                z_max = max(z_max, z)

    return r_min, r_max, z_min, z_max


def forward_kinematics(base_yaw, angle1, angle2, angle3, l1, l2, l3):
    dir_x = cos(base_yaw)
    dir_y = sin(base_yaw)

    p0 = (0.0, 0.0, 0.0)
    p1 = p0
    r1 = l1 * cos(angle1)
    z1 = l1 * sin(angle1)
    r2 = r1 + l2 * cos(angle1 + angle2)
    z2 = z1 + l2 * sin(angle1 + angle2)
    r3 = r2 + l3 * cos(angle1 + angle2 + angle3)
    z3 = z2 + l3 * sin(angle1 + angle2 + angle3)

    p2 = (p1[0] + r1 * dir_x, p1[1] + r1 * dir_y, p1[2] + z1)
    p3 = (p1[0] + r2 * dir_x, p1[1] + r2 * dir_y, p1[2] + z2)
    p4 = (p1[0] + r3 * dir_x, p1[1] + r3 * dir_y, p1[2] + z3)
    return p0, p1, p2, p3, p4


def offset_points(points, offset):
    ox, oy, oz = offset
    return [(p[0] + ox, p[1] + oy, p[2] + oz) for p in points]


fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")
plt.subplots_adjust(left=0.0, bottom=0.0, right=1.0, top=1.0)

NUM_LEGS = 6
HIP_RADIUS = 137.5
LEG_ANGLES = np.deg2rad(np.arange(0.0, 360.0, 60.0))
HIP_POSITIONS = [(HIP_RADIUS * cos(a), HIP_RADIUS * sin(a), 0.0) for a in LEG_ANGLES]
LEG_COLORS = ["red", "orange", "brown", "blue", "purple", "green"]

lines = []
for i in range(NUM_LEGS):
    label = "leg" if i == 0 else "_nolegend_"
    leg_line, = ax.plot([], [], [], marker="o", markersize=5, color=LEG_COLORS[i], label=label)
    lines.append(leg_line)

hip_xs = [p[0] for p in HIP_POSITIONS]
hip_ys = [p[1] for p in HIP_POSITIONS]
hip_zs = [p[2] for p in HIP_POSITIONS]
ax.scatter(hip_xs, hip_ys, hip_zs, marker="s", color="black", s=20, label="hip")

center, = ax.plot([0.0], [0.0], [0.0], marker="+", color="gray", label="center")

target, = ax.plot([], [], [], marker="x", color="red", label="target")
info = ax.text2D(0.02, 0.98, "", transform=ax.transAxes, va="top", ha="left", fontsize=9)

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_title("Hexabot 3D inverse kinematics")
ax.legend(loc="upper right")

L1 = 26.0
L2 = 57.0
L3 = 122.0

HIP_MIN = np.deg2rad(-80.0)
HIP_MAX = np.deg2rad(80.0)
KNEE_MIN = np.deg2rad(-30.0)
KNEE_MAX = np.deg2rad(90.0)
ANKLE_MIN = np.deg2rad(-140.0)
ANKLE_MAX = np.deg2rad(20.0)

DEFAULT_BASE_YAW = 0.0
DEFAULT_HIP = 0.0
DEFAULT_KNEE = 0.0
DEFAULT_ANKLE = np.deg2rad(90.0)

# Fixed local target for all legs in their own hip-relative frame
# X > 0 is outward, Y = 0 is centered, Z < 0 is down
TGT_LOCAL_X = 150.0
TGT_LOCAL_Y = 0.0
TGT_LOCAL_Z = -80.0

foot_angle = np.deg2rad(0.0)
maxr = HIP_RADIUS + L1 + L2 + L3 + 50
ax.set_xlim(-maxr, maxr)
ax.set_ylim(-maxr, maxr)
ax.set_zlim(-maxr, maxr)

default_points = forward_kinematics(DEFAULT_BASE_YAW, DEFAULT_HIP, DEFAULT_KNEE, DEFAULT_ANKLE, L1, L2, L3)

# Calculate IK once for the standard leg configuration
result = solve_ik_3d(TGT_LOCAL_X, TGT_LOCAL_Y, TGT_LOCAL_Z, L1, L2, L3, foot_angle)

reachable_count = 0
first_leg_text = ""

# For visualization of the target for Leg 0
target.set_data([TGT_LOCAL_X + HIP_RADIUS], [0])
target.set_3d_properties([TGT_LOCAL_Z])

for idx, hip_pos in enumerate(HIP_POSITIONS):
    leg_angle = LEG_ANGLES[idx]
    
    # Decide which points to draw
    if result is None:
        pts = default_points
    else:
        base_yaw, angle1, angle2, angle3, phi, psi, r = result
        if not (
            within_limits(angle1, HIP_MIN, HIP_MAX)
            and within_limits(angle2, KNEE_MIN, KNEE_MAX)
            and within_limits(angle3, ANKLE_MIN, ANKLE_MAX)
        ):
             pts = default_points
        else:
            reachable_count += 1
            # Forward kinematics in local frame
            # base_yaw is relative to the leg's forward direction
            fk_points = forward_kinematics(base_yaw, angle1, angle2, angle3, L1, L2, L3)
            pts = fk_points
            
            if idx == 0:
                 first_leg_text = "\n".join(
                    [
                        f"base_yaw = {base_yaw:.3f} rad ({deg(base_yaw):.1f} deg)",
                        f"angle1   = {angle1:.3f} rad ({deg(angle1):.1f} deg)",
                        f"angle2   = {angle2:.3f} rad ({deg(angle2):.1f} deg)",
                        f"angle3   = {angle3:.3f} rad ({deg(angle3):.1f} deg)",
                        f"tgt (loc)= ({TGT_LOCAL_X:.2f}, {TGT_LOCAL_Y:.2f}, {TGT_LOCAL_Z:.2f})",
                    ]
                )

    # Rotate points from local frame to global frame
    rotated_points = []
    ca = cos(leg_angle)
    sa = sin(leg_angle)
    
    for p in pts:
        lx, ly, lz = p
        gx = lx * ca - ly * sa
        gy = lx * sa + ly * ca
        gz = lz
        rotated_points.append((gx, gy, gz))

    # Add hip position
    final_points = offset_points(rotated_points, hip_pos)

    xs = [p[0] for p in final_points]
    ys = [p[1] for p in final_points]
    zs = [p[2] for p in final_points]
    lines[idx].set_data(xs, ys)
    lines[idx].set_3d_properties(zs)

if first_leg_text:
    info.set_text(first_leg_text + f"\nreachable legs = {reachable_count}/{NUM_LEGS}")
else:
    info.set_text(f"Target unreachable\nreachable legs = {reachable_count}/{NUM_LEGS}")

plt.show()
