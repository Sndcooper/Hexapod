"""
Hexapod Gait Runner — Phase-Based with Serial Output
=====================================================
Runs the phase-based tripod gait on the real hexapod AND shows the
matplotlib simulation simultaneously.

Features:
  • Phase-based tripod gait (no keyframes, continuous)
  • Parabolic arc swing trajectories
  • Body velocity control
  • IK → Servo microseconds with clamping
  • Serial output to COM17 (leg order: 0,1,2,5,4,3)

Usage:  python gait_sim.py
"""

import math
import numpy as np
import serial
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ==========================================
# ROBOT GEOMETRY
# ==========================================
L1 = 26.0        # Coxa  (mm)
L2 = 57.0        # Femur (mm)
L3 = 122.0       # Tibia (mm)
HIP_RADIUS = 137.5

LEG_MOUNT_ANGLES = np.deg2rad([0, 60, 120, 180, 240, 300])

HOME_RADIAL = 220.50
HOME_Z = -122.0

HOME_POSITIONS = []
for _i in range(6):
    _a = LEG_MOUNT_ANGLES[_i]
    HOME_POSITIONS.append((HOME_RADIAL * math.cos(_a),
                           HOME_RADIAL * math.sin(_a),
                           HOME_Z))

TRIPOD_A = {0, 2, 4}
TRIPOD_B = {1, 3, 5}

# ==========================================
# SERVO CONFIG
# ==========================================
PORT = "COM17"
BAUD = 115200

SERVO_MID_US = 1650

HIP_MIN_US  = 1000;  HIP_MAX_US  = 2300
KNEE_MIN_US = 1200;  KNEE_MAX_US = 2300
FOOT_MIN_US = 1000;  FOOT_MAX_US = 2050

HIP_DIR  =  1
KNEE_DIR =  1
FOOT_DIR = -1   # reversed

# Serial leg order: physical wiring goes 0,1,2,5,4,3
SERIAL_LEG_ORDER = [0, 1, 2, 5, 4, 3]

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
    angle2 = az + math.acos(cos_offset)
    return base_yaw, 0.0, angle2, angle3

# ==========================================
# ANGLE → MICROSECONDS (with clamp)
# ==========================================
def angle_to_us(angle_deg, clamp_min, clamp_max, direction=1):
    rate = 1000.0 / 90.0
    us = SERVO_MID_US + angle_deg * rate * direction
    return int(max(clamp_min, min(clamp_max, us)))

def get_servo_us_for_leg(leg_index, tx, ty, tz):
    """Convert global foot target to 3 servo us values."""
    mount = LEG_MOUNT_ANGLES[leg_index]
    hx = HIP_RADIUS * math.cos(mount)
    hy = HIP_RADIUS * math.sin(mount)
    dx, dy = tx - hx, ty - hy
    lx = dx * math.cos(-mount) - dy * math.sin(-mount)
    ly = dx * math.sin(-mount) + dy * math.cos(-mount)

    result = solve_ik_3d(lx, ly, tz, L1, L2, L3)
    if result is None:
        return SERVO_MID_US, SERVO_MID_US, SERVO_MID_US

    yaw, _, a2, a3 = result
    us_hip  = angle_to_us(math.degrees(yaw), HIP_MIN_US,  HIP_MAX_US,  HIP_DIR)
    us_knee = angle_to_us(math.degrees(a2),  KNEE_MIN_US, KNEE_MAX_US, KNEE_DIR)
    us_foot = angle_to_us(math.degrees(a3),  FOOT_MIN_US, FOOT_MAX_US, FOOT_DIR)
    return us_hip, us_knee, us_foot

# ==========================================
# FK  (joint positions for drawing)
# ==========================================
def get_leg_points(leg_index, tx, ty, tz):
    mount = LEG_MOUNT_ANGLES[leg_index]
    hx = HIP_RADIUS * math.cos(mount)
    hy = HIP_RADIUS * math.sin(mount)
    dx, dy = tx - hx, ty - hy
    lx = dx * math.cos(-mount) - dy * math.sin(-mount)
    ly = dx * math.sin(-mount) + dy * math.cos(-mount)
    result = solve_ik_3d(lx, ly, tz, L1, L2, L3)
    if result is None:
        return None
    yaw, _, a2, a3 = result
    cx, cy = L1 * math.cos(yaw), L1 * math.sin(yaw)
    r2 = L1 + L2 * math.cos(a2)
    kx, ky, kz = r2 * math.cos(yaw), r2 * math.sin(yaw), L2 * math.sin(a2)
    a3g = a3 - math.pi / 2
    r3 = L1 + L2 * math.cos(a2) + L3 * math.cos(a2 + a3g)
    fz = L2 * math.sin(a2) + L3 * math.sin(a2 + a3g)
    fx, fy = r3 * math.cos(yaw), r3 * math.sin(yaw)
    def g(x, y, z):
        return (x*math.cos(mount) - y*math.sin(mount) + hx,
                x*math.sin(mount) + y*math.cos(mount) + hy, z)
    return [g(0,0,0), g(cx,cy,0), g(kx,ky,kz), g(fx,fy,fz)]

# ==========================================
# PHASE-BASED FOOT POSITION
# ==========================================
def foot_position(leg_index, global_phase, half_stride, lift_height):
    hx, hy, _ = HOME_POSITIONS[leg_index]
    offset = 0.0 if leg_index in TRIPOD_A else math.pi
    local = (global_phase + offset) % (2 * math.pi)

    if local < math.pi:
        # STANCE: foot on ground, slides backward
        t = local / math.pi
        x_off = half_stride * (1.0 - 2.0 * t)
        z = HOME_Z
    else:
        # SWING: foot in air, arc forward
        t = (local - math.pi) / math.pi
        x_off = half_stride * (-1.0 + 2.0 * t)
        z = HOME_Z + lift_height * math.sin(math.pi * t)

    return (hx + x_off, hy, z)

# ==========================================
# BODY HEXAGON
# ==========================================
LEG_COLORS = ['r', 'g', 'b', 'c', 'm', 'y']
_hip_pts = []
for _i in range(6):
    _a = LEG_MOUNT_ANGLES[_i]
    _hip_pts.append((HIP_RADIUS*math.cos(_a), HIP_RADIUS*math.sin(_a), 0))
_hip_pts.append(_hip_pts[0])
_bx, _by, _bz = zip(*_hip_pts)

# ==========================================
# SIMULATION + SERIAL
# ==========================================
def simulate(body_speed, cycle_time, lift_height, num_cycles,
             step_delay=0.02, use_serial=True, show_plot=True):
    """
    Run the gait on the real hexapod and/or show the simulation.

    body_speed  : forward speed in mm/s
    cycle_time  : duration of one full gait cycle (s)
    lift_height : peak swing height (mm)
    num_cycles  : cycles to run
    step_delay  : seconds between frames (0.02 = 50fps)
    use_serial  : whether to send to COM17
    show_plot   : whether to show matplotlib animation
    """
    stance_time = cycle_time / 2.0
    stride      = body_speed * stance_time
    half_stride = stride / 2.0

    # Calculate frames: ~25 steps per move, 6 moves per cycle
    steps_per_move = 60
    moves_per_cycle = 6
    frames_per_cycle = steps_per_move * moves_per_cycle  # 150
    total_frames = frames_per_cycle * num_cycles
    d_phase = 2 * math.pi / frames_per_cycle

    print("=" * 55)
    print("  HEXAPOD GAIT RUNNER  (Phase-Based + Serial)")
    print("=" * 55)
    print(f"  Body speed     : {body_speed} mm/s")
    print(f"  Cycle time     : {cycle_time} s")
    print(f"  Stride         : {stride:.1f} mm  (half = ±{half_stride:.1f})")
    print(f"  Lift height    : {lift_height} mm")
    print(f"  Cycles         : {num_cycles}")
    print(f"  Steps/move     : {steps_per_move}")
    print(f"  Step delay     : {step_delay}s")
    print(f"  Serial         : {'ON → ' + PORT if use_serial else 'OFF'}")
    print(f"  Serial order   : legs {SERIAL_LEG_ORDER}")
    print(f"  Total frames   : {total_frames}")
    print("=" * 55)

    # Serial connection
    ser = None
    if use_serial:
        try:
            ser = serial.Serial(PORT, BAUD, timeout=1)
            time.sleep(2)
            print(f"  Serial connected to {PORT}.\n")
        except serial.SerialException as e:
            print(f"  ⚠ Serial error: {e}")
            print(f"  Running simulation only.\n")
            ser = None

    # Plot setup
    if show_plot:
        plt.ion()
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

    phase = 0.0
    try:
        for f in range(total_frames):
            # 1. Compute foot positions
            coords = [foot_position(i, phase, half_stride, lift_height)
                      for i in range(6)]

            # 2. Convert to servo us for all 6 legs
            all_us = {}
            for i in range(6):
                all_us[i] = get_servo_us_for_leg(i, *coords[i])

            # 3. Build serial string in order [0,1,2,5,4,3]
            serial_values = []
            for leg in SERIAL_LEG_ORDER:
                serial_values.extend(all_us[leg])

            # 4. Send to hexapod
            if ser:
                line = ",".join(str(v) for v in serial_values) + "\n"
                ser.write(line.encode("ascii"))

            # 5. Draw
            if show_plot:
                ax.clear()
                ax.plot(_bx, _by, _bz, 'k-', linewidth=2)
                for i in range(6):
                    pts = get_leg_points(i, *coords[i])
                    if pts:
                        px, py, pz = zip(*pts)
                        ax.plot(px, py, pz, '-o', color=LEG_COLORS[i],
                                linewidth=2, markersize=4)
                ax.set_xlim(-400, 400)
                ax.set_ylim(-400, 400)
                ax.set_zlim(-200, 200)
                ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
                cycle_num = f // frames_per_cycle + 1
                ax.set_title(f"Gait Runner  |  Speed {body_speed} mm/s  |  "
                             f"Cycle {cycle_num}/{num_cycles}")
                plt.draw()
                plt.pause(step_delay)
            else:
                time.sleep(step_delay)

            phase += d_phase

            # Cycle progress
            if (f + 1) % frames_per_cycle == 0:
                print(f"  Cycle {(f+1)//frames_per_cycle} complete.")

    except KeyboardInterrupt:
        print("\n  Stopped by user.")
    except Exception:
        pass

    # Cleanup
    if ser:
        # Return to mid
        mid_line = ",".join([str(SERVO_MID_US)] * 18) + "\n"
        ser.write(mid_line.encode("ascii"))
        ser.close()
        print("  Serial closed. Servos at mid.")

    if show_plot:
        plt.ioff()
        print("Simulation finished.")
        plt.show()

# ==========================================
# CONFIG — Tune these and run!
# ==========================================
if __name__ == "__main__":

    simulate(
        body_speed  = 200.0,   # mm/s forward (stride = 100mm)
        cycle_time  = 1.0,     # seconds per gait cycle
        lift_height = 40.0,    # mm peak swing height
        num_cycles  = 20,       # cycles to run
        step_delay  = 0.0001,    # 20ms per frame (servos need this!)
        use_serial  = True,    # send to COM17
        show_plot   = True,   # OFF for smooth servo motion (plot causes jitter)
    )
