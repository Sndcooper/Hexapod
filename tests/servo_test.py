"""
Servo Joint Test — Move each joint from mid to its limits
==========================================================
Sends 18 us values to COM17. Tests one joint at a time:
  mid → min → mid → max → mid  then moves to the next joint.

Order: Leg0(hip,knee,foot), Leg1(hip,knee,foot), ... Leg5(hip,knee,foot)
"""

import serial
import time

PORT = "COM17"
BAUD = 115200

# Servo mid and limits (microseconds)
SERVO_MID = 1650

# Per-joint-type limits: (min, max)
LIMITS = {
    "hip":  (1000, 2300),
    "knee": (1200, 2300),
    "foot": (1000, 2050),
}

# Joint order per leg: hip, knee, foot  (repeated for 6 legs = 18 servos)
JOINT_TYPES = ["hip", "knee", "foot"] * 6

# How many interpolation steps for each move
STEPS = 25
STEP_DELAY = 0.02   # seconds between each step (controls speed)
PAUSE_AT_LIMIT = 0.5 # seconds to hold at min/max before returning

def interpolate(start, end, steps):
    """Generate values from start to end over N steps."""
    for i in range(steps + 1):
        t = i / steps
        yield int(start + (end - start) * t)

def send_values(ser, values):
    """Send 18 comma-separated us values + newline."""
    line = ",".join(str(v) for v in values) + "\n"
    ser.write(line.encode("ascii"))

def test_joint(ser, joint_index):
    """Move one joint: mid → min → mid → max → mid."""
    jtype = JOINT_TYPES[joint_index]
    j_min, j_max = LIMITS[jtype]
    leg = joint_index // 3
    joint_name = JOINT_TYPES[joint_index % 3]

    print(f"\n  Joint {joint_index:2d}  (Leg {leg} {joint_name:4s})  "
          f"range [{j_min} — {SERVO_MID} — {j_max}]")

    # All servos start at mid
    values = [SERVO_MID] * 18

    def sweep(start, end, label):
        print(f"    {label}: {start} → {end}", end="", flush=True)
        for v in interpolate(start, end, STEPS):
            values[joint_index] = v
            send_values(ser, values)
            time.sleep(STEP_DELAY)
        print(f"  ✓")

    # Mid → Min
    sweep(SERVO_MID, j_min, "mid→min")
    time.sleep(PAUSE_AT_LIMIT)

    # Min → Mid
    sweep(j_min, SERVO_MID, "min→mid")
    time.sleep(PAUSE_AT_LIMIT)

    # Mid → Max
    sweep(SERVO_MID, j_max, "mid→max")
    time.sleep(PAUSE_AT_LIMIT)

    # Max → Mid
    sweep(j_max, SERVO_MID, "max→mid")
    time.sleep(PAUSE_AT_LIMIT)

    # Ensure it's back at mid
    values[joint_index] = SERVO_MID
    send_values(ser, values)

if __name__ == "__main__":
    print(f"Connecting to {PORT}...")
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
    print(f"Connected.\n")

    # Send all mid first
    send_values(ser, [SERVO_MID] * 18)
    print("All servos set to mid (1650).")
    time.sleep(1)

    print(f"\nTesting all 18 joints one by one...")
    print(f"Speed: {STEPS} steps × {STEP_DELAY}s = {STEPS*STEP_DELAY:.1f}s per move")

    for j in range(18):
        test_joint(ser, j)

    print("\n✓ All joints tested. Servos at mid.")
    ser.close()
