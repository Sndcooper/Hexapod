
from math import cos, sin, pi
import numpy as np

HIP_RADIUS = 137.5
TGT_LOCAL_X = 150.0
TGT_LOCAL_Y = 0.0
TGT_LOCAL_Z = -80.0

# Current leg angles: 0, 60, 120, 180, 240, 300
# User wants "the 2 legs on y axis instead of x axis"
# So instead of 0/180 on X, we want 90/270 on Y
# This is a +90 deg offset

LEG_ANGLES = np.deg2rad(np.arange(0.0, 360.0, 60.0) + 90.0)

results = []

for i, angle in enumerate(LEG_ANGLES):
    # Local coordinates (relative to hip)
    # The foot tip is at (TGT_LOCAL_X, TGT_LOCAL_Y, TGT_LOCAL_Z)
    # Plus hip radius
    
    # Global coordinates:
    # Hip position + Rotated local
    # Hip Pos: (HIP_RADIUS * cos(angle), HIP_RADIUS * sin(angle), 0)
    # Local Pos: (150 * cos(angle), 150 * sin(angle), -80)
    
    # Total Global Pos:
    # (HIP_RADIUS + 150) * cos(angle)
    # (HIP_RADIUS + 150) * sin(angle)
    # -80
    
    TOTAL_R = HIP_RADIUS + TGT_LOCAL_X
    
    gx = TOTAL_R * cos(angle)
    gy = TOTAL_R * sin(angle)
    gz = TGT_LOCAL_Z
    
    results.append(f"Leg {i}: ({gx:.2f}, {gy:.2f}, {gz:.2f})")

print("\n".join(results))
