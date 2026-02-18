# Hexabot Positions

This directory contains scripts dedicated to calculating and verifying specific static poses and movements for the hexapod. It is useful for generating look-up tables or verifying that specific body maneuvers (like shifting weight forward/backward) are geometrically valid.

## Key Files

### Pose Calculators
- **`defaultPos.py`**: Calculates the angles and coordinates for the robot's default "home" standing pose.
- **`extended legs.py`**: Computes a pose where the legs are extended further out, likely for a wider stance or lower center of gravity.
- **`forward+x.py`**: Calculates the leg positions required to shift the body forward (positive X direction).
- **`forward-x.py`**: Calculates the leg positions required to shift the body backward (negative X direction).

### Utilities
- **`calc_rotated_tips.py`**: Helper script to calculate the effect of body rotation on the foot tip positions.

## Usage
Run these scripts to visual check if a specific body posture is achievable within the servo limits. They typically produce visual plots or output text files with the required joint angles.
