# Hexabot Repository

## Overview

The **Hexabot** repository is a comprehensive robotics project dedicated to the simulation, control, and gait analysis of a six-legged robot, or hexapod. Designed for both virtual simulation and physical implementation, this project explores the complexities of legged locomotion, inverse kinematics (IK), and path planning. At its core, the repository features a robust collection of Python scripts that calculate and visualize the precise joint angles required for various body postures and movements.

The project is structured to facilitate easy experimentation with different gait patterns, such as the tripod gait, where three legs move simultaneously while the other three provide stability. Key components include `defaultPos.py` for establishing the robot's neutral stance, and `gait_sim.py`, which simulates phase-based walking cycles, allowing users to visualize leg trajectories and body positioning in real-time. The inverse kinematics engine, a critical part of the system, mathematically determines the necessary servo angles to achieve desired foot positions, ensuring fluid and accurate movement.

Beyond basic locomotion, the repository includes utilities for advanced maneuvers. Scripts like `extended_legs.py` and `forward_plus_x.py` demonstrate the robot's ability to adapt its body configuration for stability or directional shifts. The inclusion of `calc_rotated_tips.py` further enhances the system's capability to handle complex body rotations.

This repository serves as a valuable resource for robotics enthusiasts, students, and engineers interested in the mechanics of hexapod robots. Whether you are looking to build your own hexapod, understand the underlying mathematics of legged robots, or simply experiment with gait algorithms, Hexabot provides a solid foundation and a flexible framework for further development and innovation.

## Repository Structure

The project has been organized into the following directories to improve maintainability and navigation:

### `simulation/`
Contains the core logic for kinematics, gait simulation, and pose calculations.
- **`gait_sim.py`**: The main simulation script for the phase-based tripod gait. It visualizes the robot's movement and can send serial commands to the physical robot.
- **`defaultPos.py`**: Calculates the angles and coordinates for the robot's default "home" standing pose.
- **`extended_legs.py`**: Computes a pose with legs extended further out for a wider stance.
- **`forward_plus_x.py`** / **`forward_minus_x.py`**: Calculates leg positions to shift the body forward or backward along the X-axis.
- **`calc_rotated_tips.py`**: Helper script to calculate the effect of body rotation on foot tip positions.
- **`plot_cords.py`**: Visualizes coordinate data for debugging and verification.
- **`update_gait_cell.py`**: Utility for updating gait cell configurations.

### `tests/`
Includes scripts for testing individual components and hardware.
- **`servo_test.py`**: Simple script to test servo motor responsiveness and calibration.
- **`one_leg_test.ipynb`**: A Jupyter Notebook for testing and tuning the inverse kinematics of a single leg.

### `data/`
Stores static data files used by the simulations.
- **`cords.txt`**: Contains coordinate data used for path planning or gait generation.

## Usage

1.  **Simulation**: To run the main gait simulation:
    ```bash
    python simulation/gait_sim.py
    ```
2.  **Pose Verification**: To check a specific static pose, run the corresponding script in the `simulation/` folder:
    ```bash
    python simulation/defaultPos.py
    ```
3.  **Hardware Testing**: Use the scripts in `tests/` to verify servo connections and individual leg movements before running full gaits.

## 300-Character Description
*For quick previews:*

**Hexabot** is a Python-based robotics framework for simulating and controlling hexapod walkers. It features real-time inverse kinematics, tripod gait generation, and 3D visualization. verification tools included for custom poses and hardware testing. Perfect for learning legged locomotion mechanics.
