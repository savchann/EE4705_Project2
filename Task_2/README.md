
# Task 2 — UR5e STOMP Path Planning

## Overview

Implementation of Stochastic Trajectory Optimization for Motion Planning (STOMP) using the 6-DOF UR5e robot manipulator in MATLAB.

The planner uses a 39-sphere collision model to optimize robot trajectories and avoid obstacles.

## Requirements

- MATLAB R2026a (or compatible version)
- Robotics System Toolbox
- Statistics and Machine Learning Toolbox

## How to Run

Open MATLAB and set the working directory to `Task_2`.

### 1. Main Simulation

Run:
```matlab
UR5e_STOMP_Path_Planning
```

Generates and visualizes the optimized UR5e trajectory.

### 2. Random Seed Validation

Run:
```matlab
Task2_Validation_Test
```

Evaluates STOMP performance across 10 random seeds.

### 3. Waypoint Validation

Run:
```matlab
Task2_Waypoint_Validation
```

Compares trajectory optimization using 10, 20, and 30 waypoints.

## Results

Simulation and validation results are saved under `Task_2/Results/`.

- `.mat` — MATLAB simulation data
- `.csv` — Validation results
- `.png` — Generated graphs
- `.fig` — Editable MATLAB figures

Waypoint comparison results are saved under `Results/Waypoint_Validation/`.

## Notes

- Default configuration: 20 waypoints and 20 sampled trajectories per iteration.
- Forward kinematics uses MATLAB's `getTransform()`.
- Task 3 replaces built-in forward kinematics with Product of Exponentials (PoE).
- Shared Task 1 functions must be available in the adjacent `Task 1` or `Task_1` folder.
