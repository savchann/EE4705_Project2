
# Task 3 — UR5e PoE Forward Kinematics

## Overview

This task replaces MATLAB's built-in forward kinematics (`getTransform`) with the Product of Exponentials (PoE) formulation for the 6-DOF UR5e manipulator.

Six screw axes and the home transformations are calculated from the robot geometry and reused during STOMP trajectory optimization.

## Requirements

- MATLAB R2026a (or compatible version)
- Robotics System Toolbox
- Statistics and Machine Learning Toolbox

## How to Run

Open MATLAB and set the working directory to `Task_3`.

### 1. PoE Forward Kinematics Validation

Run:
```matlab
Task3_ValidatePoE
Task3_ValidatePoE_Random
Task3_ValidatePoELinks
Task3_ValidateJointPositions
```

These scripts compare PoE forward kinematics against MATLAB's `getTransform()` for the end-effector and intermediate robot links.

### 2. Main PoE-STOMP Simulation

Run:
```matlab
UR5e_PoE_STOMP_Main
```

Generates and visualizes an optimized collision-free trajectory using PoE forward kinematics.

### 3. Forward Kinematics Benchmark

Run:
```matlab
Task3_FK_Benchmark
```

Compares the computational performance and numerical accuracy of PoE against MATLAB's built-in forward kinematics.

## Results

Generated results are saved under `Task_3/Results/`, including:

- `.mat` — MATLAB simulation and validation data
- `.csv` — Numerical validation and benchmark results
- `.png` — Generated graphs
- `.fig` — Editable MATLAB figures

## Notes

- PoE uses six screw axes corresponding to the UR5e's revolute joints.
- `ur5ePoEModel.m` initializes screw axes and home transformations.
- `ur5eFKinSpace.m` implements the PoE formula.
- `updateJointsWorldPosition.m` uses PoE for link transformations during STOMP.
- The adjacent `Task 1` and `Task_2` folders must be available for shared functions.
- MATLAB's `getTransform()` is retained for initialization and independent validation.
