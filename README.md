# EE4705 Project 2 — STOMP Robot Motion Planning

MATLAB implementation and evaluation of **Stochastic Trajectory Optimization for Motion Planning (STOMP)** for the **Kinova Gen3** and **Universal Robots UR5e** manipulators.

The project progresses from a baseline STOMP implementation to robot adaptation, Product of Exponentials (PoE) forward kinematics, challenging obstacle environments, and orientation-constrained motion planning.

## Contents

1. [Project overview](#project-overview)
2. [Software requirements](#software-requirements)
3. [Repository structure](#repository-structure)
4. [Getting started](#getting-started)
5. [Running each task](#running-each-task)
6. [How the STOMP pipeline works](#how-the-stomp-pipeline-works)
7. [Key implementation files](#key-implementation-files)
8. [Validation and recorded results](#validation-and-recorded-results)
9. [Outputs and reproducibility](#outputs-and-reproducibility)
10. [Troubleshooting](#troubleshooting)
11. [Scope and limitations](#scope-and-limitations)

## Project overview

STOMP is a sampling-based trajectory optimization method. Starting with a joint-space trajectory between a specified initial and goal configuration, it samples correlated random perturbations, evaluates the resulting trajectories, and updates the trajectory to reduce an objective combining obstacle-related penalties and smoothness.

The repository contains five related tasks:

| Task | Robot | Objective |
| --- | --- | --- |
| **1 — Baseline STOMP** | Kinova Gen3 (7 DOF) | Implement and visualize the STOMP trajectory planner. |
| **2 — Robot adaptation** | UR5e (6 DOF) | Adapt STOMP to a different robot, including IK, collision geometry and validation. |
| **3 — PoE forward kinematics** | UR5e | Replace repeated built-in forward-kinematics calculations with a Product of Exponentials implementation. |
| **4 — Complex obstacle scenarios** | UR5e | Evaluate planning in pillar, clutter, window and shelf environments, including stopping-rule experiments. |
| **5 — Orientation constraints** | UR5e | Add an end-effector upright-orientation penalty and compare constrained and unconstrained trajectories. |

All motion planning and visualization are performed in MATLAB; no physical robot is required.

## Software requirements

The project was developed and tested with **MATLAB R2026a** on Windows. A compatible MATLAB release may work, but has not been verified here.

Required toolboxes for the core workflows:

- **Robotics System Toolbox** — robot models, inverse kinematics, transforms, robot visualization and collision checks.
- **Statistics and Machine Learning Toolbox** — multivariate Gaussian trajectory sampling (`mvnrnd`).

Other MATLAB functions used include plotting, matrix exponentiation (`expm`), MAT-file saving, CSV export and optional video recording.

To check installed products in the MATLAB Command Window:

```matlab
ver
```

Quick robot-model test:

```matlab
robot = loadrobot('universalUR5e', 'DataFormat', 'column');
showdetails(robot)
show(robot)
```

If `loadrobot` is unavailable, install or enable Robotics System Toolbox. If `mvnrnd` is unavailable, check Statistics and Machine Learning Toolbox.

## Repository structure

```text
EE4705_Project2/
├── Task 1/
│   ├── KINOVA_STOMP_Path_Planning.m
│   ├── Task1_Demo_Video.m
│   ├── helperSTOMP.m
│   ├── stompSamples.m
│   ├── stompTrajCost.m
│   ├── stompObstacleCost.m
│   ├── stompUpdateProb.m
│   ├── stompDTheta.m
│   ├── stompUpdateTheta.m
│   ├── sEDT_3d.m
│   ├── Results/
│   └── Videos/
├── Task_2/
│   ├── README.md
│   ├── UR5e_STOMP_Path_Planning.m
│   ├── stompPlan.m
│   ├── ur5eSolveIK.m
│   ├── ur5eSphereModel.m
│   ├── stompCheckCollision.m
│   ├── Task2_Validation_Test.m
│   ├── Task2_Waypoint_Validation.m
│   └── Results/
├── Task_3/
│   ├── README.md
│   ├── UR5e_PoE_STOMP_Main.m
│   ├── ur5ePoEModel.m
│   ├── ur5eFKinSpace.m
│   ├── updateJointsWorldPosition.m
│   ├── Task3_ValidatePoE_Random.m
│   ├── Task3_ValidatePoELinks.m
│   ├── Task3_ValidateJointPositions.m
│   ├── Task3_FK_Benchmark.m
│   └── Results/
├── Task_4/
│   ├── Task4_UR5e_Scenarios.m
│   ├── Task4_Extras.m
│   ├── helperScenarioUR5e.m
│   ├── buildVoxelWorld.m
│   ├── Results_v1/
│   └── Results_v2/
├── Task_5/
│   ├── Task5_CupUpright.m
│   ├── Task5_Extras.m
│   ├── helperScenarioCup.m
│   ├── stompTrajCost.m
│   ├── eeTiltDeg.m
│   ├── Results5_v2/
│   └── Results_board/
└── README.md
```

The tree highlights entry points and major supporting files; it is not an exhaustive listing. **Preserve the folder names**, particularly `Task 1` (with a space), because later scripts locate shared functions using relative paths.

## Getting started

### 1. Clone the repository

```powershell
git clone https://github.com/savchann/EE4705_Project2.git
cd EE4705_Project2
```

### 2. Open MATLAB

Set MATLAB's **Current Folder** to the task you want to run, or navigate using `cd`:

```matlab
cd('C:\Robotics\EE4705_Project2\Task_2')
```

Replace the example Windows path with your local checkout location. Most main scripts resolve their own directory and add the required shared-function paths automatically.

### 3. Run one task at a time

Each task may contain functions with the same name but different implementations, such as `stompTrajCost.m` and `updateJointsWorldPosition.m`. **Avoid adding all task folders indiscriminately** to the MATLAB path. Start with the appropriate main script and check function resolution if results are unexpected:

```matlab
which stompTrajCost -all
which updateJointsWorldPosition -all
which stompRobotSphere -all
```

The active implementation should correspond to the task being run. If switching between tasks within one MATLAB session, clear cached functions as appropriate or start a fresh session.

## Running each task

### Task 1 — Kinova Gen3 baseline STOMP

Set the Current Folder to `Task 1` and run:

```matlab
KINOVA_STOMP_Path_Planning
```

This loads the Kinova Gen3, defines the environment and initial/goal configurations, optimizes the joint trajectory using STOMP, and visualizes the result.

To generate the separate demonstration video, see:

```matlab
Task1_Demo_Video
```

Existing recordings are available under `Task 1/Videos/`.

### Task 2 — UR5e adaptation and validation

Set the Current Folder to `Task_2` and run:

```matlab
UR5e_STOMP_Path_Planning
```

The main simulation loads the UR5e, solves seeded inverse kinematics for the start and goal, initializes a **39-sphere collision approximation**, runs STOMP, checks the optimized path and plots the cost histories.

For repeated-seed validation:

```matlab
Task2_Validation_Test
```

For waypoint-resolution comparison (10, 20 and 30 waypoints):

```matlab
Task2_Waypoint_Validation
```

These scripts generate CSV/MAT summaries and graphs under `Task_2/Results/` and `Task_2/Results/Waypoint_Validation/`.

**Important:** The shared `stompPlan.m` now supports both `original` and `improved` stopping rules. To reproduce the originally reported Task 2 experiments, explicitly use:

```matlab
opts.stopRule = 'original';
```

The original rule stops on a small cost change (threshold 0.1) or the iteration limit. The improved rule is intended for more challenging cases and also considers sphere-model clearance and cost stabilization. Check the active options before comparing new runs with the saved Task 2 results.

See [Task_2/README.md](Task_2/README.md) for task-specific notes.

### Task 3 — PoE forward kinematics

Set the Current Folder to `Task_3` and run:

```matlab
UR5e_PoE_STOMP_Main
```

Task 3 computes the UR5e's six screw axes and link home transformations, then uses PoE to evaluate moving-link poses during STOMP trajectory-cost calculations. The Task 2 implementation is retained separately for comparison.

Validation scripts:

```matlab
Task3_ValidatePoE
Task3_ValidatePoE_Random
Task3_ValidatePoELinks
Task3_ValidateJointPositions
```

Standalone forward-kinematics performance benchmark:

```matlab
Task3_FK_Benchmark
```

The PoE model is initialized using `ur5ePoEModel.m`, while `ur5eFKinSpace.m` performs the matrix-exponential forward-kinematics calculation. MATLAB `getTransform()` is retained for one-time model extraction and independent numerical checks, rather than repeated link evaluation inside the PoE-STOMP optimization loop.

See [Task_3/README.md](Task_3/README.md) for task-specific notes.

### Task 4 — Challenging collision-avoidance scenarios

Set the Current Folder to `Task_4` and run:

```matlab
Task4_UR5e_Scenarios
```

The script defines four scenarios:

| Scenario | Description |
| --- | --- |
| `pillar` | Plan around a tall pillar. |
| `clutter` | Navigate a cluttered tabletop environment. |
| `window` | Move through a narrow opening in a wall. |
| `shelf` | Navigate between shelf compartments. |

The scenarios are designed with nontrivial initial paths that intersect obstacles. The script checks endpoints, performs planning, evaluates waypoint and dense collision checks, and generates visualizations and summaries. At the top of the script, settings control repeated-seed studies, animation, video recording and optional window-size experiments.

Additional experiments:

```matlab
Task4_Extras
```

These compare the original and improved stopping criteria and investigate the effect of pillar placement. The extra experiments may take substantially longer than a single demonstration run.

The uploaded repository contains recorded outputs in `Task_4/Results_v1/` and `Task_4/Results_v2/`. The current scripts may create a new `Task_4/Results/` folder when run; this is expected.

### Task 5 — Orientation-constrained STOMP

Set the Current Folder to `Task_5` and run:

```matlab
Task5_CupUpright
```

Task 5 extends the trajectory cost with an end-effector orientation penalty. The intended motion is analogous to carrying a cup upright while avoiding obstacles. The implementation aligns the tool's local **y-axis** with the world vertical direction and compares constrained and unconstrained planning.

The main script tests selected scenarios from Task 4 and generates trajectory overlays, tilt-angle plots and comparisons with and without the orientation penalty. Its `qcWeight` parameter controls the orientation-cost weighting (the main script uses `10`; `0` disables the constraint).

Additional experiments:

```matlab
Task5_Extras
```

These investigate constraint-weight selection, scenario difficulty and alternative trajectory initialization. They can require substantially more runtime than the main demonstration.

**Path caution:** Task 5 has its own `stompTrajCost.m`, which adds the orientation term. The main script checks that this version is active. Do not replace it with the Task 2 cost function when running Task 5.

The uploaded repository contains recorded results under `Task_5/Results5_v2/` and `Task_5/Results_board/`. The scripts may create `Task_5/Results/` for new outputs.

## How the STOMP pipeline works

The core planning sequence is:

```text
Robot model + start/goal poses + obstacle geometry
                       |
                       v
           Initial joint-space trajectory
                       |
                       v
       Correlated Gaussian trajectory sampling
                       |
                       v
        Evaluate obstacle / constraint costs
                       |
                       v
         Convert sample costs to probabilities
                       |
                       v
       Compute weighted perturbation update
                       |
                       v
         Smooth and update the trajectory
                       |
                       v
       Check convergence / stopping criterion
                       |
             repeat or finish
                       |
                       v
         Validate collisions and visualize
```

The planner uses a joint-space trajectory matrix with one column per waypoint and one row per moving joint. Its principal objective combines a weighted obstacle cost and a smoothness penalty; Task 5 adds an orientation-constraint cost.

The 39-sphere UR5e approximation supports repeated obstacle-cost evaluation. The separate collision-validation routine checks the planned trajectory using MATLAB's robot collision geometry at discrete and interpolated configurations. These checks are stronger than inspecting only the nominal waypoints, but they do **not** mathematically guarantee continuous collision-free motion.

### Stopping criteria

The shared Task 2 planner provides two modes:

- **`original`**: stop when the absolute cost change falls below `convergenceThreshold` (typically `0.1`), when the update is negligible, or at the maximum iteration count (typically `50`).
- **`improved`**: uses sphere-model clearance together with cost stabilization, with additional stall detection and a higher default iteration cap (typically `150`). This mode is used for the harder Task 4/5 experiments.

An optimizer stopping is not itself proof of a collision-free trajectory; the post-planning collision checks must still be inspected.

## Key implementation files

| File | Role |
| --- | --- |
| `Task 1/stompSamples.m` | Generate correlated random trajectory perturbations. |
| `Task 1/stompUpdateProb.m` | Convert sampled costs into probability weights. |
| `Task 1/stompDTheta.m` | Calculate the probability-weighted trajectory perturbation. |
| `Task 1/stompUpdateTheta.m` | Smooth and apply trajectory updates. |
| `Task_2/stompPlan.m` | Function-based optimization loop and stopping criteria. |
| `Task_2/stompTrajCost.m` | UR5e obstacle and smoothness cost evaluation. |
| `Task_2/ur5eSolveIK.m` | Seeded inverse kinematics for start and goal poses. |
| `Task_2/ur5eSphereModel.m` | Construct the 39-sphere collision representation. |
| `Task_2/stompCheckCollision.m` | Validate planned configurations against collision geometry. |
| `Task_3/ur5ePoEModel.m` | Derive and store screw axes and home transformations. |
| `Task_3/ur5eFKinSpace.m` | Compute forward kinematics using matrix exponentials. |
| `Task_3/updateJointsWorldPosition.m` | Evaluate moving-link poses with PoE. |
| `Task_4/helperScenarioUR5e.m` | Define challenging obstacle scenarios. |
| `Task_4/buildVoxelWorld.m` | Build obstacle representations and voxel distance fields. |
| `Task_5/stompTrajCost.m` | Add the orientation penalty to trajectory cost. |
| `Task_5/eeTiltDeg.m` | Measure end-effector tilt along a trajectory. |

## Validation and recorded results

The following figures summarize **recorded Task 2 and Task 3 experiments**. They describe particular test conditions, not universal guarantees or independently reproduced results on every MATLAB installation.

### Task 2 — UR5e single-scenario result

For the recorded baseline UR5e scenario, the initial trajectory intersected obstacles at **7 of 20 waypoints**. An optimized run achieved **0 collisions across 191 interpolated configurations**, with approximately **38 mm minimum clearance** according to the implemented checks. The total cost decreased from approximately **178.7 to 43.3**.

### Task 2 — Random-seed validation

Ten random seeds were tested with 20 waypoints and 20 sampled trajectories per iteration:

| Metric | Recorded result |
| --- | ---: |
| Collision-free trials | 10 / 10 |
| Mean optimization runtime | 3.00 s |
| Mean iterations | 9.4 |
| Lowest recorded clearance | 26.4 mm |

### Task 2 — Waypoint-resolution experiment

| Waypoints | Observed success | Mean runtime | Mean iterations |
| --- | ---: | ---: | ---: |
| 10 | 90% | 1.23 s | 7.6 |
| 20 | 100% | 3.07 s | 9.4 |
| 30 | 100% | 5.59 s | 11.7 |

Twenty waypoints provided a practical trade-off in the tested scenario. Results may differ under other obstacles, seeds or stopping criteria.

### Task 3 — PoE accuracy and FK benchmark

The PoE implementation was compared with MATLAB's built-in forward kinematics across random joint configurations. A 1,000-configuration, five-repeat benchmark reported:

| Metric | Recorded result |
| --- | ---: |
| Mean runtime, `getTransform()` | 0.7835 s |
| Mean runtime, PoE | 0.2472 s |
| Relative FK speedup | 3.17× |
| Maximum position error | 1.343 × 10⁻¹⁵ m |
| Maximum rotation-matrix error (Frobenius norm) | 1.558 × 10⁻¹⁵ |

This is a **forward-kinematics microbenchmark**, not a controlled measurement of total STOMP speedup. One-time PoE model initialization was excluded from the timed evaluation.

A separate recorded PoE-STOMP run passed collision checks at 0/191 interpolated configurations. If reproducing published numerical results, use the same random seed, solver settings, cost-function version and stopping rule; the shared planner has since gained an improved stopping option.

### Task 4 and Task 5 outputs

The repository includes saved `.mat`, `.csv` and `.png` outputs for the complex obstacle and orientation-constraint experiments. Consult the corresponding task scripts and result summaries for scenario-specific measurements; this README does not claim a single aggregate success rate across these differing environments.

## Outputs and reproducibility

Typical generated files:

| Extension | Contents |
| --- | --- |
| `.mat` | MATLAB trajectories, optimization histories and validation structures. |
| `.csv` | Trial-level and summary numerical results. |
| `.png` | Static convergence, trajectory and comparison figures. |
| `.fig` | Editable MATLAB figures. |
| `.mp4` | Optional robot-motion demonstrations. |

Several scripts use `rng(..., 'twister')` for reproducible stochastic sampling. Exact runtime depends on hardware, MATLAB release, path resolution and which stopping rule is selected.

For reproducibility, record the random seed, robot model, obstacle scenario, number of waypoints, sampled paths per iteration, cost weights, stopping rule and iteration limit. Preserve the source code corresponding to each saved result.

## Troubleshooting

**`Unrecognized function or variable 'loadrobot'`**  
Install or enable Robotics System Toolbox and verify with `ver`.

**`mvnrnd` is unavailable**  
Check that Statistics and Machine Learning Toolbox is installed.

**`Too many input arguments` in `stompRobotSphere`**  
MATLAB may be calling the Task 1 version instead of the UR5e-specific function. Run `which stompRobotSphere -all` and correct the path order.

**Unexpected PoE or built-in FK implementation**  
Run `which updateJointsWorldPosition -all`. Task 3 must use its PoE version; Task 2 must use its built-in FK version.

**Wrong trajectory cost function in Task 5**  
Run `which stompTrajCost -all`. Task 5's constraint-aware function must be first.

**`inverseKinematics` System object reports an input data-type change**  
The cached IK solver may be locked to an earlier input type. Clear `ur5eSolveIK` or use a fresh solver with consistent numeric and string input types.

**Planner stops but the path still collides**  
Stopping criteria and collision verification are separate. Inspect the reported stop reason and dense collision check; consider the improved stopping rule and alternative initialization for difficult scenarios.

**A script cannot locate Task 1 or Task 2**  
Keep all task folders together under the same repository root and preserve the original names.

## Scope and limitations

- The experiments use predefined robot and obstacle models in MATLAB; they do not implement real-time perception or physical-robot control.
- Sphere-based collision costs approximate robot geometry; separate mesh-based collision checks are used for validation where provided.
- Dense interpolation checks sample additional configurations but are not formal continuous-collision proofs.
- STOMP is stochastic and can converge to different local solutions for different seeds or initial trajectories.
- Runtime comparisons are hardware- and implementation-dependent. The PoE FK benchmark must not be interpreted as a whole-planner speedup.
- Saved experiment results should be associated with the exact code and stopping criteria used to produce them.

---

**Suggested reading order:** Start with Task 1 for the STOMP algorithm, Task 2 for the UR5e integration, Task 3 for PoE, then Tasks 4 and 5 for the more advanced planning scenarios and constraints.
