# Task 2 – UR5e port

Plans a collision-free path for a **Universal Robots UR5e** in the same kind of scenario as Task 1 (one box between start and goal). The STOMP algorithm itself is unchanged from Task 1.

## How to run

Folder layout expected (folders side by side):

```
project_2/
  Task 1/   (teammate's completed STOMP code, unchanged)
  Task_2/   (this folder)
  Task_4/
  Task_5/
```

Run `UR5e_STOMP_Path_Planning.m`. It adds `Task 1` and `Task_2` to the MATLAB path itself.
Run `checkSphereModelUR5e.m` once to see the collision spheres drawn over the robot (good slide figure).

Files used unchanged from `Task 1`: `sEDT_3d.m`, `stompSamples.m`, `stompDTheta.m`, `stompUpdateProb.m`, `stompUpdateTheta.m`, `stompObstacleCost.m`.

> Running Tasks 2/4/5 puts their folders on the MATLAB path. Before re-running the Kinova Task 1 code in the same session, run `restoredefaultpath` (or run Task 1 from inside its own folder), otherwise MATLAB may pick up the UR5e versions of shared file names.

## Files

| File | What it does |
|---|---|
| `UR5e_STOMP_Path_Planning.m` | Main script (UR5e version of `KINOVA_STOMP_Path_Planning.m`) |
| `helperSTOMP.m` | Runs the planner, checks the initial and final paths for collisions, animation, plots, saves results |
| `stompPlan.m` | The Task 1 STOMP loop as a function (same algorithm and parameters), so Tasks 4/5 can call it many times |
| `updateJointsWorldPosition.m` | Forward kinematics with `getTransform`; handles fixed bodies; optional end-effector pose |
| `ur5eSphereModel.m` | Collision spheres that follow the real UR5e link geometry |
| `stompRobotSphere.m` | Places the spheres using the current link poses |
| `stompTrajCost.m` | Same cost as Task 1, using the new sphere model |
| `ur5eSolveIK.m` | IK with an explicit seed, plus a check of the reached pose |
| `stompCheckCollision.m` | Mesh-based collision check at waypoints and densely in between |
| `helperCreateObstaclesUR5e.m` | Single box obstacle, moved for the UR5e |
| `helperInitialVisualizerUR5e.m` | Start-pose figure |
| `checkSphereModelUR5e.m` | Draws the sphere model over the robot meshes |

## What had to change for the UR5e, and why

1. **Robot and end-effector:** `loadrobot('universalUR5e')`, end-effector `tool0`.
2. **Joint ↔ body mapping:** the UR5e tree contains fixed bodies (e.g. `base_link_inertia`, `flange`, `tool0`). So joint *k* is **not** `BodyNames{k}`. `updateJointsWorldPosition` finds the moving bodies once and caches them. The Task 1 version re-scanned `robot.Bodies{k}` on every call, which the brief notes is slow; the function runs 20 × 20 × 6 times per iteration.
3. **Collision spheres:** the Kinova code puts spheres on straight lines between joint origins. On the UR5e the real links are offset sideways from the joint axes; the upper arm sits ~14 cm to the side. Depending on how the URDF places its joint frames, a joint-to-joint chain misses part of the arm. `checkSphereModelUR5e.m` draws that chain in red so you can see how far off it is in your MATLAB model. `ur5eSphereModel` instead defines 7 segments on the physical arm (shoulder, upper arm, elbow, forearm, three wrist housings) at the zero pose. It converts them once into each link's frame and moves them with the link. That gives 39 spheres, radius 4–6.5 cm. Check the coverage with `checkSphereModelUR5e.m`.
4. **IK branch:** the UR5e has up to 8 IK solutions per pose. Starting the solver from the home configuration can return an elbow-down or flipped-wrist solution, and the straight-line initial path then swings through odd places. Start and goal IK are seeded on the elbow-up branch.
5. **Poses and obstacle:** the start (0.45, −0.10, 0.20) and goal (0.40, 0.55, 0.30) have the gripper pointing down. The box is moved to (0.45, 0.25, 0.25) so that the goal itself is collision-free and the straight-line path collides (7 of 20 waypoints).
6. **Workspace:** the UR5e reaches ~0.85 m, so the Task 1 voxel world (2 m cube, 2 cm voxels) still fits; no change needed.

## Expected result

In a Python re-implementation of this exact setup (same STOMP parameters and costs), the planner found a collision-free path for 8/8 random seeds in about 10 iterations. MATLAB uses a different random generator and checks collisions with the real meshes, so iteration counts will differ.

## Note on forward kinematics

This uses `getTransform`. Task 3 replaces it with PoE. If the PoE version keeps the same outputs `[X, T, Tee]`, it drops into this file without changing anything else.
