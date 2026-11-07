# Task 4 – Custom collision-avoidance scenarios (UR5e)

Four scenarios, each with a straight-line initial path that collides (no trivial initialisation). Each is run once reproducibly (`rng(42)`) and then with several random seeds to get a success rate.

## How to run

Needs `Task 1` and `Task_2` next to this folder. Run `Task4_UR5e_Scenarios.m`.
Settings at the top:

| Setting | Default | Meaning |
|---|---|---|
| `scenarioNames` | all four | which scenarios to run |
| `runSeedStudy`, `nTrials` | true, 5 | repeat each scenario with seeds 1–5 and report the success rate |
| `runWindowSweep`, `windowSizes` | true, [0.34 0.30 0.27] | success rate vs window size |
| `animate`, `saveVideo` | false | play / record each planned motion |

Results go to `Task_4/Results`:
- for each scenario: an overlay PNG, a convergence PNG and a `.mat` file
- `Task4_summary.csv`
- `Task4_window_sweep.png`

Each planning run takes roughly 10–20 s (estimate). The full script runs 39 planning runs, so allow ~10 min. Switch off the studies for a quick run.

## Files

| File | What it does |
|---|---|
| `Task4_UR5e_Scenarios.m` | Main script |
| `helperScenarioUR5e.m` | Scenario definitions: obstacles, start/goal poses, IK seeds |
| `buildVoxelWorld.m` | Builds collision objects **and** the voxel SDF from one shape list (boxes with yaw, spheres, cylinders, any number) |
| `plotTrajectoryOverlay.m` | One-image overlay of the motion: faded robot copies, planned vs initial end-effector paths |

`buildVoxelWorld` exists because the Task 1/2 obstacle script handles exactly one axis-aligned cube. It also keeps the two obstacle representations in sync:
- `world` (collision objects, used for checking and plotting)
- the voxel distance field (what STOMP actually sees)

If you add an obstacle to only one of them, it is drawn but ignored by the planner, or the reverse.

## Scenarios

All have a table under the robot (top 1 cm below the base). These are the results from my Python re-implementation of the same planner, over 8 seeds. Your MATLAB numbers will differ; see the caveats below.

| Scenario | Setup | Initial path colliding waypoints | Success (Python, 8 seeds) | Why it is hard |
|---|---|---|---|---|
| **pillar** | 10×10×90 cm pillar 0.55 m in front; move from one side to the other, gripper down | 8/20 | 3/8 | Needs a big change of arm shape (pull in, pass between base and pillar). Outcome depends on the random samples |
| **clutter** | divider, ball, bottle, crate, can, overhead bar; top-down pick and place | 16/20 | 5/8 (+1 within 5 mm) | Many obstacles at once; tight clearances; the overhead bar blocks "lift over everything" |
| **window** | wall with a 30 cm window; reach through to a point behind the wall | 12/20 | 3/8 (27 cm window: 1/8) | Narrow passage: few random samples land inside the opening |
| **shelf** | two compartments; move from inside the lower one to inside the upper one | 13/20 | 2/8 | The good path is a long detour (pull out, up, push in), far from the straight-line start |

## Failures and shortcomings to discuss

These come straight out of the results and are worth a slide:

1. **Local optimiser, stochastic outcome.** STOMP only improves the current path using noise around it. When the good solution is a large detour (shelf, pillar), most seeds never find it. One successful run is not evidence the scenario is "solved"; that is why the success rate is reported.
2. **Narrow passages.** In the window sweep, success drops sharply as the window shrinks from 30 to 27 cm. The sampling noise is the same everywhere, so the chance that sampled paths pass through the opening falls quickly.
3. **Premature stopping.** The stop rule is "|Q change| < 0.1 between two iterations" (Task 1). Occasionally a run stops after ≤5 iterations while still in collision, because one iteration happened to change the cost very little (1 of 8 shelf runs in simulation). Check the iteration counts in `Task4_summary.csv` and the seed study output.
4. **No best-so-far tracking.** The update is not guaranteed to lower the cost, and the returned path is the last one, not the best one seen. This did not hurt in these four scenarios in simulation, but it does in Task 5's `board_high` case.
5. **Spheres vs meshes; waypoints vs continuous motion.**
   - STOMP sees the robot as spheres and the obstacles as 2 cm voxels; the verification uses the real meshes.
   - The cost is only evaluated at 20 waypoints. The dense check between waypoints sometimes finds collisions the planner could not see.
   - Clutter often ends with near-misses (0–5 mm penetration): the 5 cm safety margin is traded against smoothness.
6. **Obstacle cost × speed.** The obstacle cost is multiplied by the speed of each sphere. Near the start and goal, where the arm moves slowly, being close to an obstacle is cheap.

## Caveats

I could not run MATLAB while writing this. The scenarios were designed and tuned in a Python re-implementation of the same STOMP (same parameters, cost and update rules) with the same UR5e kinematics and sphere model. MATLAB will differ in:
- the random numbers (`rng(42)` in MATLAB ≠ Python)
- collision checking against the real meshes instead of my sphere approximation

So expect similar behaviour (which scenarios are easy or hard), not identical numbers. Report what MATLAB gives you.
