# Task 5 – Keep the end-effector y-axis upright ("carry a cup")

## The constraint

`stompTrajCost.m` in this folder adds a constraint cost at every waypoint:

```
q_c(i) = w_c * || y_ee(i) - [0;0;1] ||_1        (w_c = 10)
S(i)   = 1000 * q_o(i) + q_c(i)
```

- `y_ee(i)` is the end-effector (`tool0`) y-axis in world coordinates: the 2nd column of the rotation from the forward kinematics.
- This is option 2 of the brief's technical tips, the l1 norm of the residual.
- It is zero only when the cup is upright, and rises with tilt: ≈0.19 at 10°, 0.63 at 30°, 2 at 90°.
- Rotation about the vertical is free, so the planner keeps that freedom for avoiding obstacles.

The start and goal orientations are changed to the "cup" pose (tool z horizontal, tool y up). Start and goal are fixed, not planned, so they have to satisfy the constraint themselves; the brief points this out.

`w_c = 10` was tuned in simulation. With it, a 30° tilt costs about as much as a few collision spheres sitting 1 cm inside an obstacle. The weight sweep shows what happens if it is lower or higher.

## How to run

Needs `Task 1`, `Task_2` and `Task_4` next to this folder. Run `Task5_CupUpright.m`.

This folder is added to the path last, so its `stompTrajCost.m` overrides the Task 2 one. The script checks this with `which`.

Settings at the top:

| Setting | Default | Meaning |
|---|---|---|
| `qcWeight` | 10 | constraint weight |
| `runSeedStudy`, `nTrials` | true, 5 | success rate and tilt over seeds 1–5 |
| `runWeightSweep`, `sweepWeights` | true, [0 2 5 10 20 50] | trade-off curve on `board_low` |

The full script runs 66 planning runs, so allow ~20 min. Switch the studies off for a quick run.

Results go to `Task_5/Results`:
- overlay PNGs with the cup axis drawn at every waypoint, without and with the constraint
- a tilt-along-the-path PNG per scenario
- `Task5_summary.csv`
- `Task5_weight_sweep.png`

## Files

| File | What it does |
|---|---|
| `Task5_CupUpright.m` | Main script: without vs with constraint, seed study, weight sweep |
| `stompTrajCost.m` | Cost with the orientation constraint |
| `helperScenarioCup.m` | Task 5 scenarios |
| `eeTiltDeg.m` | Cup tilt (angle of tool y from vertical) along the path, evaluated densely |

## Scenarios and expected results (Python re-implementation, 8 seeds)

| Scenario | Without constraint | With constraint (w_c = 10) | Story |
|---|---|---|---|
| **window** (Task 4) | 3/8 collision-free, median max tilt **18°** | 3/8, median max tilt **4°** | Constraint works and does not cost success |
| **board_low**: carry the cup over a low board (top at 0.25 m) | 7/8, median max tilt **38°**: lifts over the board but tips the cup, would spill | 6/8, median max tilt **5°** | Clearest before/after |
| **board_high**: same, board 8 cm higher | 4/8, but tilting ~38° | **0/8**, tilt ~2° | **Failure case.** With the constraint, STOMP tries to slide the cup *under* the board (staying low and level is cheap for the constraint) and never finds the lift-over route. The constraint creates a local minimum |

Expect the same trends in MATLAB, not the same numbers (different random generator; mesh-based collision check).

## Points for the presentation

1. **A soft constraint, not a hard one.** It is a penalty, so the cup can still tilt a few degrees, and obstacles and the constraint are traded off through `w_c`. The weight sweep shows the trade-off: higher `w_c` means more upright but fewer collision-free paths.
2. **Constraints change the cost landscape.** `board_high` is the clearest demo. The unconstrained planner escapes by tilting; the constrained one gets stuck in the "go under" local minimum. A global planner, or a better initial path (e.g. a lift-over waypoint), would be needed. In simulation, 5 of 8 constrained `board_high` runs even ended with a *higher* total cost than the straight-line start. STOMP returns the last path, not the best one seen, so keeping the best-so-far path would be an easy improvement.
3. **Only waypoints are constrained.** The cost is evaluated at 20 waypoints. `eeTiltDeg` measures tilt densely, so you can show the tilt between waypoints too.
4. **The l1 residual is not exactly an angle.** It depends on the tilt direction and peaks near 135°. A cleaner alternative is `1 - y_ee(3)` (= 1 − cos θ) or the angle `acos(y_ee(3))` itself. Option 1 of the brief, a full target rotation, would also fix the rotation about the vertical, which is stricter and leaves less freedom for avoiding obstacles.
