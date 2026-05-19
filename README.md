# arm-smoothness-testing

A small Go program that drives a Viam-connected robot arm back and forth between
two Cartesian points and measures how smoothly it moves. Used to compare motion
strategies (motion-service straight-line vs. per-step IK + `MoveToJointPositions`)
across different arms (UR20, xArm6).

## What it does

1. Connects to a Viam machine using a hard-coded preset (machine address,
   API key, arm component name, point A, point B, end-effector orientation).
2. Moves the arm between point A and point B for `-n` cycles using one of two
   modes (`-mode`):
   - **`linear`** — uses the Viam motion service `Move` with a `LinearConstraint`
     (2 mm line tolerance, 5° orientation tolerance) so the arm tracks a single
     straight Cartesian segment from A to B.
   - **`waypoints`** — pre-plans the leg as N intermediate Cartesian waypoints
     spaced `-spacing-mm` apart, calls `PlanMotion` per waypoint (seeded by the
     previous solution) to get joint configs, then streams them to the arm with
     `MoveToJointPositions`.
3. In `waypoints` mode, writes per-`MoveToJointPositions` timing rows to
   `timings.csv` and shells out to `plot_timings.py` to render `timings.png`.

## Arm presets

Defined in `armPresets` at the top of `main.go`:

| Preset | Machine | Notes |
|--------|---------|-------|
| `ur20` | `viam-lab-sanding-25-1-main…` | UR20 |
| `xarm` | `orbital-sanding-with-sanding-module-main…` | xArm6. Adjacent-link collision meshes in the JSON kinematics permanently overlap, so this preset declares `allowedSelfCollisions` between every consecutive link pair; otherwise the planner rejects every IK solution. |

Each preset hard-codes point A, point B, and an `OrientationVectorDegrees` shared
by both endpoints (so the EE orientation is held constant along the leg).

## Usage

```bash
go run . [flags]
