# Simulation

`py -m robotpy sim` (or the VS Code task *PyFRC: Run current project in the
PyFRC simulator*) runs the real robot code against simulated hardware:

- **Drivetrain**: maple-sim's swerve physics, ported to Python (maple-sim
  itself is Java-only). Each Kraken is modelled from the voltage Phoenix
  commands. Wheels have a grip limit, so they can skid; when they do, the
  encoders over-count, as they would on carpet. The gyro drifts like a Pigeon 2.
  The robot collides with the walls, hubs, trench walls and tower uprights,
  and drives over the bumps.
- **Mechanisms**: the flywheel, accelerator, hood, indexer, intake rollers and
  intake pivot spin up, spin down and travel at motor speed. Every
  `get_*_speed()` / `get_position()` reads a simulated encoder; none returns
  the commanded value.
- **Fuel**: every ball on the field is simulated in 3D: rolling, bouncing,
  pushed by the bumpers, colliding with each other, and scored in the hubs.
  The intake only collects when its pivot is down and its rollers are pulling
  in. Shots leave at the speed and angle the *simulated* flywheel and hood
  give them, so a slow flywheel shoots short. Shots fly with the same drag and
  backspin model `shot_calc.py` used to build the shooting table.
- **Vision**: the four AprilTag cameras are simulated by PhotonVision at
  their real mounting transforms, with pixel noise and latency. The pose
  estimator fuses them exactly as on the robot.
- **Battery**: sags under load, using maple-sim's battery model.

The robot code knows nothing about this except one line in `hopper.py`:
submitting an auto in simulation puts the robot on its starting spot and resets
the field, the job a human does on the real field.

| File | What it simulates |
|---|---|
| `physics.py` | Entry point that `robotpy sim` loads |
| `simulation/arena.py` | Steps everything once per loop; NetworkTables output |
| `simulation/swerve_sim.py` | Swerve modules, chassis friction, gyro, collisions (maple-sim port) |
| `simulation/motor_sim.py` | Phoenix Talon → DC motor model; current limits are read from each Talon's config |
| `simulation/mechanisms.py` | Shooter, indexer, intake; fuel feeding and launching |
| `simulation/fuel_sim.py` | Balls, field geometry for balls, hub scoring |
| `simulation/vision_sim.py` | PhotonVision cameras |
| `simulation/field.py` | Field obstacles for the robot (maple-sim 2026 map) |

## Running an auto in the simulator

1. `py -m robotpy sim` from the `robot` folder.
2. In the sim GUI: drag a controller (or *Keyboard 0*) from **System
   Joysticks** onto **Joysticks** slot 0. Use slot 1 for driver 2.
3. Choose the auto and alliance:
   - Set the alliance in the sim GUI's **FMS** panel (Blue 1 / Red 1).
   - Open Elastic (or the sim GUI's NetworkTables view) on `localhost`.
   - Pick the auto in **Auto Chooser**.
   - Set **Submit Auto? (and FMS Connected)** to `1`. The robot jumps to that
     auto's starting spot, with a fresh field and 8 balls preloaded.
4. Click **Autonomous**, then **Enable**, in the sim GUI's Robot State panel.
5. To run it again, restart the sim (the auto can only be submitted once per
   run).

### Without the GUI

```
py tests/sim_auto.py 6 blue 20
```

This runs auto 6 (the chooser's value) on blue for 20 s. It prints the robot's
position, hopper count, shots fired and scored, and pose-estimate error every
second. At the end it prints shooting accuracy and loop timing. It runs in real
time, so you can watch it in AdvantageScope while it runs.

Watch the **loop period** line at the end. It must stay near 20 ms. If the
robot code or the physics gets slow, PathPlanner's timers run ahead of a robot
that is updated less often, and paths look much worse than they really are.

## AdvantageScope

1. Start the simulator, then in AdvantageScope choose **File → Connect to
   Simulator**. Use plain NetworkTables (NT4), not AdvantageKit: the PyKit
   logger isn't fed each loop, so `/AdvantageKit` stays empty.
2. **File → Import Layout** and pick `robot/advantagescope-sim-layout.json`.
   It's the team's layout with the 3D field wired to the topics below. It
   needs the **Jonez** robot model in AdvantageScope's custom assets folder:
   **Preferences → Custom Assets** must point at the folder that *contains*
   `Robot_Jonez`, not at `Robot_Jonez` itself.

   To set the 3D field up by hand instead, open a **3D Field** tab, pick the
   2026 field, and drag these topics from the sidebar onto it:

   | Topic | Show as | What it is |
   |---|---|---|
   | `Sim/Robot Pose` | Robot | Where the simulated robot really is |
   | `Robot/Estimated Pose` | Ghost | Where the robot code thinks it is (odometry + vision) |
   | `Robot/Component Poses` | Component, dropped onto the Robot | All 28 parts of the Jonez model, in order: hood, inner and outer intake arms, the 24 hopper balls (hidden when empty), climber |
   | `FuelSimulation/Fuels` | Game Piece (Fuel) | Every ball on the field |
   | `Vision/Seen Tags` | Vision Target, dropped onto the Robot | Tags the cameras used this loop |

   The gap between the robot and the ghost is your pose-estimation error.
   When replaying a real match log there's no `Sim/Robot Pose`; use
   `Robot/Estimated Pose` as the robot instead.
3. In a **Swerve** tab, add `Robot/Module States` (measured) and
   `Robot/Module Setpoints` (commanded), with `Robot/Estimated Pose` as the
   robot rotation. `Sim/Module States` shows what the wheels really did.
   During skids, measured differs from real.
4. A **Line Graph** tab for shooter tuning:
   - `Sim/Flywheel RPS` against `SmartDashboard/Shooter/Commanded Fly Speed`
   - `Sim/Hood Rotations`, `Sim/Launch Speed`, `Sim/Launch Angle`
   - `Sim/Shots Fired`, `Sim/Shots Scored`, `Sim/Hopper Count`
   - `Sim/Battery Voltage`
5. **File → Export Layout** saves the tabs. Import the layout next time.

Every topic is also recorded in the `.wpilog` that `DataLogManager` writes to
`robot/logs`. Open one in AdvantageScope to replay a run. `Robot/*` and
`Vision/*` are also published on the real robot, so match logs replay the same
way.

## What to tune

The numbers that aren't in the robot code are estimates. They're all at the
top of their files:

- `simulation/mechanisms.py`:
  - Flywheel and other inertias, intake pivot gearing.
  - Feed rate: 8 balls/s at full indexer speed.
  - Shot dispersion.
  - Where the ball leaves the robot.

  Compare against a real robot log (spin-up time, balls per second) and adjust.
- `simulation/vision_sim.py`: camera resolution, FPS, latency and noise.
  `ENABLE_VISION` in `arena.py` turns vision off, to see pure odometry drift.
- `simulation/field.py`: `BUMPS_ARE_WALLS` (maple-sim's default) makes the
  bumps impassable.
- Robot mass, MOI, bumper size and wheel COF come from
  `deploy/pathplanner/settings.json`. Keep them accurate there and the paths
  and the sim both use them.

## Things the simulator turned up

- **Red autos are off by about 1 m with the installed PathPlanner.** This
  computer has `robotpy-pathplannerlib` 2025.2.1. Its `FlippingUtil` mirrors
  red paths using the 2025 field size (17.548 m long). The robot code flips
  start poses with the 2026 size (16.541 m), so on red the path starts 1 m from
  the robot. `py -m robotpy sync` installs the 2026 libraries that
  `pyproject.toml` asks for. Check which version is on the roboRIO, too.
  With the 2026 field size, the red autos run exactly like the blue ones.
- **"Shoot until empty" steps always run to their timeout.** Nothing on the
  robot counts fuel, so `robot.fuel_in_hopper` stays at 8.
  `drive_to_zone_trench` and friends wait for the full `withTimeout`. The
  simulator counts fuel separately and leaves `fuel_in_hopper` alone, so
  simulated autos take as long as real ones. The old simulator emptied it and
  moved on early.
- **`Shooter.ready_to_shoot` doesn't check flywheel speed.**
  `abs(abs(fly) - cmd <= 3)` takes `abs()` of a boolean, so it's true whenever
  the flywheel is *below* its target. Aimed shots hide it: by the time the
  robot has turned to the hub (about a second), the flywheel is up to speed.
  A manual ("down bad") shot skips aiming and can feed into a slow flywheel.

## Limits

- Bumps are flat floor for the robot: it doesn't slow down or tip on them.
  Balls do roll on the bump ramps.
- Clearing a jam doesn't spit balls back out.
- Balls don't push back on the robot.
- The outposts and human player aren't simulated.
