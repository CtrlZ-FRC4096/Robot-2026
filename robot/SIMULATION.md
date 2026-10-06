# Robot simulation

Run `python -m robotpy sim` from the `robot` directory. Use the existing Driver
Station controls, auto chooser, and **Submit Auto? (and FMS Connected)** entry.
The coroutine flags and subsystem state machine still command the robot.

RobotPy discovers `PhysicsEngine` in `physics.py` and calls its `update_sim`
after the normal robot loop and command scheduler. This advances
`RobotSimulation` using elapsed simulation time, with four motor integration
steps. Delayed loops no longer advance just 20 ms while match time runs ahead.
Fuel collision candidates are cached by 3D grid cell and tested in NumPy.
Settled fuel skips repeated static field checks and duplicate display updates.
Steering inertia is 0.008 kg m².
The robot's public `simulationPeriodic()` method is not a RobotPy robot lifecycle
callback; adding a method with that name alone does not run physics.
Calling drive or changing a mechanism
setpoint does not advance time. Dashboard logging only displays the simulation.

## Motors and feedback

Phoenix controllers, including followers, generate output voltages using the
existing control requests and gains. WPILib `DCMotorSim` plants model each drive
and steering motor, the three flywheel motors, accelerator, hood, intake rollers,
intake pivot, and indexer. Plants include inertia, gearing, back EMF, battery
voltage limits, and hood/pivot travel stops. Rotor position and velocity are fed
back through Phoenix simulation APIs. Steering and pivot CANcoders receive
mechanism positions, with their configured offsets included.

Mechanism getters read these sensors instead of returning their setpoints.
Spin-up and deployment therefore take time, and readiness checks see actual
feedback. The flywheel readiness comparison also now tests absolute speed error;
its previous parentheses applied `abs` to a boolean and accepted an underspeed
flywheel. Battery voltage sags with estimated electrical power draw. Disabling
the robot removes motor drive, stops fuel collection/firing, and lets the plants
slow down.

The drivetrain integrates motion from the simulated wheel states. Its physical
pose (`robot.sim.pose`) is separate from `poseEstimator.curEstPose`, which is
updated by the usual wheel/gyro odometry and vision measurements. Fuel contacts,
launch locations, and camera visibility use the physical pose. Auto selection
and `poseEstimator.reset_pose(pose)` explicitly place the simulated robot at a
starting pose; `set_yaw` changes the gyro reference without teleporting it.
Only an explicit starting-pose reset links the physical and estimated pose.

## Estimated pose and ghost robot

Use **`/SmartDashboard/Sim/Estimated Pose`** for the ghost in your existing
viewer. Like `RobotPose`, it is a number array `[x metres, y metres, heading
degrees]`. **`/SmartDashboard/Sim/Estimated Pose2d`** publishes the same result
as a native WPILib `Pose2d` struct for tools such as AdvantageScope. The dashboard
`Field` also contains an `Estimated Pose` object. In simulation, `RobotPose`
now displays the physical robot, so the ghost's error is visible beside it.
Autonomous controllers continue to use the estimated pose, as on hardware.

The data flow is:

1. Controller voltages drive physical motor plants and the physical robot pose.
2. Encoder readings have wheel-radius scale errors (0.8–1.4%). Gyro readings
   have 0.015 degrees/second drift and 0.03 degrees standard deviation noise.
3. PhotonVision projects field tags from the **physical** pose history and
   camera mounts, with latency and noisy image corners. Its solvePNP output
   includes geometry-dependent ambiguity and visibility.
4. The existing camera wrapper uses the same single-tag calculations and
   rejects distances over 5 m or ambiguity over 0.3. Accepted tag poses are
   averaged per camera exactly as on hardware.
5. The existing `SwerveDrive4PoseEstimator` fuses those timestamped observations
   with reported encoders and gyro, using the same 0.03 m XY and 1000 rad heading
   vision standard deviations. That result drives both controls and the ghost.

The estimator never supplies the scene used to generate camera frames. Its
previous heading remains part of your real single-tag reconstruction algorithm;
its estimated translation is not used to synthesize observations. There is no
second filter with artificially tuned weights and no correction copied from
ground truth. `Sim/Odometry Pose` shows the same sensors integrated without
vision, making drift during coverage gaps easier to compare.

Simulation vision is processed before auto submission too, so you can inspect
coverage in teleop. The submission gate on real hardware is unchanged.

## Cameras and fuel

The four configured AprilTag cameras use the existing names, mounting transforms,
and 2026 welded field layout. PhotonVision simulation supplies observations to
the existing `PhotonCamera` readers and vision fusion code. Initial camera
parameters are 960 x 720, 90 degree diagonal FOV, 30 FPS, 35 ms average latency
with 5 ms jitter, and pixel measurement noise. Tags must be in view and within
5 m. Python PhotonLib supplies target data; it does not render video streams.
The real camera transforms and ambiguity rejection are retained.

Fuel collection requires a deployed, spinning intake. Shooting requires moving
flywheels, accelerator, and indexer, with a 60 ms feed interval. Launch speed and
angle come from measured flywheel speed and hood position through the existing
calibration tables. Shot energy slows the flywheels. Launch angles are radians
inside `FuelSim`, and upward launch velocity is positive Z.

Shots travel along the robot's **local -Y axis**, matching the existing
`get_target_angle()` aiming calculation. They inherit chassis translation and
the tangential velocity of the launch point when the robot rotates. Gravity
produces the arc, and quadratic air drag is enabled. Hub nets only contact fuel
near their actual planes, and both hubs return scored fuel to their exits.

Fuel on carpet slows by `FRICTION * 9.81` m/s² (`FRICTION = 0.18`), rather than
the old nearly frictionless exponential decay. Speeds below 0.035 m/s and tiny
ground bounces below 0.25 m/s settle to rest. Moving fuel and bumpers can wake
resting balls. Bumper impacts use relative contact velocity, including rotation;
they no longer add repeated velocity boosts. Fuel/wall restitution dissipates
energy, and separating fuel contacts do not receive another bounce impulse.
These friction, bounce, and drag parameters are approximations to tune against
actual fuel on your carpet; they are not measured ball characteristics.

Field-fuel positions are published at up to 30 Hz through the existing
`FuelSimulation/Fuels` topic, only when positions change. Physics still runs
every robot loop. Held fuel retains `Hopper/Sim Fuels/Fuel 1` through `Fuel 24`
as robot-relative pose arrays. It fills supported staggered sphere layers
from the floor upward, entirely inside the configured 25 x 18 x 15 inch hopper,
instead of a cube suspended above it. The count changes update the display;
unused slots remain hidden below the floor.

Dashboard entries:

- `Sim/Ground Truth Pose`: physical X, Y, heading in degrees.
- `RobotPose`: physical robot pose in the simulation viewer.
- `Sim/Estimated Pose`, `Sim/Estimated Pose2d`: fused ghost pose (array/struct).
- `Sim/Odometry Pose`: encoder/gyro pose without vision corrections.
- `Sim/Estimate Position Error M`, `Sim/Estimate Heading Error Deg`,
  `Sim/Odometry Position Error M`: disagreement with physical truth.
- `Sim/Vision/<camera>/Detected Tag IDs`, `Accepted Tag IDs`, `Ambiguities`,
  `Distances M`, `Capture Timestamp Seconds`: per-camera coverage/quality.
- `Sim/Vision/Has Recent Measurement`, `Last Measurement Age Seconds`,
  `Measurements Fused`: whether the ghost has fresh usable vision; an age of
  -1 means none has arrived since the last starting-pose reset.
- `Sim/Battery Voltage`, `Sim/Current Amps`: estimated electrical load.
- `Sim/Step Count`: increases while the simulation physics callback is running.
- `Sim/Enabled`: Driver Station enable state seen by the motor plants.
- `Sim/Drive Motor Volts`, `Sim/Wheel Speeds MPS`: controller output and physical
  wheel speeds, ordered front left, front right, back left, back right.
- `Sim/Physics Frame MS`, `Sim/Step Duration MS`: work time and elapsed time
  advanced by physics, for diagnosing slow frames.
- Existing mechanism feedback, camera, hopper, and `FuelSimulation` entries.

## Calibration and limits

Mechanism inertias, intake pivot gearing, battery resistance, wheel calibration,
gyro error, and camera
properties in `robot_sim.py` are initial approximations. Drive mass comes from
PathPlanner's robot config. The hood conversion preserves the existing code's
rotor-rotations-as-degrees convention; confirm the physical hood reduction before
using the simulation to predict hardware behavior. Existing controller gains
may behave differently with these approximate plants.

This models motor/sensor response, rather than complete contact dynamics:
swerve wheel slip, chassis forces, ramps, interior robot-field collisions,
mechanism gravity, and camera occlusion by field structures
are not modeled. The chassis is constrained to the field perimeter. The existing
fuel physics handles fuel contacts. The currently commented-out fuel detection
cameras and climber are not enabled by this change.

Deterministic plant/fuel checks:
`python -m pytest tests/test_simulation.py tests/test_fuel_simulation.py -q`.
Full control pipeline regression:
`python -m pytest tests/test_simulation_pipeline.py -q`.
This starts `TimedRobot.startCompetition()` through RobotPy's physics adapter,
sends Driver Station joystick axes and intake/shooting triggers, checks controller
output/encoder feedback/published poses, and submits a real autonomous path
through the chooser. It measures forward/rotation response with all starting
fuel present, verifies that right-trigger shots leave upward toward the aimed
target and disappear from the hopper display, and verifies actual vision fusion corrects an injected
estimate error without moving truth. It also checks that disabled physics keeps
running while the motors stop. It does not manually invoke the physics update.
The existing whole-robot suite runs with `python -m robotpy test`; its fixed
two-second initialization timeout can expire during Phoenix setup on Windows.
