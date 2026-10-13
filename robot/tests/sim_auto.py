"""Run one autonomous routine in the full physics simulation, without the GUI,
and report what happened: path followed, fuel picked up and shot, hub score,
pose-estimate error and loop timing.

    python tests/sim_auto.py [auto number] [blue|red] [seconds]
    python tests/sim_auto.py 6 blue 20

Auto numbers are the auto chooser values in robot.py (5, 6, 14, 15, 16).
Runs in real time: Phoenix's simulated devices run on the wall clock.
Watch it live by connecting AdvantageScope to localhost while it runs.
"""

import math
import os
import sys
import threading
import time
from pathlib import Path

ROBOT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROBOT_DIR))
os.chdir(ROBOT_DIR)
# wpilib finds deploy/ next to the main script.
sys.argv[0] = str(ROBOT_DIR / "robot.py")
sys.modules["__main__"].__file__ = str(ROBOT_DIR / "robot.py")

AUTO_NAMES = {
    5: "Right Trench & Bump MVR PP",
    6: "Left Trench & Bump MVR PP",
    14: "Left Trench Safe 3-Bot + Depot After PP",
    15: "Steph Curry 3-Bot Depot First",
    16: "Just Depot Auto 3-Bot",
    0: "Default (no auto)",
}


def main():
    choice = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    alliance = sys.argv[2] if len(sys.argv) > 2 else "blue"
    seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0

    import hal
    import ntcore
    import wpilib
    from pyfrc.physics.core import PhysicsInterface
    from wpilib.simulation import DriverStationSim

    import robot as robot_module
    from simulation.arena import SimulatedArena

    _, robot_class = PhysicsInterface._create_and_attach(robot_module.Robot, ROBOT_DIR)
    robot = robot_class()

    # Time the robot loop: robot code, logging, physics, and the loop period.
    timings = {"robot code": [], "logging": [], "physics": [], "loop period": []}
    loop_starts = []

    def timed(name, function, marks_loop=False):
        def wrapper(*args):
            start = time.perf_counter()
            if marks_loop:
                if loop_starts:
                    timings["loop period"].append((start - loop_starts[-1]) * 1000)
                loop_starts.append(start)
            result = function(*args)
            timings[name].append((time.perf_counter() - start) * 1000)
            return result
        return wrapper

    robot.robotPeriodic = timed("robot code", robot.robotPeriodic, marks_loop=True)
    robot.log = timed("logging", robot.log)
    SimulatedArena.update = timed("physics", SimulatedArena.update)

    errors = []

    def competition():
        try:
            robot.startCompetition()
        except BaseException as exc:
            errors.append(exc)
            raise

    threading.Thread(target=competition, daemon=True).start()
    deadline = time.monotonic() + 60
    while SimulatedArena.instance() is None:
        if errors or time.monotonic() > deadline:
            raise RuntimeError(f"simulation did not start: {errors}")
        time.sleep(0.05)
    arena = SimulatedArena.instance()

    DriverStationSim.setDsAttached(True)
    for port in (0, 1):  # two idle Xbox controllers, so the bindings don't warn
        DriverStationSim.setJoystickIsXbox(port, True)
        DriverStationSim.setJoystickAxisCount(port, 6)
        DriverStationSim.setJoystickButtonCount(port, 10)
        DriverStationSim.setJoystickPOVCount(port, 1)
        DriverStationSim.setJoystickPOV(port, 0, -1)
    DriverStationSim.setAllianceStationId(
        hal.AllianceStationID.kRed1 if alliance == "red" else hal.AllianceStationID.kBlue1)
    DriverStationSim.notifyNewData()
    time.sleep(0.5)
    table = ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard")
    deadline = time.monotonic() + 10
    while robot.auto_chooser.getSelected() != choice:
        if time.monotonic() > deadline:
            raise RuntimeError("auto chooser did not take the selection")
        table.getSubTable("Auto Chooser").putString("selected", AUTO_NAMES[choice])
        time.sleep(0.1)
    wpilib.SmartDashboard.putNumber("Submit Auto? (and FMS Connected)", 1)
    time.sleep(0.5)
    if not robot.auto_submitted:
        raise RuntimeError("auto was not submitted")
    start_pose = arena.drive.pose
    print(f"Auto {choice} ({AUTO_NAMES[choice]}), {alliance}, start "
          f"({start_pose.X():.2f}, {start_pose.Y():.2f}, {start_pose.rotation().degrees():.0f} deg)")

    DriverStationSim.setAutonomous(True)
    DriverStationSim.setEnabled(True)
    DriverStationSim.notifyNewData()
    t0 = time.monotonic()
    worst_error = 0.0
    print(" time | true x  y   deg | est err | batt V | pivot roller | flywheel | hopper fired scored | intake / shooter")
    next_print = 0.0
    while time.monotonic() - t0 < seconds:
        time.sleep(0.02)
        if errors:
            raise RuntimeError(f"robot crashed: {errors}")
        t = time.monotonic() - t0
        true_pose = arena.drive.pose
        est = robot.poseEstimator.curEstPose
        error = true_pose.translation().distance(est.translation())
        worst_error = max(worst_error, error)
        if t >= next_print:
            next_print += 1.0
            print(f"{t:5.1f} | {true_pose.X():5.2f} {true_pose.Y():4.2f} {true_pose.rotation().degrees():4.0f} "
                  f"| {error:5.2f} m | {arena.battery_voltage:5.1f}  "
                  f"| {arena.intake.pivot.rotations:5.2f} {arena.intake.rollers.rps:5.0f}  "
                  f"| {abs(arena.shooter.flywheel.rps):5.0f}    "
                  f"| {arena.hopper_count:6d} {arena.shooter.shots_fired:5d} {arena.fuel.shots_scored:6d} "
                  f"| {robot.state.intake.value} / {robot.state.shooter.value}")

    DriverStationSim.setEnabled(False)
    DriverStationSim.notifyNewData()
    fired, scored = arena.shooter.shots_fired, arena.fuel.shots_scored
    print()
    print(f"Fuel shot {fired}, scored {scored}, missed {arena.fuel.shots_missed}"
          f"{f' ({100 * scored / fired:.0f}% accuracy)' if fired else ''}")
    print(f"Hub score: blue {arena.fuel.blue_hub.score}, red {arena.fuel.red_hub.score}")
    print(f"Worst pose-estimate error: {worst_error:.2f} m")
    for name, values in timings.items():
        values = sorted(values[len(values) // 10:])  # skip start-up
        if values:
            print(f"{name:>12}: median {values[len(values) // 2]:5.1f} ms, "
                  f"95th percentile {values[int(len(values) * 0.95)]:5.1f} ms")
    print("(the loop period should stay near 20 ms; if it doesn't, paths and timers run ahead of the robot)")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
