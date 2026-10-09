"""Exercise the real TimedRobot/RobotPy loop in a fresh native-CAN process."""

import math
import os
from pathlib import Path
import sys
import threading
import time
import traceback

ROBOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROBOT_DIR))
# RobotPy resolves deploy files relative to the main robot script.
sys.argv[0] = str(ROBOT_DIR / "robot.py")
sys.modules["__main__"].__file__ = str(ROBOT_DIR / "robot.py")

import ntcore
import hal
import wpilib
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import DriverStationSim
from robot import Robot


def run():
    errors = []
    for port in (0, 1):
        DriverStationSim.setJoystickAxisCount(port, 6)
        DriverStationSim.setJoystickButtonCount(port, 10)
        DriverStationSim.setJoystickPOVCount(port, 1)
        DriverStationSim.setJoystickPOV(port, 0, -1)
        DriverStationSim.setJoystickIsXbox(port, True)
    DriverStationSim.setDsAttached(True)
    DriverStationSim.setEnabled(False)
    DriverStationSim.setAutonomous(False)
    DriverStationSim.setAllianceStationId(hal.AllianceStationID.kBlue1)
    DriverStationSim.notifyNewData()
    interface, robot_class = PhysicsInterface._create_and_attach(Robot, ROBOT_DIR)
    assert interface is not None, "RobotPy must discover physics.py"
    robot = robot_class()

    def competition():
        try:
            robot.startCompetition()
        except BaseException:
            errors.append(traceback.format_exc())

    thread = threading.Thread(target=competition, daemon=True)
    thread.start()

    def wait_until(predicate, reason, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            DriverStationSim.notifyNewData()
            if errors:
                raise AssertionError(errors[0])
            assert thread.is_alive(), "TimedRobot competition loop stopped"
            if predicate():
                return
            time.sleep(0.02)
        raise AssertionError(reason)

    try:
        wait_until(lambda: hasattr(robot, "sim") and robot.sim.step_count > 3,
                   "RobotPy never called the custom physics", timeout=15)
        # Retain all 408 starting fuel objects: response tests must exercise the
        # performance path that previously made driving feel delayed.
        start_pose = robot.sim.pose
        start_count = robot.sim.step_count
        commanded_at = time.monotonic()
        DriverStationSim.setJoystickAxis(0, 1, -0.5)
        DriverStationSim.setEnabled(True)
        wait_until(lambda: robot.in_teleop_mode and robot.sim.step_count > start_count + 10
                   and robot.sim.pose.translation().distance(start_pose.translation()) > 0.2,
                   "Teleop joystick never moved the physical robot")
        assert time.monotonic() - commanded_at < 1.5, "Forward response is too slow with starting fuel"
        assert robot.oi.driver1.LEFT_JOY_Y() == -0.5
        assert abs(robot.drivetrain.log_chassis.vx) > 0.1
        assert max(abs(m.sim_state.motor_voltage) for m in robot.sim.drive_plants) > 0.05
        assert max(abs(s.speed) for s in robot.poseEstimator.get_module_states()) > 0.05
        wait_until(lambda: robot.poseEstimator.curEstPose.translation().distance(start_pose.translation()) > 0.15,
                   "Encoders never updated the estimated pose")
        wait_until(lambda: len(wpilib.SmartDashboard.getNumberArray("RobotPose", [])) == 3
                   and abs(wpilib.SmartDashboard.getNumberArray("RobotPose", [0])[0] - start_pose.X()) > 0.1,
                   "RobotPose used by the viewer never updated")
        print("PASS: teleop joystick -> scheduler -> controller voltage -> wheel feedback -> pose/viewer", flush=True)

        # Rotate from stationary and measure actual angular response.
        DriverStationSim.setJoystickAxis(0, 1, 0)
        DriverStationSim.setJoystickAxis(0, 4, -0.5)
        rotated_at = time.monotonic()
        wait_until(lambda: abs(robot.sim.speeds.omega) > 1.2,
                   "Rotation never reached a useful angular speed", timeout=2)
        assert time.monotonic() - rotated_at < 1.0, "Rotation response is too slow"
        DriverStationSim.setJoystickAxis(0, 4, 0)
        print("PASS: forward/rotation response with full starting fuel", flush=True)

        DriverStationSim.setJoystickAxis(0, 1, 0)
        DriverStationSim.setJoystickAxis(0, 2, 0.8)
        wait_until(lambda: robot.is_intaking and robot.intake.get_intake_speed() > 5
                   and robot.intake.get_position() < -0.25,
                   "OI trigger never changed intake state and simulated mechanism feedback")
        wait_until(lambda: wpilib.SmartDashboard.getBoolean("States/Is Intaking", False),
                   "Intake state visual cue was not published")
        components = wpilib.SmartDashboard.getNumberArray("FinalComponentPoses/Pose1", [])
        assert len(components) == 7 and all(math.isfinite(v) for v in components)
        DriverStationSim.setJoystickAxis(0, 2, 0)
        wait_until(lambda: not robot.is_intaking, "OI trigger release did not change intake state")
        print("PASS: OI trigger -> intake state/motors -> dashboard cues", flush=True)

        # Exercise the driver's actual right trigger and readiness state
        # machine, rather than directly commanding the shooting motors.
        robot.poseEstimator.reset_pose(Pose2d(2.5, 4, Rotation2d.fromDegrees(90)))
        shots = []
        launch_fuel = robot.fuel_sim.launchFuel

        def capture_launch(*args, **kwargs):
            launched = launch_fuel(*args, **kwargs)
            index = robot.fuel_sim.count - 1
            shots.append((robot.fuel_sim.positions[index].copy(),
                          robot.fuel_sim.velocities[index].copy(), robot.static_target))
            return launched

        robot.fuel_sim.launchFuel = capture_launch
        held_count = robot.fuel_in_hopper
        wait_until(lambda: wpilib.SmartDashboard.getNumberArray(
            f"Hopper/Sim Fuels/Fuel {held_count}", [0, 0, -1])[2] > 0,
            "Held fuel did not appear inside the hopper")
        DriverStationSim.setJoystickAxis(0, 3, 0.8)
        wait_until(lambda: len(shots) > 0,
                   "Right trigger never fed a shot through measured flywheel/accelerator/indexer feedback")
        position, velocity, target = shots[0]
        to_target = (target.X() - position[0], target.Y() - position[1])
        alignment = (velocity[0] * to_target[0] + velocity[1] * to_target[1]) / (
            math.hypot(*velocity[:2]) * math.hypot(*to_target))
        assert alignment > 0.95, "Shot traveled away from the aimed target"
        assert velocity[2] > 0, "Shot did not leave the hood upward"
        wait_until(lambda: robot.fuel_in_hopper < held_count
                   and wpilib.SmartDashboard.getNumberArray(
                       f"Hopper/Sim Fuels/Fuel {held_count}", [0, 0, 0])[2] < 0,
                   "Shooting did not consume and hide held fuel")
        DriverStationSim.setJoystickAxis(0, 3, 0)
        wait_until(lambda: not robot.shoot_intent, "Trigger release never completed spin-down")
        robot.fuel_sim.launchFuel = launch_fuel
        print("PASS: right trigger -> readiness/motors -> upward shot toward target -> hopper display", flush=True)

        DriverStationSim.setEnabled(False)
        wait_until(lambda: wpilib.DriverStation.isDisabled(), "Robot did not disable")
        # Select a real configured PathPlanner routine through the same NT entry
        # as the chooser UI, then submit it while disabled.
        table = ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard")
        table.getSubTable("Auto Chooser").putString("selected", "Just Depot Auto 3-Bot")
        wait_until(lambda: robot.auto_chooser.getSelected() == 16, "Auto chooser selection did not arrive")
        wpilib.SmartDashboard.putNumber("Submit Auto? (and FMS Connected)", 1)
        wait_until(lambda: robot.auto_submitted and robot.auto is not None,
                   "Auto submission did not create the selected routine")
        auto_start = robot.sim.pose
        DriverStationSim.setAutonomous(True)
        DriverStationSim.setEnabled(True)
        wait_until(lambda: robot.in_autonomous_mode
                   and robot.sim.pose.translation().distance(auto_start.translation()) > 0.2,
                   "Scheduled autonomous path never moved the physical robot")
        assert max(abs(m.sim_state.motor_voltage) for m in robot.sim.drive_plants) > 0.05
        wait_until(lambda: robot.poseEstimator.curEstPose.translation().distance(auto_start.translation()) > 0.15,
                   "Autonomous encoders never updated the estimated pose")
        print("PASS: chooser -> autonomous scheduler -> PathPlanner -> motor output -> pose", flush=True)

        # Re-enabling autonomous in the same session restarts the routine from
        # its starting pose (a fresh "match"), rather than following the path
        # from wherever the robot stopped.
        DriverStationSim.setEnabled(False)
        disabled_at = robot.sim.step_count
        # Let the robot loop itself run the disabled transition.
        wait_until(lambda: robot.sim.step_count > disabled_at + 5, "Robot loop did not run while disabled")
        DriverStationSim.setEnabled(True)
        wait_until(lambda: robot.in_autonomous_mode
                   and robot.sim.pose.translation().distance(auto_start.translation()) < 0.15,
                   "Autonomous re-run did not return to the starting pose", timeout=3)
        wait_until(lambda: robot.sim.pose.translation().distance(auto_start.translation()) > 0.2,
                   "Autonomous re-run never moved the physical robot")
        print("PASS: autonomous re-run restarts from the submitted starting pose", flush=True)

        before_disabled = robot.sim.step_count
        DriverStationSim.setEnabled(False)
        wait_until(lambda: robot.sim.step_count > before_disabled + 15
                   and max(abs(w) for w in robot.sim.chassis.wheel_velocity) < 0.5,
                   "Disabling did not stop the simulated drive motors")
        assert not wpilib.SmartDashboard.getBoolean("Sim/Enabled", True)
        print("PASS: simulation heartbeat continues while disabled; motors stop", flush=True)

        # Reset to a tag-visible location, then corrupt only the estimated
        # translation. Camera frames must remain based on physical truth, and
        # the actual single-tag pipeline/filter must correct the injected error.
        robot.poseEstimator.reset_pose(Pose2d(3.5, 4, Rotation2d()))
        wait_until(lambda: robot.poseEstimator.sim_last_vision_time is not None,
                   "No usable simulated AprilTag measurements at a tag-visible pose")
        truth = robot.sim.pose
        wrong_pose = Pose2d(truth.translation() + Translation2d(0.4, -0.2), truth.rotation())
        robot.poseEstimator.poseEst.resetPosition(robot.poseEstimator.getYaw(),
                                                  robot.poseEstimator.get_module_positions(), wrong_pose)
        before_measurements = robot.poseEstimator.sim_vision_measurements
        wait_until(lambda: robot.poseEstimator.sim_vision_measurements > before_measurements + 3
                   and robot.poseEstimator.curEstPose.translation().distance(truth.translation()) < 0.12,
                   "Vision did not correct an estimate error independently of ground truth")
        assert robot.sim.pose.translation().distance(truth.translation()) < 0.01
        ghost = wpilib.SmartDashboard.getNumberArray("Sim/Estimated Pose", [])
        assert len(ghost) == 3
        assert math.hypot(ghost[0] - truth.X(), ghost[1] - truth.Y()) < 0.12
        subscriber = ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard").getStructTopic(
            "Sim/Estimated Pose2d", Pose2d).subscribe(Pose2d())
        assert subscriber.get().translation().distance(truth.translation()) < 0.12
        assert len(wpilib.SmartDashboard.getNumberArray("Sim/Odometry Pose", [])) == 3
        assert wpilib.SmartDashboard.getBoolean("Sim/Vision/Has Recent Measurement", False)
        print("PASS: independent camera truth -> ambiguity filtering -> real estimator correction -> ghost pose", flush=True)
    finally:
        DriverStationSim.setEnabled(False)
        DriverStationSim.notifyNewData()
        robot.endCompetition()
        thread.join(timeout=3)
        assert not thread.is_alive(), "Competition thread failed to stop"


if __name__ == "__main__":
    result = 0
    try:
        run()
    except BaseException:
        traceback.print_exc()
        result = 1
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        # Phoenix owns native background threads. As with pyfrc's isolated
        # runner, finish the process without Python interpreter teardown races.
        os._exit(result)
