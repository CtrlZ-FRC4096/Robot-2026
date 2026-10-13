"""Deterministic behaviour recorder for refactors of the robot's control logic.

Runs the real Robot class in simulation with paused HAL timing, stepping one
20 ms loop at a time, while a scripted scenario presses Driver Station buttons
or runs an autonomous routine. Every loop it records what the robot actually
does: the effective control request on every TalonFX, the robot pose, and the
simulated fuel counts. Two code versions behave identically for a scenario
when their traces match.

    python tests/behavior_trace.py <robot dir> <scenario> <output .jsonl>
    python tests/behavior_trace.py --list
    python tests/behavior_trace.py --compare <a.jsonl> <b.jsonl>

compare_behavior.py runs every scenario against a baseline git revision and
the working tree, and reports the first loop where they differ.

Only hardware APIs, the Driver Station, the auto chooser and a few long-lived
robot attributes (pose estimate, fuel counts) are touched here, so the same
recorder works before and after a refactor of the state handling. The gyro
and motor sensors are replaced by deterministic fakes: Phoenix simulation
updates them from its own real-time threads, which would make runs differ.

The physics (physics.py) isn't run here, and since the robot code reads its
encoders in simulation too, the recorded robot doesn't move and mechanisms
stay at zero speed: traces compare control requests and state handling, not
driving or shooting. Use tests/sim_auto.py to watch the physics.
"""

import json
import math
import os
import random
import sys
import threading
import time
from pathlib import Path

PERIOD = 0.02

# Xbox button numbers (wpilib.XboxController.Button).
BUTTONS = {"A": 1, "B": 2, "X": 3, "Y": 4, "LB": 5, "RB": 6, "BACK": 7, "START": 8,
           "LSTICK": 9, "RSTICK": 10}
AXES = {"LX": 0, "LY": 1, "LT": 2, "RT": 3, "RX": 4, "RY": 5}
POV = {"UP": 0, "RIGHT": 90, "DOWN": 180, "LEFT": 270, "NONE": -1}

AUTO_NAMES = {
    5: "Right Trench & Bump MVR PP",
    6: "Left Trench & Bump MVR PP",
    14: "Left Trench Safe 3-Bot + Depot After PP",
    15: "Steph Curry 3-Bot Depot First",
    16: "Just Depot Auto 3-Bot",
    0: "Default (no auto)",
}


# --------------------------------------------------------------- scenarios
# Each scenario is a list of (time in seconds, action, *arguments). Actions
# are methods of Driver below. A scenario ends at its last event time.

def _teleop_start(x, y, heading, alliance="blue"):
    return [(0.0, "alliance", alliance), (0.0, "place", x, y, heading),
            (0.1, "enable", "teleop")]


SCENARIOS = {
    # Drive into the centre fuel holding the intake, back to the alliance
    # zone, then shoot until empty and release (spin down).
    "teleop_intake_then_shoot": _teleop_start(6.4, 4.4, 0) + [
        (0.3, "axis", 0, "LY", -0.6),
        (0.4, "axis", 0, "LT", 0.9),
        (2.4, "axis", 0, "LY", 0.0),
        (2.5, "axis", 0, "LT", 0.0),
        (2.6, "axis", 0, "LY", 0.7),
        (4.7, "axis", 0, "LY", 0.0),
        (5.0, "axis", 0, "RT", 0.9),
        (10.0, "axis", 0, "RT", 0.0),
        (11.5, "noop"),
    ],
    # Intake held while shooting; release the intake first (the pivot then
    # agitates while feeding), then the trigger. Then a second shot while
    # driving slowly, and a re-press of the trigger during spin-down.
    "teleop_intake_while_shooting": _teleop_start(2.6, 4.0, 90) + [
        (0.2, "set_fuel", 20),
        (0.3, "axis", 0, "LT", 0.9),
        (0.5, "axis", 0, "RT", 0.9),
        (2.0, "axis", 0, "LT", 0.0),
        (3.5, "axis", 0, "RT", 0.0),
        (3.52, "axis", 0, "RT", 0.9),
        (4.0, "axis", 0, "LX", 0.4),
        (5.5, "axis", 0, "LX", 0.0),
        (6.0, "axis", 0, "RT", 0.0),
        (7.0, "noop"),
    ],
    # Driver 2 manual ("down bad") shot, then clear-jam hold and release,
    # then a manual shot interrupted by a jam on the same hat.
    "teleop_manual_shot_and_jam": _teleop_start(3.0, 2.0, 0) + [
        (0.2, "set_fuel", 12),
        (0.4, "pov", 1, "RIGHT"),
        (2.4, "pov", 1, "NONE"),
        (3.2, "pov", 1, "LEFT"),
        (4.2, "pov", 1, "NONE"),
        (4.6, "axis", 0, "LT", 0.9),
        (5.0, "pov", 1, "LEFT"),
        (5.6, "axis", 0, "LT", 0.0),
        (6.0, "pov", 1, "NONE"),
        (6.4, "pov", 1, "RIGHT"),
        (7.0, "pov", 1, "LEFT"),
        (7.6, "pov", 1, "NONE"),
        (8.5, "noop"),
    ],
    # Cancelling: stow-everything (LB) and stow-intake (d1 POV left) during
    # shots, the trigger released after LB, and intake stow/deploy cycles.
    "teleop_cancel_shots": _teleop_start(2.2, 5.5, 0) + [
        (0.2, "set_fuel", 20),
        (0.3, "axis", 0, "RT", 0.9),
        (1.5, "button", 0, "LB", True),
        (1.6, "button", 0, "LB", False),
        (2.0, "axis", 0, "RT", 0.0),
        (2.6, "axis", 0, "RT", 0.9),
        (3.6, "pov", 0, "LEFT"),
        (3.7, "pov", 0, "NONE"),
        (4.0, "axis", 0, "RT", 0.0),
        (4.4, "axis", 0, "LT", 0.9),
        (4.8, "axis", 0, "LT", 0.0),
        (5.0, "pov", 0, "LEFT"),
        (5.1, "pov", 0, "NONE"),
        (5.4, "axis", 0, "RT", 0.9),
        (6.8, "axis", 0, "RT", 0.0),
        (7.5, "button", 0, "LB", True),
        (7.6, "button", 0, "LB", False),
        (8.0, "noop"),
    ],
    # Trench lineup (RB) alone and while aiming, plain driving with rotation,
    # heading hold, cardinal buttons, gyro zero, rotation boost, BACK toggle,
    # passing shot from the neutral zone, and a disable/enable cycle.
    "teleop_drive_and_lineup": _teleop_start(3.0, 6.5, 0) + [
        (0.3, "button", 0, "RB", True),
        (1.8, "button", 0, "RB", False),
        (2.0, "axis", 0, "RX", 0.6),
        (2.6, "axis", 0, "RX", 0.0),
        (2.7, "axis", 0, "LX", 0.5),
        (3.4, "axis", 0, "LX", 0.0),
        (3.6, "button", 0, "B", True),
        (3.7, "button", 0, "B", False),
        (4.3, "button", 0, "Y", True),
        (4.4, "button", 0, "Y", False),
        (4.6, "axis", 0, "LY", -0.4),
        (5.2, "axis", 0, "LY", 0.0),
        (5.3, "button", 0, "RSTICK", True),
        (5.4, "axis", 0, "RX", -0.5),
        (5.8, "axis", 0, "RX", 0.0),
        (5.9, "button", 0, "RSTICK", False),
        (6.0, "button", 0, "BACK", True),
        (6.1, "button", 0, "BACK", False),
        (6.2, "pov", 0, "DOWN"),
        (6.3, "pov", 0, "NONE"),
        (6.4, "set_fuel", 10),
        (6.5, "axis", 0, "RT", 0.9),
        (6.6, "button", 0, "RB", True),
        (8.0, "button", 0, "RB", False),
        (8.4, "axis", 0, "RT", 0.0),
        (9.0, "place", 8.5, 2.0, 0),
        (9.1, "axis", 0, "RT", 0.9),
        (10.5, "axis", 0, "RT", 0.0),
        (11.0, "disable"),
        (11.5, "enable", "teleop"),
        (12.0, "noop"),
    ],
}


def _auto_scenario(choice, alliance):
    return [
        (0.0, "alliance", alliance),
        (0.1, "select_auto", AUTO_NAMES[choice]),
        (0.2, "submit_auto"),
        (0.5, "enable", "auto"),
        (15.5, "enable", "teleop"),
        # Auto -> teleop: whatever the auto left running carries over.
        (16.0, "axis", 0, "RT", 0.9),
        (17.0, "axis", 0, "RT", 0.0),
        (18.0, "noop"),
    ]


for _choice in AUTO_NAMES:
    for _alliance in ("blue", "red"):
        SCENARIOS[f"auto_{_choice}_{_alliance}"] = _auto_scenario(_choice, _alliance)


# ------------------------------------------------------- deterministic fakes

class _Signal:
    def __init__(self, value):
        self.value = value


def install_fakes():
    """Deterministic gyro and motor sensors, and capture of motor requests."""
    from phoenix6 import StatusCode
    from phoenix6.hardware import Pigeon2, TalonFX
    from wpimath.geometry import Rotation3d

    def set_yaw(self, value, *args, **kwargs):
        self._fake_yaw = float(value)
        return StatusCode.OK

    def yaw(self, *args, **kwargs):
        return _Signal(getattr(self, "_fake_yaw", 0.0))

    zero = lambda self, *args, **kwargs: _Signal(0.0)
    Pigeon2.set_yaw = set_yaw
    Pigeon2.get_yaw = yaw
    for name in ("get_acceleration_x", "get_acceleration_y", "get_acceleration_z",
                 "get_angular_velocity_z_world", "get_roll", "get_pitch"):
        setattr(Pigeon2, name, zero)
    Pigeon2.getRotation3d = lambda self: Rotation3d(0, 0, math.radians(getattr(self, "_fake_yaw", 0.0)))

    for name in ("get_position", "get_velocity", "get_acceleration", "get_torque_current"):
        setattr(TalonFX, name, zero)

    original = TalonFX.set_control

    def set_control(self, request):
        fields = {k: v for k, v in vars(request).items() if isinstance(v, (bool, int, float, str))}
        self._recorded_control = [type(request).__name__, sorted(fields.items())]
        return original(self, request)

    TalonFX.set_control = set_control


# ------------------------------------------------------------------ driver

class Driver:
    def __init__(self, robot):
        from wpilib.simulation import DriverStationSim
        self.robot = robot
        self.ds = DriverStationSim
        for port in (0, 1):
            DriverStationSim.setJoystickAxisCount(port, 6)
            DriverStationSim.setJoystickButtonCount(port, 10)
            DriverStationSim.setJoystickPOVCount(port, 1)
            DriverStationSim.setJoystickPOV(port, 0, -1)
            DriverStationSim.setJoystickIsXbox(port, True)
        DriverStationSim.setDsAttached(True)

    def noop(self):
        pass

    def alliance(self, color):
        import hal
        self.ds.setAllianceStationId(hal.AllianceStationID.kRed1 if color == "red"
                                     else hal.AllianceStationID.kBlue1)

    def enable(self, mode):
        self.ds.setAutonomous(mode == "auto")
        self.ds.setEnabled(True)

    def disable(self):
        self.ds.setEnabled(False)

    def axis(self, port, name, value):
        self.ds.setJoystickAxis(port, AXES[name], value)

    def button(self, port, name, pressed):
        self.ds.setJoystickButton(port, BUTTONS[name], pressed)

    def pov(self, port, direction):
        self.ds.setJoystickPOV(port, 0, POV[direction])

    def place(self, x, y, heading):
        from wpimath.geometry import Pose2d, Rotation2d
        robot = self.robot
        pose = Pose2d(x, y, Rotation2d.fromDegrees(heading))
        robot.poseEstimator.curEstPose = pose
        robot.poseEstimator.gyro.set_yaw(heading)
        # The pose estimate comes from the estimator in simulation too now.
        robot.poseEstimator.poseEst.resetPosition(
            Rotation2d.fromDegrees(heading), robot.poseEstimator.get_module_positions(), pose)

    def set_fuel(self, count):
        self.robot.fuel_in_hopper = count

    def select_auto(self, name):
        import ntcore
        ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard") \
            .getSubTable("Auto Chooser").putString("selected", name)

    def submit_auto(self):
        import wpilib
        wpilib.SmartDashboard.putNumber("Submit Auto? (and FMS Connected)", 1)


# ------------------------------------------------------------------ record

def snapshot(robot, motors):
    pose = robot.poseEstimator.curEstPose
    # Fuel is simulated by physics.py (simulation/arena.py), which this
    # recorder doesn't run; older revisions kept a FuelSim on the robot.
    fuel = getattr(robot, "fuel_sim", None)
    return {
        "controls": {str(m.device_id): getattr(m, "_recorded_control", None) for m in motors},
        "pose": [pose.X(), pose.Y(), pose.rotation().degrees()],
        "hopper": robot.fuel_in_hopper,
        "field_fuel": len(fuel.fuels) if fuel is not None and hasattr(fuel, "fuels") else
                      getattr(fuel, "count", None),
        "score": [fuel.blueHub.getScore(), fuel.redHub.getScore()] if fuel is not None else None,
    }


def run(robot_dir, scenario_name, out_path):
    robot_dir = Path(robot_dir).resolve()
    sys.path.insert(0, str(robot_dir))
    os.chdir(robot_dir)
    sys.argv[0] = str(robot_dir / "robot.py")
    sys.modules["__main__"].__file__ = str(robot_dir / "robot.py")
    random.seed(4096)
    import numpy as np
    np.random.seed(4096)
    install_fakes()

    import wpilib
    from wpilib.simulation import pauseTiming, restartTiming, stepTiming, DriverStationSim
    from phoenix6.hardware import TalonFX
    from robot import Robot

    # Simulated time starts at zero every run (as in pyfrc), so timers that
    # subtract absolute timestamps round identically.
    pauseTiming()
    restartTiming()
    robot = Robot()
    errors = []

    def competition():
        try:
            robot.startCompetition()
        except BaseException as exc:  # surfaced below
            errors.append(exc)
            raise

    thread = threading.Thread(target=competition, daemon=True)
    thread.start()
    deadline = time.monotonic() + 60
    # robot_start has finished setting up once the last attribute it creates exists.
    while not (hasattr(robot, "fuel_sim") or hasattr(robot, "component_poses_pub")):
        if errors or time.monotonic() > deadline:
            raise RuntimeError(f"robot did not initialise: {errors}")
        time.sleep(0.01)
    driver = Driver(robot)
    motors = sorted({v for v in _all_objects(robot) if isinstance(v, TalonFX)},
                    key=lambda m: m.device_id)

    events = sorted(SCENARIOS[scenario_name], key=lambda e: e[0])
    end_tick = round(events[-1][0] / PERIOD)
    pending = list(events)
    with open(out_path, "w") as out:
        for tick in range(end_tick + 1):
            while pending and round(pending[0][0] / PERIOD) <= tick:
                _, action, *args = pending.pop(0)
                getattr(driver, action)(*args)
            DriverStationSim.notifyNewData()
            stepTiming(PERIOD)
            if errors or not thread.is_alive():
                raise RuntimeError(f"robot thread died at tick {tick}: {errors}")
            out.write(json.dumps({"tick": tick, **snapshot(robot, motors)}, default=_plain) + "\n")
    robot.endCompetition()


def _plain(value):
    """numpy scalars (fuel simulation counters) as plain numbers."""
    return value.item() if hasattr(value, "item") else str(value)


def _all_objects(robot):
    """Every attribute reachable one or two levels below the robot."""
    seen = []
    for value in vars(robot).values():
        seen.append(value)
        if hasattr(value, "__dict__"):
            seen.extend(vars(value).values())
            for inner in vars(value).values():
                if isinstance(inner, (list, tuple)):
                    for item in inner:
                        seen.append(item)
                        if hasattr(item, "__dict__"):
                            seen.extend(vars(item).values())
    return seen


# ----------------------------------------------------------------- compare

TOLERANCE = 1e-9


def _close(a, b):
    """Equal, except floats may differ by TOLERANCE.

    Repeated runs of identical code differ in the last bit or two of the pose
    (about 1e-15), so exact float equality would report noise as changes.
    Everything discrete (request types, booleans, counts) must match exactly.
    """
    if isinstance(a, float) or isinstance(b, float):
        return (isinstance(a, (int, float)) and isinstance(b, (int, float))
                and not isinstance(a, bool) and not isinstance(b, bool)
                and math.isclose(a, b, rel_tol=TOLERANCE, abs_tol=TOLERANCE))
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_close(x, y) for x, y in zip(a, b))
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_close(a[k], b[k]) for k in a)
    return a == b


def compare(path_a, path_b, limit=8):
    """Return (equivalent, report lines) for two trace files."""
    with open(path_a) as fa, open(path_b) as fb:
        a = [json.loads(line) for line in fa]
        b = [json.loads(line) for line in fb]
    report = []
    if len(a) != len(b):
        report.append(f"length differs: {len(a)} vs {len(b)} loops")
    for ra, rb in zip(a, b):
        if _close(ra, rb):
            continue
        diffs = []
        for key in ("pose", "hopper", "field_fuel", "score"):
            if not _close(ra[key], rb[key]):
                diffs.append(f"{key}: {ra[key]} -> {rb[key]}")
        for motor in sorted(set(ra["controls"]) | set(rb["controls"]), key=int):
            ca, cb = ra["controls"].get(motor), rb["controls"].get(motor)
            if not _close(ca, cb):
                diffs.append(f"motor {motor}: {_short(ca)} -> {_short(cb)}")
        report.append(f"loop {ra['tick']} ({ra['tick'] * PERIOD:.2f} s): " + "; ".join(diffs))
        if len(report) >= limit:
            report.append("...")
            break
    return not report, report


def _short(control):
    if control is None:
        return "none"
    name, fields = control
    keep = {k: v for k, v in fields if k in ("output", "velocity", "position", "leader_id")}
    return f"{name}{keep}"


if __name__ == "__main__":
    if sys.argv[1:2] == ["--list"]:
        print("\n".join(SCENARIOS))
    elif sys.argv[1:2] == ["--compare"]:
        same, lines = compare(sys.argv[2], sys.argv[3])
        print("identical" if same else "\n".join(lines))
        sys.exit(0 if same else 1)
    else:
        code = 0
        try:
            run(*sys.argv[1:4])
        except BaseException:
            import traceback
            traceback.print_exc()
            code = 1
        sys.stdout.flush()
        sys.stderr.flush()
        # Phoenix owns native threads; skip interpreter teardown races.
        os._exit(code)
