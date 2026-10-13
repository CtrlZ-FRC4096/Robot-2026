"""The simulated world (maple-sim's SimulatedArena): drivetrain, mechanisms,
fuel, cameras and battery, stepped once per robot loop by physics.py.

Robot code reaches it only in simulation, through SimulatedArena.instance(),
to teleport the robot when an auto is selected (the real robot is placed
on its starting spot by hand).
"""

import math

import ntcore
from wpilib.simulation import BatterySim, RoboRioSim
from wpimath.geometry import Pose2d, Pose3d, Rotation3d
from wpimath.kinematics import SwerveModuleState
from wpimath.units import inchesToMeters

from simulation.fuel_sim import FUEL_MASS, FUEL_RADIUS, FuelSim
from simulation.mechanisms import HOPPER_CAPACITY, Intake, Shooter
from simulation.swerve_sim import SwerveDriveSim

ENABLE_VISION = True
MECHANISM_SUB_TICKS = 5
BATTERY_VOLTAGE = 13.5  # maple-sim SimulatedBattery nominal (charged, no load)
BATTERY_RESISTANCE = 0.02  # ohms (maple-sim)
BATTERY_CURRENT_SMOOTHING = 0.3  # weight of this loop's current in the average
# Where balls get picked up, in robot coordinates (over the front bumper).
INTAKE_ZONE = (inchesToMeters(16.69), inchesToMeters(26.18), inchesToMeters(-18.98), inchesToMeters(13.01))
# Where the robot model's hopper-ball components are drawn (robot coordinates,
# relative to their zeroed position in the AdvantageScope model).
HOPPER_CORNER = (-0.26, -0.28, 0.28 + 0.5)
HOPPER_SIZE = (inchesToMeters(25), inchesToMeters(18))
HIDDEN_BALL = Pose3d(0, 0, -0.5, Rotation3d())


class SimulatedArena:
    _instance = None

    @classmethod
    def instance(cls):
        """The running arena, or None (on the real robot, or before physics starts)."""
        return cls._instance

    def __init__(self, robot):
        self.robot = robot
        estimator = robot.poseEstimator
        self.drive = SwerveDriveSim(estimator.modules, estimator.gyro, estimator.curEstPose)
        self.shooter = Shooter(robot.shooter, robot.hopper,
                               (robot.drivetrain.vel_lookup_table, robot.drivetrain.angle_lookup_table))
        self.intake = Intake(robot.intake)
        self.mechanisms = self.shooter.mechanisms + self.intake.mechanisms
        self.preload = robot.fuel_in_hopper
        self.hopper_count = self.preload
        self.fuel = FuelSim(self._robot_state, self.drive.length, self.drive.width,
                            intake_zone=INTAKE_ZONE, can_intake=self._can_intake, on_intake=self._on_intake)
        self.fuel.spawn_starting_fuel()
        self.vision = None
        if ENABLE_VISION:
            from simulation.vision_sim import VisionSim
            self.vision = VisionSim(estimator.cams)
        self.battery_voltage = BATTERY_VOLTAGE
        self.battery_current = 0.0
        self._sim_field_pose = estimator.field.getObject("Sim True Pose")

        table = ntcore.NetworkTableInstance.getDefault().getTable("Sim")
        self.pubs = {
            "pose": table.getStructTopic("Robot Pose", Pose2d).publish(),
            "modules": table.getStructArrayTopic("Module States", SwerveModuleState).publish(),
        }
        self.numbers = {name: table.getDoubleTopic(name).publish() for name in (
            "Hopper Count", "Shots Fired", "Shots Scored", "Shots Missed", "Blue Hub Score",
            "Red Hub Score", "Flywheel RPS", "Accelerator RPS", "Indexer RPS", "Hood Rotations",
            "Intake Pivot Rotations", "Intake Roller RPS", "Launch Speed", "Launch Angle",
            "Battery Voltage", "Battery Current", "Speed")}
        SimulatedArena._instance = self

    # -------------------------------------------------------------- control
    def reset_for_auto(self, start_pose: Pose2d):
        """Fresh field, preloaded hopper, robot on its starting spot."""
        self.drive.set_pose(start_pose)
        self.fuel.spawn_starting_fuel()
        self.fuel.blue_hub.score = self.fuel.red_hub.score = 0
        self.fuel.shots_scored = self.fuel.shots_missed = 0
        self.shooter.shots_fired = 0
        self.hopper_count = self.preload

    def reset_robot_pose(self, pose: Pose2d):
        self.drive.set_pose(pose)

    # ---------------------------------------------------------------- step
    def update(self, dt):
        volts = self.battery_voltage
        for mechanism in self.mechanisms:
            mechanism.sample(volts)
        self.drive.step(dt, volts)
        sub_dt = dt / MECHANISM_SUB_TICKS
        for _ in range(MECHANISM_SUB_TICKS):
            for mechanism in self.mechanisms:
                mechanism.step(sub_dt)
        for mechanism in self.mechanisms:
            mechanism.write()
        self.intake.write_cancoder()

        for _ in range(self.shooter.feed(self.hopper_count, dt)):
            self.fuel.launch(*self.shooter.shoot(self.drive.pose, self.drive.vx, self.drive.vy,
                                                 self.drive.omega, FUEL_MASS))
            self.hopper_count -= 1
        self.fuel.step(dt)

        if self.vision is not None:
            self.vision.update(self.drive.pose)

        # Like maple-sim's SimulatedBattery: averaged load current, and never
        # below the roboRIO brownout voltage.
        current = max(0.0, self.drive.supply_current() + sum(m.supply_current() for m in self.mechanisms))
        self.battery_current += BATTERY_CURRENT_SMOOTHING * (current - self.battery_current)
        self.battery_voltage = max(RoboRioSim.getBrownoutVoltage(), BatterySim.calculate(
            BATTERY_VOLTAGE, BATTERY_RESISTANCE, [self.battery_current]))
        RoboRioSim.setVInVoltage(self.battery_voltage)
        self._publish()

    def _robot_state(self):
        d = self.drive
        return d.x, d.y, d.heading, d.vx, d.vy, d.omega

    def _can_intake(self):
        return self.intake.collecting() and self.hopper_count < HOPPER_CAPACITY

    def _on_intake(self):
        self.hopper_count += 1

    # ------------------------------------------------------------- logging
    def hopper_ball_poses(self):
        """Robot-relative poses for the model's HOPPER_CAPACITY ball components,
        stacked in layers; empty slots are hidden under the floor."""
        d = 2 * FUEL_RADIUS
        per_x = max(1, int(HOPPER_SIZE[0] / d))
        per_y = max(1, int(HOPPER_SIZE[1] / d))
        x0, y0, z0 = HOPPER_CORNER
        poses = []
        for i in range(HOPPER_CAPACITY):
            if i < self.hopper_count:
                poses.append(Pose3d(x0 + (i % per_x) * d, y0 + ((i // per_x) % per_y) * d,
                                    z0 + (i // (per_x * per_y)) * d, Rotation3d()))
            else:
                poses.append(HIDDEN_BALL)
        return poses

    def _publish(self):
        pose = self.drive.pose
        self.pubs["pose"].set(pose)
        self.pubs["modules"].set(self.drive.module_states())
        self._sim_field_pose.setPose(pose)
        values = {
            "Hopper Count": self.hopper_count,
            "Shots Fired": self.shooter.shots_fired,
            "Shots Scored": self.fuel.shots_scored,
            "Shots Missed": self.fuel.shots_missed,
            "Blue Hub Score": self.fuel.blue_hub.score,
            "Red Hub Score": self.fuel.red_hub.score,
            "Flywheel RPS": self.shooter.flywheel.rps,
            "Accelerator RPS": self.shooter.accelerator.rps,
            "Indexer RPS": self.shooter.indexer.rps,
            "Hood Rotations": self.shooter.hood.rotations,
            "Intake Pivot Rotations": self.intake.pivot.rotations,
            "Intake Roller RPS": self.intake.rollers.rps,
            "Launch Speed": self.shooter.launch_speed(),
            "Launch Angle": self.shooter.launch_angle_deg(),
            "Battery Voltage": self.battery_voltage,
            "Battery Current": self.battery_current,
            "Speed": math.hypot(self.drive.vx, self.drive.vy),
        }
        for name, value in values.items():
            self.numbers[name].set(value)
