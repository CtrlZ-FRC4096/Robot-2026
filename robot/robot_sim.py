"""Simulation-only motor plants and vision world, stepped by physics.py.

Controller requests still go to Phoenix. Its simulated output voltage drives
each WPILib plant, and the resulting sensors feed the normal subsystem code.
"""

import math
import numpy as np
import time

import const
import wpilib
from wpilib.simulation import DCMotorSim, RoboRioSim
from wpimath.geometry import Pose2d, Rotation2d, Rotation3d, Translation3d, Twist2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpimath.system.plant import DCMotor, LinearSystemId
from photonlibpy.simulation import VisionSystemSim, PhotonCameraSim, SimCameraProperties
from phoenix6 import signals, sim as phoenix_sim


def hopper_fuel_positions(width, length, height, diameter,
                          origin=Translation3d(-0.26, -0.28, 0.28)):
    """Supported, staggered sphere layers, filled from the floor upward.

    Offset layers sit in the hollows between four balls, producing a pile
    rather than a cube. Dimensions describe the inside of the hopper.
    """
    radius = diameter / 2
    nx, ny = int(width / diameter), int(length / diameter)
    if nx < 1 or ny < 1 or height < diameter:
        return []
    layer_spacing = diameter / math.sqrt(2) if nx > 1 and ny > 1 else diameter
    x0 = (width - (nx - 1) * diameter) / 2
    y0 = (length - (ny - 1) * diameter) / 2
    positions = []
    layer = 0
    while radius + layer * layer_spacing <= height - radius + 1e-9:
        offset = layer % 2 if nx > 1 and ny > 1 else 0
        centers = [(x0 + (x + offset / 2) * diameter,
                    y0 + (y + offset / 2) * diameter)
                   for x in range(nx - offset) for y in range(ny - offset)]
        centers.sort(key=lambda p: (p[0] - width / 2)**2 + (p[1] - length / 2)**2)
        positions.extend(origin + Translation3d(x, y, radius + layer * layer_spacing)
                         for x, y in centers)
        layer += 1
    return positions


class MotorPlant:
    def __init__(self, motor, gearing, inertia, encoder=None, limits=None,
                 inverted=False, encoder_offset=0.0, sensor_scale=1.0):
        self.motor = motor
        self.gearing = gearing
        self.encoder = encoder
        self.encoder_offset = encoder_offset
        self.sensor_scale = sensor_scale
        motor.sim_state.orientation = (
            phoenix_sim.ChassisReference.CLOCKWISE_POSITIVE if inverted
            else phoenix_sim.ChassisReference.COUNTER_CLOCKWISE_POSITIVE
        )
        self.limits = limits  # output rotations, not rotor rotations
        gearbox = DCMotor.krakenX60(1)
        self.model = DCMotorSim(
            LinearSystemId.DCMotorSystem(gearbox, inertia, gearing), gearbox
        )

    def update(self, dt, voltage, enabled):
        sim = self.motor.sim_state
        sim.set_supply_voltage(voltage)
        applied_voltage = max(-voltage, min(voltage, sim.motor_voltage)) if enabled else 0.0
        self.model.setInputVoltage(applied_voltage)
        self.model.update(dt)
        position = self.model.getAngularPositionRotations()
        if self.limits:
            bounded = max(self.limits[0], min(self.limits[1], position))
            if bounded != position:
                self.model.setState(bounded * math.tau, 0.0)
                position = bounded
        velocity = self.model.getAngularVelocityRPM() / 60
        sim.set_raw_rotor_position(position * self.gearing * self.sensor_scale)
        sim.set_rotor_velocity(velocity * self.gearing * self.sensor_scale)
        if self.encoder:
            self.encoder.sim_state.set_supply_voltage(voltage)
            self.encoder.sim_state.set_raw_position(position - self.encoder_offset)
            self.encoder.sim_state.set_velocity(velocity)
        # DCMotorSim reports winding current. Convert input power to battery
        # current; at partial duty cycle these are substantially different.
        return max(0, self.model.getCurrentDraw()) * abs(applied_voltage) / voltage


class RobotSimulation:
    PERIOD = 0.02
    SUBSTEPS = 4
    # Approximate mechanism inertias (kg m^2); tune from measurements/SysId.
    FLYWHEEL_INERTIA = 0.004
    ACCELERATOR_INERTIA = 0.001
    HOOD_GEARING = 360.0  # existing code uses rotor rotations as hood degrees
    INTAKE_DEPLOY_GEARING = 40.0
    BATTERY_RESISTANCE = 0.020
    STEER_INERTIA = 0.008
    # Imperfect wheel-radius calibration and gyro readings; physical motion
    # still comes from the unmodified plants, never these reported sensors.
    ODOMETRY_WHEEL_SCALES = (1.012, 1.008, 1.010, 1.014)
    GYRO_DRIFT_DPS = 0.015
    GYRO_NOISE_STD_DEG = 0.03

    def __init__(self, robot):
        self.robot = robot
        self.pose = robot.poseEstimator.curEstPose
        self.speeds = ChassisSpeeds()
        self.voltage = 12.0
        self.current = 0.0
        self.shot_elapsed = 0.0
        self.step_count = 0
        self.elapsed = 0.0
        self.sensor_rng = np.random.default_rng(4096)
        self.plants = []
        self.drive_plants = []
        self.steer_plants = []
        self.flywheel_plants = []
        # Reflect one quarter of the configured robot mass to each wheel.
        mass = robot.robot_config.massKG
        radius = const.SWERVE_WHEEL_CIRCUMFERENCE / math.tau
        for index, module in enumerate(robot.poseEstimator.modules):
            drive = MotorPlant(module.drive_motor, const.SWERVE_DRIVE_GEAR_RATIO,
                               mass / 4 * radius**2,
                               inverted=module.drive_invert == signals.InvertedValue.CLOCKWISE_POSITIVE,
                               sensor_scale=self.ODOMETRY_WHEEL_SCALES[index])
            steer = MotorPlant(module.angle_motor, const.SWERVE_ANGLE_GEAR_RATIO, self.STEER_INERTIA,
                               inverted=module.angle_invert == signals.InvertedValue.CLOCKWISE_POSITIVE,
                               encoder=module.angle_encoder,
                               encoder_offset=-module.angle_offset.degrees() / 360)
            # Match the sensor's initial absolute reset, avoiding a first-tick jump.
            steer.model.setState(module.get_angle().radians(), 0.0)
            self.drive_plants.append(drive)
            self.steer_plants.append(steer)
            self.plants.extend((drive, steer))
        shooter = robot.shooter
        for motor in (shooter.left_up_fly_motor, shooter.left_down_fly_motor,
                      shooter.right_fly_motor):
            plant = MotorPlant(motor, 1, self.FLYWHEEL_INERTIA)
            self.flywheel_plants.append(plant)
            self.plants.append(plant)
        self.plants.extend((
            MotorPlant(shooter.accelerator_motor, 1, self.ACCELERATOR_INERTIA),
            MotorPlant(shooter.hood_motor, self.HOOD_GEARING, 0.03,
                       limits=(0, 45 / 360)),
            MotorPlant(robot.intake.left_intake_motor, 1, 0.001),
            MotorPlant(robot.intake.right_intake_motor, 1, 0.001),
            MotorPlant(robot.intake.deploy_motor, self.INTAKE_DEPLOY_GEARING, 0.04,
                       encoder=robot.intake.deploy_cancoder, limits=(-0.35, 0),
                       inverted=True, encoder_offset=0.8),
            MotorPlant(robot.hopper.indexer_motor, 1, 0.002),
        ))
        self.vision = VisionSystemSim("robot")
        self.vision.addAprilTags(robot.poseEstimator.tag_layout)
        self.cameras = []
        for wrapper in robot.poseEstimator.cams:
            props = SimCameraProperties()
            props.setCalibrationFromFOV(960, 720, Rotation2d.fromDegrees(90))
            props.setCalibError(0.25, 0.08)
            props.setFPS(30)
            # photonlibpy's setAvgLatency currently assigns vgLatency instead
            # of avgLatency. The simulator consumes this seconds-valued field.
            props.avgLatency = 0.035
            props.setLatencyStdDev(0.005)
            camera = PhotonCameraSim(wrapper.cam, props, robot.poseEstimator.tag_layout,
                                     maxSightRange=5.0)
            # photonlibpy simulates detections; its image-stream methods are
            # unimplemented (unlike the Java library).
            self.vision.addCamera(camera, wrapper.robotToCam)
            self.cameras.append(camera)
        self.reset_pose(self.pose)

    def reset_pose(self, pose):
        """Place the simulated robot at an explicitly selected starting pose."""
        self.pose = pose
        self.speeds = ChassisSpeeds()
        self.gyro_reference = pose.rotation()
        self.elapsed = 0.0
        self.robot.poseEstimator.gyro.sim_state.set_raw_yaw(0.0)
        self.robot.poseEstimator.gyro.set_yaw(pose.rotation().degrees())
        self.vision.resetRobotPose(pose)

    def get_field_speeds(self):
        return ChassisSpeeds.fromRobotRelativeSpeeds(self.speeds, self.pose.rotation())

    def update(self, period=PERIOD):
        if period <= 0:
            return
        started = time.perf_counter()
        enabled = wpilib.DriverStation.isEnabled()
        self.step_count += 1
        self.elapsed += period
        dt = period / self.SUBSTEPS
        for _ in range(self.SUBSTEPS):
            self.current = sum(max(0, plant.update(dt, self.voltage, enabled))
                               for plant in self.plants)
            states = tuple(SwerveModuleState(
                drive.model.getAngularVelocityRPM() / 60 * const.SWERVE_WHEEL_CIRCUMFERENCE,
                Rotation2d(steer.model.getAngularPosition()),
            ) for drive, steer in zip(self.drive_plants, self.steer_plants))
            self.speeds = const.SWERVE_KINEMATICS.toChassisSpeeds(states)
            next_pose = self.pose.exp(Twist2d(self.speeds.vx * dt,
                                            self.speeds.vy * dt, self.speeds.omega * dt))
            # Bumpers stay inside the field; wheel encoders may still turn at a wall.
            half_x = (const.DRIVE_BASE_LENGTH + 6) * 0.0254 / 2
            half_y = (const.DRIVE_BASE_WIDTH + 6) * 0.0254 / 2
            angle = next_pose.rotation()
            extent_x = abs(angle.cos()) * half_x + abs(angle.sin()) * half_y
            extent_y = abs(angle.sin()) * half_x + abs(angle.cos()) * half_y
            self.pose = Pose2d(
                max(extent_x, min(self.robot.fieldConstants.fieldLength - extent_x, next_pose.X())),
                max(extent_y, min(self.robot.fieldConstants.fieldWidth - extent_y, next_pose.Y())),
                angle,
            )
        gyro = self.robot.poseEstimator.gyro.sim_state
        gyro.set_supply_voltage(self.voltage)
        gyro_error = self.elapsed * self.GYRO_DRIFT_DPS + self.sensor_rng.normal(0, self.GYRO_NOISE_STD_DEG)
        gyro.set_raw_yaw((self.pose.rotation() - self.gyro_reference).degrees() + gyro_error)
        gyro.set_angular_velocity_z(math.degrees(self.speeds.omega))
        loaded_voltage = max(6.0, 12.0 - self.current * self.BATTERY_RESISTANCE)
        self.voltage += (1 - math.exp(-period / 0.09)) * (loaded_voltage - self.voltage)
        RoboRioSim.setVInVoltage(self.voltage)
        self.vision.update(self.pose)
        self._update_fuel(enabled, period)
        wpilib.SmartDashboard.putNumberArray("Sim/Ground Truth Pose", [
            self.pose.X(), self.pose.Y(), self.pose.rotation().degrees()])
        wpilib.SmartDashboard.putNumber("Sim/Battery Voltage", self.voltage)
        wpilib.SmartDashboard.putNumber("Sim/Current Amps", self.current)
        wpilib.SmartDashboard.putNumber("Sim/Step Count", self.step_count)
        wpilib.SmartDashboard.putBoolean("Sim/Enabled", enabled)
        wpilib.SmartDashboard.putNumberArray("Sim/Drive Motor Volts", [
            plant.motor.sim_state.motor_voltage for plant in self.drive_plants])
        wpilib.SmartDashboard.putNumberArray("Sim/Wheel Speeds MPS", [
            state.speed for state in states])
        wpilib.SmartDashboard.putNumber("Sim/Physics Frame MS", (time.perf_counter() - started) * 1000)
        wpilib.SmartDashboard.putNumber("Sim/Step Duration MS", period * 1000)

    def _update_fuel(self, enabled, period=PERIOD):
        robot = self.robot
        shooter = robot.shooter
        feeding = (enabled and abs(shooter.get_fly_speed()) >= 5
                   and shooter.get_accelerator_speed() >= 5
                   and robot.hopper.get_speed() > 5 and robot.fuel_in_hopper > 0)
        self.shot_elapsed = self.shot_elapsed + period if feeding else 0.0
        while self.shot_elapsed >= 0.06 and robot.fuel_in_hopper > 0:
            self.shot_elapsed -= 0.06
            # Invert the existing calibration tables using measured mechanism state.
            velocity = robot.drivetrain.vel_lookup_table
            angle = robot.drivetrain.angle_lookup_table
            speeds, fly_speeds = zip(*velocity.data)
            angles, hood_angles = zip(*angle.data)
            launch_speed = float(np.interp(abs(shooter.get_fly_speed()),
                                          (0, *fly_speeds), (0, *speeds)))
            launch_angle = float(np.interp(shooter.get_hood_position(), hood_angles[::-1], angles[::-1]))
            offset = Translation3d(0, 0.27, 0.52).rotateBy(Rotation3d(0, 0, self.pose.rotation().radians()))
            robot.fuel_sim.launchFuel(launch_speed, math.radians(launch_angle), 0,
                                     Translation3d(self.pose.translation()) + offset)
            robot.fuel_in_hopper -= 1
            # Transferring kinetic energy to a ball briefly slows the flywheels.
            energy = sum(0.5 * self.FLYWHEEL_INERTIA * p.model.getAngularVelocity()**2
                         for p in self.flywheel_plants)
            shot_energy = 0.5 * robot.fuel_sim.FUEL_MASS * launch_speed**2
            scale = math.sqrt(max(0, 1 - shot_energy / energy)) if energy > 0 else 0
            for plant in self.flywheel_plants:
                plant.model.setAngularVelocity(plant.model.getAngularVelocity() * scale)
        robot.fuel_sim.updateSim(period)
