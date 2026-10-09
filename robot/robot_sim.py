"""Simulation-only motor plants and vision world, stepped by physics.py.

Controller requests still go to Phoenix. Its simulated output voltage drives
each WPILib plant (or the swerve chassis model for the drive motors), and the
resulting sensors feed the normal subsystem code.
"""

import math
import numpy as np
import time

import const
import wpilib
from wpilib.simulation import DCMotorSim, RoboRioSim
from wpimath.geometry import Pose2d, Rotation2d, Rotation3d, Translation3d
from wpimath.kinematics import ChassisSpeeds
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


class SwerveChassis:
    """Rigid swerve chassis driven through traction-limited wheel contacts.

    Motor torque spins each wheel. Tread friction passes that force to the
    chassis only up to the wheel's friction circle (cof * normal load), and the
    same limit resists sideways scrub. Wheels therefore slip when overdriven,
    and a robot pushing a wall or hub stalls its wheels instead of letting the
    encoders run freely. Positions, velocities and impulses are SI units.
    """
    GRAVITY = 9.81
    SOLVER_ITERATIONS = 8

    def __init__(self, mass, moi, module_positions, wheel_radius, gearing, motor,
                 wheel_cof, wheel_inertia, friction_torque, viscous_drag,
                 current_limit, half_size, field_size, obstacles=()):
        self.mass = mass
        self.moi = moi
        self.module_positions = [(p.X(), p.Y()) for p in module_positions]
        self.wheel_radius = wheel_radius
        self.gearing = gearing
        self.motor = motor
        self.wheel_cof = wheel_cof
        self.wheel_inertia = wheel_inertia
        self.friction_torque = friction_torque
        self.viscous_drag = viscous_drag
        self.current_limit = current_limit
        self.half_size = half_size
        self.field_size = field_size
        self.obstacles = obstacles  # axis-aligned boxes: (center x, center y, half side)
        self.pose = Pose2d()
        self.vx = self.vy = self.omega = 0.0  # field-relative chassis velocity
        count = len(self.module_positions)
        self.wheel_velocity = [0.0] * count  # wheel rad/s
        self.wheel_position = [0.0] * count  # wheel radians
        self.currents = [0.0] * count       # motor stator amps
        self.contact_normals = []           # field-frame normals of touching walls/hubs

    def reset(self, pose):
        self.pose = pose
        self.vx = self.vy = self.omega = 0.0
        self.wheel_velocity = [0.0] * len(self.wheel_velocity)
        self.contact_normals = []
        self._resolve_obstacles()

    def robot_speeds(self):
        heading = self.pose.rotation()
        c, s = heading.cos(), heading.sin()
        return ChassisSpeeds(c * self.vx + s * self.vy, -s * self.vx + c * self.vy, self.omega)

    def step(self, dt, voltages, angles):
        """Advance by dt with each drive motor's applied voltage and module angle (rad)."""
        gearing, radius, inertia = self.gearing, self.wheel_radius, self.wheel_inertia
        motor = self.motor
        for i, volts in enumerate(voltages):
            motor_speed = self.wheel_velocity[i] * gearing
            current = (volts - motor_speed / motor.Kv) / motor.R
            current = max(-self.current_limit, min(self.current_limit, current))
            self.currents[i] = current
            torque = gearing * (motor.Kt * current - self.viscous_drag * motor_speed)
            speed = self.wheel_velocity[i] + torque / inertia * dt
            # Static drag can stop a wheel, but never reverse it.
            drop = gearing * self.friction_torque / inertia * dt
            self.wheel_velocity[i] = math.copysign(max(0.0, abs(speed) - drop), speed)

        heading = self.pose.rotation()
        c, s = heading.cos(), heading.sin()
        vx = c * self.vx + s * self.vy
        vy = -s * self.vx + c * self.vy
        omega = self.omega
        inv_mass, inv_moi = 1 / self.mass, 1 / self.moi
        wheel_term = radius * radius / inertia
        max_impulse = self.wheel_cof * self.mass * self.GRAVITY / len(voltages) * dt
        directions = [(math.cos(a), math.sin(a)) for a in angles]
        impulses = [[0.0, 0.0] for _ in voltages]
        # Walls/hubs touched last step, as robot-frame normals.
        blockers = [(c * nx + s * ny, -s * nx + c * ny) for nx, ny in self.contact_normals]
        # Sequential impulses: drive each contact's longitudinal slip and
        # lateral scrub to zero, clamped to its friction circle. Touching
        # obstacles cancel inward velocity inside the same loop, so wheels
        # against a wall stall (or slip) instead of rolling with the chassis.
        for _ in range(self.SOLVER_ITERATIONS):
            for nx, ny in blockers:
                inward = vx * nx + vy * ny
                if inward < 0:
                    vx -= inward * nx
                    vy -= inward * ny
            for i, ((mx, my), (ux, uy)) in enumerate(zip(self.module_positions, directions)):
                contact_x = vx - omega * my
                contact_y = vy + omega * mx
                slip_long = contact_x * ux + contact_y * uy - self.wheel_velocity[i] * radius
                slip_lat = contact_y * ux - contact_x * uy
                arm_long = mx * uy - my * ux
                arm_lat = mx * ux + my * uy
                old_long, old_lat = impulses[i]
                new_long = old_long - slip_long / (inv_mass + arm_long**2 * inv_moi + wheel_term)
                new_lat = old_lat - slip_lat / (inv_mass + arm_lat**2 * inv_moi)
                total = math.hypot(new_long, new_lat)
                if total > max_impulse:
                    new_long *= max_impulse / total
                    new_lat *= max_impulse / total
                impulses[i] = [new_long, new_lat]
                d_long, d_lat = new_long - old_long, new_lat - old_lat
                jx = d_long * ux - d_lat * uy
                jy = d_long * uy + d_lat * ux
                vx += jx * inv_mass
                vy += jy * inv_mass
                omega += (mx * jy - my * jx) * inv_moi
                self.wheel_velocity[i] -= d_long * radius / inertia

        self.vx, self.vy, self.omega = c * vx - s * vy, s * vx + c * vy, omega
        for i, speed in enumerate(self.wheel_velocity):
            self.wheel_position[i] += speed * dt
        self.pose = Pose2d(self.pose.X() + self.vx * dt, self.pose.Y() + self.vy * dt,
                           heading.rotateBy(Rotation2d(omega * dt)))
        self._resolve_obstacles()

    def _resolve_obstacles(self, touch=1e-6):
        """Push the bumpers out of walls/hubs and remove velocity into them.

        Records the normal of every obstacle touched (within `touch` metres)
        for the next step's solver.
        """
        half_x, half_y = self.half_size
        heading = self.pose.rotation()
        c, s = heading.cos(), heading.sin()
        x, y = self.pose.X(), self.pose.Y()
        extent_x = abs(c) * half_x + abs(s) * half_y
        extent_y = abs(s) * half_x + abs(c) * half_y
        length, width = self.field_size
        normals = []
        if x <= extent_x + touch:
            normals.append((1.0, 0.0))
        if x >= length - extent_x - touch:
            normals.append((-1.0, 0.0))
        if y <= extent_y + touch:
            normals.append((0.0, 1.0))
        if y >= width - extent_y - touch:
            normals.append((0.0, -1.0))
        x = max(extent_x, min(length - extent_x, x))
        y = max(extent_y, min(width - extent_y, y))
        for box_x, box_y, box_half in self.obstacles:
            dx, dy = x - box_x, y - box_y
            # Separating-axis test between the rotated bumper and the box.
            best = None
            for nx, ny in ((1.0, 0.0), (0.0, 1.0), (c, s), (-s, c)):
                robot_radius = half_x * abs(c * nx + s * ny) + half_y * abs(-s * nx + c * ny)
                box_radius = box_half * (abs(nx) + abs(ny))
                distance = dx * nx + dy * ny
                overlap = robot_radius + box_radius - abs(distance)
                if overlap < -touch:
                    best = None
                    break
                if best is None or overlap < best[0]:
                    sign = 1.0 if distance >= 0 else -1.0
                    best = (overlap, nx * sign, ny * sign)
            if best:
                overlap, nx, ny = best
                x += nx * max(overlap, 0.0)
                y += ny * max(overlap, 0.0)
                normals.append((nx, ny))
        for nx, ny in normals:
            into = self.vx * nx + self.vy * ny
            if into < 0:
                self.vx -= into * nx
                self.vy -= into * ny
        self.contact_normals = normals
        self.pose = Pose2d(x, y, heading)


class RobotSimulation:
    PERIOD = 0.02
    SUBSTEPS = 4
    MAX_SUBSTEP = 0.005  # keep integration fine when a robot loop runs late
    # Approximate mechanism inertias (kg m^2); tune from measurements/SysId.
    FLYWHEEL_INERTIA = 0.004
    ACCELERATOR_INERTIA = 0.001
    HOOD_GEARING = 360.0  # existing code uses rotor rotations as hood degrees
    INTAKE_DEPLOY_GEARING = 40.0
    BATTERY_RESISTANCE = 0.020
    STEER_INERTIA = 0.008
    # Drivetrain contact and losses. Tread friction (wheelCOF from the
    # PathPlanner robot config, the same value paths are generated with) caps
    # cornering and launch; the drive current limit usually binds first. The
    # drag terms are approximate gearbox and rolling losses at the motor shaft
    # (roughly 4 A static and 12 A at top speed).
    WHEEL_INERTIA = 0.003           # kg m^2 per wheel, including reflected rotor
    DRIVE_FRICTION_TORQUE = 0.07    # N m at the motor
    DRIVE_VISCOUS_DRAG = 0.0003     # N m per motor rad/s
    DRIVE_STATOR_LIMIT = 100.0      # A; matches the drive motors' stator limit
    FEED_INTERVAL = 0.06            # s between balls while the indexer feeds
    # Ball exit in robot coordinates. Shots leave along the robot's -Y axis,
    # which get_target_angle() points at the target; drivetrain distance math
    # measures from the same side of the robot.
    SHOOTER_EXIT = Translation3d(0, 0.27, 0.52)
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
        for module in robot.poseEstimator.modules:
            module.drive_motor.sim_state.orientation = (
                phoenix_sim.ChassisReference.CLOCKWISE_POSITIVE
                if module.drive_invert == signals.InvertedValue.CLOCKWISE_POSITIVE
                else phoenix_sim.ChassisReference.COUNTER_CLOCKWISE_POSITIVE)
            steer = MotorPlant(module.angle_motor, const.SWERVE_ANGLE_GEAR_RATIO, self.STEER_INERTIA,
                               inverted=module.angle_invert == signals.InvertedValue.CLOCKWISE_POSITIVE,
                               encoder=module.angle_encoder,
                               encoder_offset=-module.angle_offset.degrees() / 360)
            # Match the sensor's initial absolute reset, avoiding a first-tick jump.
            steer.model.setState(module.get_angle().radians(), 0.0)
            self.drive_plants.append(module.drive_motor)
            self.steer_plants.append(steer)
        hub = robot.fieldConstants.Hub
        hub_center = hub.topCenterPoint.toTranslation2d()
        field_length = robot.fieldConstants.fieldLength
        field_width = robot.fieldConstants.fieldWidth
        self.chassis = SwerveChassis(
            robot.robot_config.massKG, robot.robot_config.MOI,
            const.SWERVE_KINEMATICS.getModules(),
            const.SWERVE_WHEEL_CIRCUMFERENCE / math.tau, const.SWERVE_DRIVE_GEAR_RATIO,
            DCMotor.krakenX60FOC(1), robot.robot_config.moduleConfig.wheelCOF, self.WHEEL_INERTIA,
            self.DRIVE_FRICTION_TORQUE, self.DRIVE_VISCOUS_DRAG, self.DRIVE_STATOR_LIMIT,
            half_size=((const.DRIVE_BASE_LENGTH + 6) * 0.0254 / 2,
                       (const.DRIVE_BASE_WIDTH + 6) * 0.0254 / 2),
            field_size=(field_length, field_width),
            obstacles=((hub_center.X(), hub_center.Y(), hub.width / 2),
                       (field_length - hub_center.X(), field_width - hub_center.Y(), hub.width / 2)))
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
        """Place the simulated robot at an explicitly selected starting pose.

        A pose overlapping a hub (the no-auto default does) is pushed clear;
        read the result back from self.pose.
        """
        self.chassis.reset(pose)
        pose = self.pose = self.chassis.pose
        self.speeds = ChassisSpeeds()
        self.gyro_reference = pose.rotation()
        self.elapsed = 0.0
        self.robot.poseEstimator.gyro.sim_state.set_raw_yaw(0.0)
        self.robot.poseEstimator.gyro.set_yaw(pose.rotation().degrees())
        self.vision.resetRobotPose(pose)

    def get_field_speeds(self):
        return ChassisSpeeds.fromRobotRelativeSpeeds(self.speeds, self.pose.rotation())

    def _step_drivetrain(self, dt, enabled):
        """Steering plants set module angles; drive voltages push the chassis."""
        current = sum(max(0, steer.update(dt, self.voltage, enabled)) for steer in self.steer_plants)
        voltages = []
        for motor in self.drive_plants:
            motor.sim_state.set_supply_voltage(self.voltage)
            voltages.append(max(-self.voltage, min(self.voltage, motor.sim_state.motor_voltage))
                            if enabled else 0.0)
        chassis = self.chassis
        chassis.step(dt, voltages, [steer.model.getAngularPosition() for steer in self.steer_plants])
        gearing = const.SWERVE_DRIVE_GEAR_RATIO
        for index, motor in enumerate(self.drive_plants):
            scale = gearing * self.ODOMETRY_WHEEL_SCALES[index] / math.tau
            motor.sim_state.set_raw_rotor_position(chassis.wheel_position[index] * scale)
            motor.sim_state.set_rotor_velocity(chassis.wheel_velocity[index] * scale)
            # Stator current to battery current: P = V_applied * I_stator.
            current += abs(chassis.currents[index] * voltages[index]) / self.voltage
        self.pose = chassis.pose
        self.speeds = chassis.robot_speeds()
        return current

    def update(self, period=PERIOD):
        if period <= 0:
            return
        started = time.perf_counter()
        enabled = wpilib.DriverStation.isEnabled()
        self.step_count += 1
        self.elapsed += period
        substeps = max(self.SUBSTEPS, min(20, math.ceil(period / self.MAX_SUBSTEP)))
        dt = period / substeps
        for _ in range(substeps):
            self.current = self._step_drivetrain(dt, enabled) + sum(
                max(0, plant.update(dt, self.voltage, enabled)) for plant in self.plants)
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
            motor.sim_state.motor_voltage for motor in self.drive_plants])
        wpilib.SmartDashboard.putNumberArray("Sim/Wheel Speeds MPS", [
            speed * self.chassis.wheel_radius for speed in self.chassis.wheel_velocity])
        wpilib.SmartDashboard.putNumberArray("Sim/Drive Currents A", self.chassis.currents)
        wpilib.SmartDashboard.putNumber("Sim/Chassis Speed MPS", math.hypot(self.chassis.vx, self.chassis.vy))
        wpilib.SmartDashboard.putNumber("Sim/Physics Frame MS", (time.perf_counter() - started) * 1000)
        wpilib.SmartDashboard.putNumber("Sim/Step Duration MS", period * 1000)

    def _update_fuel(self, enabled, period=PERIOD):
        robot = self.robot
        shooter = robot.shooter
        feeding = (enabled and abs(shooter.get_fly_speed()) >= 5
                   and shooter.get_accelerator_speed() >= 5
                   and robot.hopper.get_speed() > 5 and robot.fuel_in_hopper > 0)
        self.shot_elapsed = self.shot_elapsed + period if feeding else 0.0
        while self.shot_elapsed >= self.FEED_INTERVAL and robot.fuel_in_hopper > 0:
            self.shot_elapsed -= self.FEED_INTERVAL
            # Invert the existing calibration tables using measured mechanism state.
            velocity = robot.drivetrain.vel_lookup_table
            angle = robot.drivetrain.angle_lookup_table
            speeds, fly_speeds = zip(*velocity.data)
            angles, hood_angles = zip(*angle.data)
            launch_speed = float(np.interp(abs(shooter.get_fly_speed()),
                                          (0, *fly_speeds), (0, *speeds)))
            launch_angle = float(np.interp(shooter.get_hood_position(), hood_angles[::-1], angles[::-1]))
            offset = self.SHOOTER_EXIT.rotateBy(Rotation3d(0, 0, self.pose.rotation().radians()))
            launch_point = Translation3d(self.pose.translation()) + offset
            # A late loop can feed several balls at once; each left the shooter
            # shot_elapsed seconds ago, so they must not spawn stacked together.
            launched = robot.fuel_sim.launchFuel(launch_speed, math.radians(launch_angle), 0,
                                                 launch_point, age=self.shot_elapsed)
            target = getattr(robot, "static_target", None)
            if launched is not None and target is not None:
                # Ball travel direction versus the aim point, including the
                # velocity inherited from a moving robot. Near 0 means on target.
                wanted = math.atan2(target.Y() - launch_point.Y(), target.X() - launch_point.X())
                error = math.atan2(launched.Y(), launched.X()) - wanted
                wpilib.SmartDashboard.putNumber("Sim/Shot Direction Error Deg",
                                                math.degrees(math.atan2(math.sin(error), math.cos(error))))
            robot.fuel_in_hopper -= 1
            # Transferring kinetic energy to a ball briefly slows the flywheels.
            energy = sum(0.5 * self.FLYWHEEL_INERTIA * p.model.getAngularVelocity()**2
                         for p in self.flywheel_plants)
            shot_energy = 0.5 * robot.fuel_sim.FUEL_MASS * launch_speed**2
            scale = math.sqrt(max(0, 1 - shot_energy / energy)) if energy > 0 else 0
            for plant in self.flywheel_plants:
                plant.model.setAngularVelocity(plant.model.getAngularVelocity() * scale)
        robot.fuel_sim.updateSim(period)
