"""Swerve drive physics ported from maple-sim (SwerveDriveSimulation,
SwerveModuleSimulation, GyroSimulation, SimulatedArena timing).

Per 20 ms robot loop the drive runs 5 sub-ticks of 4 ms. In each sub-tick:
  1. chassis friction pulls the chassis velocity toward what the wheels
     allow (this kills sideways sliding), capped at mu*m*g;
  2. chassis friction torque does the same for rotation;
  3. each module turns its steer motor and pushes the chassis along its wheel
     with the drive motor torque, capped at the wheel's grip (mu*m*g/4);
     past that the wheel skids and its encoder reads too fast;
  4. the gyro integrates the true rotation rate with Pigeon 2 noise.
dyn4j's job (integration, damping 1.4/s, contacts with the field) is done by
SwerveDriveSim.step itself, with one rigid rectangle for the bumpers.

Robot numbers (mass, MOI, bumpers, wheel COF) come from PathPlanner's
deploy/pathplanner/settings.json, the same ones the auto paths use.
"""

import collections
import json
import math
import random
from pathlib import Path

from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import SwerveModuleState
from wpimath.system.plant import DCMotor

import const
from simulation import field
from simulation.motor_sim import TAU, SimTalon

GRAVITY = 9.8  # maple-sim uses 9.8 for wheel normal force
LINEAR_DAMPING = 1.4
ANGULAR_DAMPING = 1.4
FRICTION_FORCE_GAIN = 3.0
FRICTION_TORQUE_GAIN = 1.0
DRIVE_FRICTION_VOLTAGE = 0.1
STEER_FRICTION_VOLTAGE = 0.2
STEER_MOI = 0.03  # kg m^2 at the steer axis (maple-sim COTS default)
SETTINGS = Path(__file__).resolve().parent.parent / "deploy" / "pathplanner" / "settings.json"


class GyroSim:
    """maple-sim GyroSimulation.ofPigeon2(): 2% rate noise, a jump on hard
    impacts (angular acceleration over 500 rad/s^2), and maple-sim's still-drift
    term (nominally 0.5 deg/30 s, but its per-step formula gives far less;
    kept as in maple-sim). Most heading error comes from the rate noise
    while turning and from impacts.

    The contacts here are perfectly rigid and change the spin within one
    4 ms sub-tick, where real bumpers squash over tens of milliseconds, so the
    angular acceleration is measured over IMPACT_WINDOW sub-ticks (one robot
    loop). Otherwise every scrape on the hub counts as a hard hit and the
    gyro drifts several degrees a minute."""

    IMPACT_WINDOW = 5

    def __init__(self, drift_deg_per_30s=0.5, velocity_std_fraction=0.02):
        self.drift = drift_deg_per_30s
        self.velocity_std = velocity_std_fraction
        self.reading = 0.0  # rad, accumulated measured yaw
        self.measured_velocity = 0.0
        self._recent_velocities = collections.deque([0.0] * self.IMPACT_WINDOW,
                                                    maxlen=self.IMPACT_WINDOW)

    def step(self, true_velocity, dt):
        accel = (true_velocity - self._recent_velocities[0]) / (self.IMPACT_WINDOW * dt)
        self._recent_velocities.append(true_velocity)
        # A hit stays in the window for IMPACT_WINDOW sub-ticks; spread its jump over them.
        impact = (-math.copysign(abs(accel) / 500 * math.radians(1), accel) / self.IMPACT_WINDOW
                  if abs(accel) > 500 else 0.0)
        self.measured_velocity = random.gauss(true_velocity, self.velocity_std * abs(true_velocity))
        self.reading += (impact + self.measured_velocity * dt
                         + math.radians(random.gauss(0.0, self.drift / 30 * dt)))


class ModuleSim:
    def __init__(self, module, translation, wheel_radius, drive_gearing, steer_gearing):
        self.module = module
        self.x, self.y = translation.X(), translation.Y()
        self.wheel_radius = wheel_radius
        self.drive_gearing = drive_gearing
        self.steer_gearing = steer_gearing
        self.drive = SimTalon(module.drive_motor, DCMotor.krakenX60FOC(1))
        self.steer = SimTalon(module.angle_motor, DCMotor.krakenX60(1))
        # The robot code zeroed the steer motor from the CANcoder before the
        # physics existed, so take the angle it believes as the true one.
        reported = module.angle_motor.get_position().refresh().value
        self.steer_angle = reported / steer_gearing * TAU
        self.steer.rotor_offset = -reported
        self.steer_velocity = 0.0
        self.wheel_speed = 0.0     # rad/s, what the drive encoder sees
        self.wheel_position = 0.0  # rad
        motor = self.drive.motor
        self.drive_friction = motor.Kt * DRIVE_FRICTION_VOLTAGE / motor.R
        self.steer_friction = self.steer.motor.Kt * STEER_FRICTION_VOLTAGE / self.steer.motor.R

    def sample(self, bus_voltage):
        self.drive.sample(self.wheel_speed * self.drive_gearing, bus_voltage)
        self.steer.sample(self.steer_velocity * self.steer_gearing, bus_voltage)

    def step_steer(self, dt):
        """maple-sim MapleMotorSim/SimMotorState.step for the steer."""
        g = self.steer_gearing
        current = self.steer.current(self.steer_velocity * g)
        self.steer_velocity += self.steer.motor.torque(current) * g / STEER_MOI * dt
        friction_dv = math.copysign(self.steer_friction, -self.steer_velocity) / STEER_MOI * dt
        if (self.steer_velocity + friction_dv) * self.steer_velocity <= 0:
            self.steer_velocity = 0.0
        else:
            self.steer_velocity += friction_dv
        self.steer_angle += self.steer_velocity * dt

    def propelling_force(self, ground_vx, ground_vy, heading, grip):
        """maple-sim updateSimulationSubTickGetModuleForce: force along the
        wheel (N) and the new wheel speed. ground_v* is this module's floor
        velocity in field coordinates."""
        g, r = self.drive_gearing, self.wheel_radius
        motor = self.drive.motor
        current = self.drive.current(self.wheel_speed * g)
        torque = motor.torque(current) * g
        torque = (torque - math.copysign(self.drive_friction, torque)
                  if abs(torque) >= self.drive_friction else 0.0)
        force = torque / r
        skidding = abs(force) > grip
        if skidding:
            force = math.copysign(grip, force)
        facing = heading + self.steer_angle
        projected = ground_vx * math.cos(facing) + ground_vy * math.sin(facing)
        self.wheel_speed = projected / r
        if skidding:
            # The wheel spins at the motor's speed under that load: odometry slips.
            load_current = force * r / (g * motor.Kt)
            free = (self.drive.last_voltage - load_current * motor.R) * motor.Kv / g
            self.wheel_speed = 0.5 * self.wheel_speed + 0.5 * free
        return force, facing

    def free_speed(self):
        """Wheel ground speed if it spun freely at the applied voltage (m/s)."""
        motor = self.drive.motor
        friction = min(abs(self.drive.last_voltage), DRIVE_FRICTION_VOLTAGE / self.drive_gearing)
        volts = self.drive.last_voltage - math.copysign(friction, self.drive.last_voltage)
        return volts * motor.Kv / self.drive_gearing * self.wheel_radius

    def write(self, dt_period):
        self.drive.write(self.wheel_position * self.drive_gearing / TAU,
                         self.wheel_speed * self.drive_gearing / TAU)
        self.steer.write(self.steer_angle * self.steer_gearing / TAU,
                         self.steer_velocity * self.steer_gearing / TAU)
        # Keep the CANcoder consistent with the wheel angle (same zero as the code).
        offset = self.module.angle_offset.degrees()
        self.module.angle_encoder.sim_state.set_raw_position(
            (math.degrees(self.steer_angle) + offset) / 360)
        self.module.angle_encoder.sim_state.set_velocity(self.steer_velocity / TAU)

    def state(self):
        return SwerveModuleState(self.wheel_speed * self.wheel_radius, Rotation2d(self.steer_angle))

    def supply_current(self):
        return self.drive.supply_current() + self.steer.supply_current()


class SwerveDriveSim:
    SUB_TICKS = 5

    def __init__(self, modules, pigeon, start_pose: Pose2d):
        settings = json.loads(SETTINGS.read_text())
        self.mass = settings["robotMass"]
        self.length = settings["robotLength"]  # bumper-to-bumper along robot x
        self.width = settings["robotWidth"]
        self.moi = self.mass * (self.length ** 2 + self.width ** 2) / 12  # dyn4j rectangle
        self.cof = settings["wheelCOF"]
        wheel_radius = const.SWERVE_WHEEL_CIRCUMFERENCE / TAU
        translations = const.SWERVE_KINEMATICS.getModules()
        self.modules = [ModuleSim(m, t, wheel_radius, const.SWERVE_DRIVE_GEAR_RATIO,
                                  const.SWERVE_ANGLE_GEAR_RATIO)
                        for m, t in zip(modules, translations)]
        self.kinematics = const.SWERVE_KINEMATICS
        self.pigeon = pigeon
        self.gyro = GyroSim()
        self._gyro_written = 0.0

        motor = DCMotor.krakenX60FOC(1)
        self.max_linear_speed = motor.freeSpeed / const.SWERVE_DRIVE_GEAR_RATIO * wheel_radius
        track_x = max(m.x for m in self.modules) - min(m.x for m in self.modules)
        track_y = max(m.y for m in self.modules) - min(m.y for m in self.modules)
        self.max_angular_speed = self.max_linear_speed / math.hypot(track_x, track_y)
        self.module_radius = math.hypot(self.modules[0].x, self.modules[0].y)
        self.obstacles = field.obstacles()

        self.x, self.y, self.heading = start_pose.X(), start_pose.Y(), start_pose.rotation().radians()
        self.vx = self.vy = self.omega = 0.0
        self._previous_module_speeds = (0.0, 0.0)

    # ------------------------------------------------------------- state
    @property
    def pose(self):
        return Pose2d(self.x, self.y, Rotation2d(self.heading))

    def set_pose(self, pose: Pose2d):
        """Teleport (like maple-sim's setSimulationWorldPose); the gyro keeps
        whatever yaw the robot code set."""
        self.x, self.y, self.heading = pose.X(), pose.Y(), pose.rotation().radians()
        self.vx = self.vy = self.omega = 0.0
        self._previous_module_speeds = (0.0, 0.0)
        self._resolve_contacts()

    def module_states(self):
        return [m.state() for m in self.modules]

    def supply_current(self):
        return sum(m.supply_current() for m in self.modules)

    # -------------------------------------------------------------- step
    def step(self, period, bus_voltage):
        for module in self.modules:
            module.sample(bus_voltage)
        dt = period / self.SUB_TICKS
        for _ in range(self.SUB_TICKS):
            self._sub_tick(dt)
        for module in self.modules:
            module.write(period)
        self.pigeon.sim_state.add_yaw(math.degrees(self.gyro.reading - self._gyro_written))
        self._gyro_written = self.gyro.reading
        self.pigeon.sim_state.set_angular_velocity_z(math.degrees(self.gyro.measured_velocity))

    def _sub_tick(self, dt):
        cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
        fx = fy = torque = 0.0

        # 1-2. Chassis friction toward the speeds the wheels allow.
        module_speeds = self.kinematics.toChassisSpeeds(tuple(self.module_states()))
        grip_total = self.cof * self.mass * GRAVITY
        chassis_vx_robot = self.vx * cos_h + self.vy * sin_h
        chassis_vy_robot = -self.vx * sin_h + self.vy * cos_h
        dvx_r = module_speeds.vx - chassis_vx_robot
        dvy_r = module_speeds.vy - chassis_vy_robot
        dvx, dvy = dvx_r * cos_h - dvy_r * sin_h, dvx_r * sin_h + dvy_r * cos_h
        dv = math.hypot(dvx, dvy)
        if dv > 1e-9:
            magnitude = min(FRICTION_FORCE_GAIN * grip_total * dv, grip_total)
            fx, fy = dvx / dv * magnitude, dvy / dv * magnitude
        # Centripetal term: the wheels also turn the velocity's direction.
        mvx = module_speeds.vx * cos_h - module_speeds.vy * sin_h
        mvy = module_speeds.vx * sin_h + module_speeds.vy * cos_h
        pvx, pvy = self._previous_module_speeds
        previous_speed = math.hypot(pvx, pvy)
        if previous_speed > 1e-6 and math.hypot(mvx, mvy) > 1e-6:
            turn = math.atan2(mvy, mvx) - math.atan2(pvy, pvx)
            turn = math.atan2(math.sin(turn), math.cos(turn))
            centripetal = previous_speed * (turn / dt) * self.mass
            direction = math.atan2(pvy, pvx) + math.pi / 2
            fx += centripetal * math.cos(direction)
            fy += centripetal * math.sin(direction)
        self._previous_module_speeds = (mvx, mvy)
        total = math.hypot(fx, fy)
        if total > grip_total:
            fx, fy = fx / total * grip_total, fy / total * grip_total

        torque_grip = self.cof * self.mass * GRAVITY / 4 * self.module_radius * 4
        free_states = tuple(SwerveModuleState(m.free_speed(), Rotation2d(m.steer_angle))
                            for m in self.modules)
        desired_omega = self.kinematics.toChassisSpeeds(free_states).omega
        if (abs(self.omega) / self.max_angular_speed < 0.01
                and abs(desired_omega) / self.max_angular_speed < 0.02):
            self.omega = 0.0
        else:
            d_omega = module_speeds.omega - self.omega
            torque += math.copysign(min(FRICTION_TORQUE_GAIN * torque_grip * abs(d_omega), torque_grip),
                                    d_omega)

        # 3. Module propelling forces at each module's world position.
        grip = self.cof * self.mass * GRAVITY / len(self.modules)
        for module in self.modules:
            module.step_steer(dt)
            rx = module.x * cos_h - module.y * sin_h
            ry = module.x * sin_h + module.y * cos_h
            ground_vx = self.vx - self.omega * ry
            ground_vy = self.vy + self.omega * rx
            force, facing = module.propelling_force(ground_vx, ground_vy, self.heading, grip)
            mfx, mfy = force * math.cos(facing), force * math.sin(facing)
            fx += mfx
            fy += mfy
            torque += rx * mfy - ry * mfx
            module.wheel_position += module.wheel_speed * dt

        # 4. Integrate (dyn4j: symplectic Euler, then damping, then contacts).
        self.vx += fx / self.mass * dt
        self.vy += fy / self.mass * dt
        self.omega += torque / self.moi * dt
        linear_keep = max(0.0, 1 - dt * LINEAR_DAMPING)
        self.vx *= linear_keep
        self.vy *= linear_keep
        self.omega *= max(0.0, 1 - dt * ANGULAR_DAMPING)
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.heading += self.omega * dt
        self._resolve_contacts()
        self.gyro.step(self.omega, dt)

    # ---------------------------------------------------------- contacts
    def _corners(self):
        c, s = math.cos(self.heading), math.sin(self.heading)
        hx, hy = self.length / 2, self.width / 2
        return [(self.x + sx * hx * c - sy * hy * s, self.y + sx * hx * s + sy * hy * c)
                for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]

    def _contact(self, box):
        """Separating-axis test of the bumper rectangle against an obstacle
        box. Returns (normal pointing out of the box, depth, contact point)."""
        min_x, min_y, max_x, max_y = box
        bcx, bcy = (min_x + max_x) / 2, (min_y + max_y) / 2
        bhx, bhy = (max_x - min_x) / 2, (max_y - min_y) / 2
        c, s = math.cos(self.heading), math.sin(self.heading)
        hx, hy = self.length / 2, self.width / 2
        dx, dy = bcx - self.x, bcy - self.y
        best = None
        for ax, ay, robot_face in ((1.0, 0.0, False), (0.0, 1.0, False), (c, s, True), (-s, c, True)):
            robot_extent = hx * abs(c * ax + s * ay) + hy * abs(-s * ax + c * ay)
            box_extent = bhx * abs(ax) + bhy * abs(ay)
            distance = dx * ax + dy * ay
            overlap = robot_extent + box_extent - abs(distance)
            if overlap <= 0:
                return None
            if best is None or overlap < best[2]:
                sign = -1.0 if distance > 0 else 1.0
                best = (sign * ax, sign * ay, overlap, robot_face)
        nx, ny, depth, robot_face = best
        if not robot_face:
            # Obstacle face: the robot corner deepest into the box.
            point = min(self._corners(), key=lambda p: p[0] * nx + p[1] * ny)
        else:
            # Robot face: the box corner deepest into the robot, kept on the bumper.
            corners = [(min_x, min_y), (min_x, max_y), (max_x, min_y), (max_x, max_y)]
            px, py = max(corners, key=lambda p: p[0] * nx + p[1] * ny)
            lx = (px - self.x) * c + (py - self.y) * s
            ly = -(px - self.x) * s + (py - self.y) * c
            lx, ly = max(-hx, min(hx, lx)), max(-hy, min(hy, ly))
            point = (self.x + lx * c - ly * s, self.y + lx * s + ly * c)
        return nx, ny, depth, point

    def _resolve_contacts(self, iterations=3):
        for _ in range(iterations):
            touching = False
            for box in self.obstacles:
                hit = self._contact(box)
                if hit is None:
                    continue
                touching = True
                nx, ny, depth, (px, py) = hit
                self.x += nx * depth
                self.y += ny * depth
                self._contact_impulse(nx, ny, px + nx * depth, py + ny * depth)
            if not touching:
                return

    def _contact_impulse(self, nx, ny, px, py):
        rx, ry = px - self.x, py - self.y
        vx = self.vx - self.omega * ry
        vy = self.vy + self.omega * rx
        vn = vx * nx + vy * ny
        if vn >= 0:
            return
        restitution = field.CONTACT_RESTITUTION if -vn > field.RESTITUTION_MIN_SPEED else 0.0
        rn = rx * ny - ry * nx
        j = -(1 + restitution) * vn / (1 / self.mass + rn * rn / self.moi)
        tx, ty = -ny, nx
        vt = vx * tx + vy * ty
        rt = rx * ty - ry * tx
        jt = -vt / (1 / self.mass + rt * rt / self.moi)
        jt = max(-field.CONTACT_FRICTION * j, min(field.CONTACT_FRICTION * j, jt))
        ix, iy = j * nx + jt * tx, j * ny + jt * ty
        self.vx += ix / self.mass
        self.vy += iy / self.mass
        self.omega += (rx * iy - ry * ix) / self.moi
