"""Shooter, indexer and intake mechanisms, and the fuel that moves through them.

Each motor group is a Mechanism driven by the voltage Phoenix computes from
the robot code's requests, so spin-up, spin-down and hood/pivot travel take
real time. Fuel is collected only while the intake is actually deployed and
its rollers are pulling in, and a ball leaves the shooter at the speed and
angle the *simulated* flywheel and hood give it: the inverse of the robot's
own calibration tables (launch speed -> flywheel rps, launch angle -> hood).
A slow flywheel therefore shoots short, as on the real robot.

Mechanism masses and gear ratios that aren't in the code are estimates;
they're the constants at the top of this file.
"""

import math
import random

from wpimath.system.plant import DCMotor

from simulation.motor_sim import TAU, Mechanism, SimTalon

# -- estimates (tune against the real robot) --------------------------------
FLYWHEEL_MOI = 0.008          # kg m^2, all three flywheel motors on one shaft
ACCELERATOR_MOI = 0.0008
INDEXER_MOI = 0.002
INTAKE_ROLLER_MOI = 0.001
HOOD_ROTOR_MOI = 0.0004       # hood reflected to its motor (positions are rotor turns)
HOOD_MAX_ROTATIONS = 48.0
INTAKE_PIVOT_GEARING = 25.0
INTAKE_PIVOT_MOI = 0.2
INTAKE_STOWED = -0.04         # CANcoder rotations
INTAKE_DEPLOYED_BELOW = -0.25  # pivot must be at least this far down to collect
INTAKE_PIVOT_RANGE = (-0.36, 0.0)

HOPPER_CAPACITY = 24
MAX_FEED_RATE = 8.0           # balls/s with the indexer at full speed
INDEXER_FULL_RPS = 80.0
MIN_INDEXER_RPS = 10.0        # below this nothing moves into the shooter
MIN_ACCELERATOR_RPS = 20.0    # below this fed balls jam instead of launching
SHOT_ENERGY_LOSS_FACTOR = 1.5  # flywheel energy lost per ball / ball kinetic energy

# Shot dispersion (1 sigma).
SPEED_NOISE_FRACTION = 0.015
PITCH_NOISE_DEG = 0.75
YAW_NOISE_DEG = 0.75

# Exit point on the robot and direction: the drivetrain aims the robot's -Y
# side at the target (Drivetrain.get_target_angle), shooting from +0.196 m Y
# at 0.52 m high (the numbers the shot tables were computed for).
SHOOTER_POSITION = (0.0, 0.196, 0.52)
SHOOTER_YAW = -math.pi / 2


def _interpolate(table, x):
    """Piecewise-linear lookup in sorted (x, y) pairs, extrapolating the ends."""
    if x <= table[0][0]:
        (x1, y1), (x2, y2) = table[0], table[1]
    elif x >= table[-1][0]:
        (x1, y1), (x2, y2) = table[-2], table[-1]
    else:
        for (x1, y1), (x2, y2) in zip(table, table[1:]):
            if x1 <= x <= x2:
                break
    return y1 + (y2 - y1) * (x - x1) / (x2 - x1)


class Shooter:
    def __init__(self, shooter, hopper, robot_lookup_tables):
        foc, plain = DCMotor.krakenX60FOC(1), DCMotor.krakenX60(1)
        self.flywheel = Mechanism(
            SimTalon(shooter.left_up_fly_motor, foc),
            gearing=1.0, moi=FLYWHEEL_MOI, friction_torque=0.05, viscous=0.0002,
            followers=[(SimTalon(shooter.right_fly_motor, foc), -1),
                       (SimTalon(shooter.left_down_fly_motor, foc), +1)])
        self.accelerator = Mechanism(
            SimTalon(shooter.accelerator_motor, foc),
            gearing=1.0, moi=ACCELERATOR_MOI, friction_torque=0.03, viscous=0.0001)
        self.hood = Mechanism(
            SimTalon(shooter.hood_motor, plain),
            gearing=1.0, moi=HOOD_ROTOR_MOI, friction_torque=0.02, viscous=0.003,
            min_position=0.0, max_position=HOOD_MAX_ROTATIONS * TAU)
        self.indexer = Mechanism(
            SimTalon(hopper.indexer_motor, plain),
            gearing=1.0, moi=INDEXER_MOI, friction_torque=0.05, viscous=0.0002)
        self.mechanisms = [self.flywheel, self.accelerator, self.hood, self.indexer]

        # Inverse of the robot's tables: flywheel rps -> launch speed (m/s),
        # hood rotations -> launch angle (deg).
        velocity_table, angle_table = robot_lookup_tables
        self.speed_from_rps = sorted((rps, speed) for speed, rps in velocity_table.data)
        self.speed_from_rps.insert(0, (0.0, 0.0))
        self.angle_from_hood = sorted((hood, angle) for angle, hood in angle_table.data)
        self._feed_progress = 0.0
        self.shots_fired = 0

    def launch_speed(self):
        return max(0.0, _interpolate(self.speed_from_rps, abs(self.flywheel.rps)))

    def launch_angle_deg(self):
        hood = min(max(self.hood.rotations, self.angle_from_hood[0][0]), self.angle_from_hood[-1][0])
        return _interpolate(self.angle_from_hood, hood)

    def feed(self, hopper_count, dt):
        """How many balls the indexer pushes into the shooter this step."""
        indexer_rps = self.indexer.rps
        if (hopper_count <= 0 or indexer_rps < MIN_INDEXER_RPS
                or self.accelerator.rps < MIN_ACCELERATOR_RPS):
            self._feed_progress = 0.0
            return 0
        self._feed_progress += MAX_FEED_RATE * min(1.0, indexer_rps / INDEXER_FULL_RPS) * dt
        balls = min(int(self._feed_progress), hopper_count)
        self._feed_progress -= balls
        return balls

    def shoot(self, robot_pose, field_vx, field_vy, omega, ball_mass):
        """Launch state of one ball: field position, velocity and spin axis."""
        speed = self.launch_speed() * random.gauss(1.0, SPEED_NOISE_FRACTION)
        pitch = math.radians(self.launch_angle_deg() + random.gauss(0.0, PITCH_NOISE_DEG))
        heading = robot_pose.rotation().radians()
        yaw = heading + SHOOTER_YAW + math.radians(random.gauss(0.0, YAW_NOISE_DEG))
        sx, sy, sz = SHOOTER_POSITION
        c, s = math.cos(heading), math.sin(heading)
        ox, oy = sx * c - sy * s, sx * s + sy * c
        position = (robot_pose.X() + ox, robot_pose.Y() + oy, sz)
        horizontal = speed * math.cos(pitch)
        velocity = (horizontal * math.cos(yaw) + field_vx - omega * oy,
                    horizontal * math.sin(yaw) + field_vy + omega * ox,
                    speed * math.sin(pitch))
        # Backspin: axis horizontal, perpendicular to the shot, lifting the ball.
        spin_axis = (math.sin(yaw), -math.cos(yaw), 0.0)

        # The ball takes its kinetic energy (plus losses) out of the flywheel.
        energy = 0.5 * ball_mass * speed * speed * SHOT_ENERGY_LOSS_FACTOR
        w = self.flywheel.velocity
        self.flywheel.velocity = math.copysign(
            math.sqrt(max(0.0, w * w - 2 * energy / FLYWHEEL_MOI)), w)
        self.shots_fired += 1
        return position, velocity, spin_axis


class Intake:
    def __init__(self, intake):
        foc, plain = DCMotor.krakenX60FOC(1), DCMotor.krakenX60(1)
        self.rollers = Mechanism(
            SimTalon(intake.left_intake_motor, plain),
            gearing=1.0, moi=INTAKE_ROLLER_MOI, friction_torque=0.05, viscous=0.0002,
            followers=[(SimTalon(intake.right_intake_motor, plain), -1)])
        # The pivot closes its loop on the deploy CANcoder (REMOTE_CANCODER).
        self.cancoder = intake.deploy_cancoder
        self.magnet_offset = intake_magnet_offset(intake)
        low, high = INTAKE_PIVOT_RANGE
        self.pivot = Mechanism(
            SimTalon(intake.deploy_motor, foc),
            gearing=INTAKE_PIVOT_GEARING, moi=INTAKE_PIVOT_MOI, friction_torque=0.5, viscous=0.5,
            min_position=low * TAU, max_position=high * TAU, position=INTAKE_STOWED * TAU)
        self.mechanisms = [self.rollers, self.pivot]

    def write_cancoder(self):
        self.cancoder.sim_state.set_raw_position(self.pivot.rotations - self.magnet_offset)
        self.cancoder.sim_state.set_velocity(self.pivot.rps)

    def collecting(self):
        return self.pivot.rotations <= INTAKE_DEPLOYED_BELOW and self.rollers.rps > 10.0


def intake_magnet_offset(intake):
    """The deploy CANcoder's configured magnet offset (reported = raw + offset)."""
    from phoenix6 import configs
    config = configs.CANcoderConfiguration()
    intake.deploy_cancoder.configurator.refresh(config)
    return config.magnet_sensor.magnet_offset
