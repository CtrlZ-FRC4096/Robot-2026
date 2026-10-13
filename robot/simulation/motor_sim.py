"""DC motor models behind simulated TalonFXs.

Phoenix 6 runs each Talon's control loop (PID, Motion Magic, current
control) in simulation and reports the voltage it applies. SimTalon turns
that voltage into current and torque with WPILib's DCMotor model, the way
maple-sim's SimMotorConfigs does, and writes the resulting rotor position
and velocity back so the robot code reads them as encoder values.
"""

import math

import wpilib
from phoenix6 import configs, signals
from phoenix6.signals import ControlModeValue
from phoenix6.sim import ChassisReference, TalonFXSimState
from wpimath.system.plant import DCMotor

TAU = 2 * math.pi

# Requests the Talon closes on stator current at ~1 kHz. Phoenix's voltage is
# only sampled once per robot loop here, so for these the current is held
# for the whole loop instead of the voltage (what the real current loop does).
TORQUE_CURRENT_MODES = {m for m in ControlModeValue if "TORQUE_CURRENT" in m.name}
# Requests whose zero output is "neutral" (brake or coast as configured).
NEUTRAL_AT_ZERO_MODES = {
    ControlModeValue.DISABLED_OUTPUT, ControlModeValue.NEUTRAL_OUT, ControlModeValue.COAST_OUT,
    ControlModeValue.DUTY_CYCLE_OUT, ControlModeValue.VOLTAGE_OUT,
}


class SimTalon:
    """One simulated TalonFX driving a mechanism.

    motor: WPILib DCMotor for one motor (Kraken X60, FOC or not). Inversion,
    neutral mode and the stator, supply and torque current limits are read
    from the Talon's own configuration, so they match what the robot applied.
    """

    def __init__(self, talon, motor: DCMotor):
        self.talon = talon
        self.motor = motor
        self.sim = talon.sim_state
        self.sim.set_motor_type(TalonFXSimState.MotorType.KRAKEN_X60)
        self._mode_signal = talon.get_control_mode()
        self.voltage = 0.0
        self.bus_voltage = 12.0
        self.mode = ControlModeValue.DISABLED_OUTPUT
        self.held_current = None
        self.last_current = 0.0
        self.last_voltage = 0.0
        # Raw rotor position that corresponds to mechanism position 0.
        self.rotor_offset = 0.0
        self.read_config()

    def read_config(self):
        """Read the configs the robot applied (slow in simulation: ~20 ms)."""
        config = configs.TalonFXConfiguration()
        self.talon.configurator.refresh(config)
        inverted = config.motor_output.inverted == signals.InvertedValue.CLOCKWISE_POSITIVE
        self.sim.orientation = (ChassisReference.CLOCKWISE_POSITIVE if inverted
                                else ChassisReference.COUNTER_CLOCKWISE_POSITIVE)
        self.brake = config.motor_output.neutral_mode == signals.NeutralModeValue.BRAKE
        limits = config.current_limits
        self.stator_limit = limits.stator_current_limit if limits.stator_current_limit_enable else math.inf
        self.supply_limit = limits.supply_current_limit if limits.supply_current_limit_enable else math.inf
        self.torque_limits = (config.torque_current.peak_reverse_torque_current,
                              config.torque_current.peak_forward_torque_current)

    def sample(self, rotor_speed, bus_voltage):
        """Read the Talon's output once per robot loop. rotor_speed in rad/s."""
        self.bus_voltage = bus_voltage
        self.sim.set_supply_voltage(bus_voltage)
        self.voltage = self.sim.motor_voltage
        self.mode = self._mode_signal.refresh().value
        if self.mode in TORQUE_CURRENT_MODES and wpilib.DriverStation.isEnabled():
            current = self.motor.current(rotor_speed, self.voltage)
            self.held_current = max(self.torque_limits[0], min(self.torque_limits[1], current))
        else:
            self.held_current = None

    def neutral(self):
        return (not wpilib.DriverStation.isEnabled()
                or (self.voltage == 0.0 and self.mode in NEUTRAL_AT_ZERO_MODES))

    def current(self, rotor_speed):
        """Stator current (A) at this rotor speed (rad/s) for the sampled output."""
        m = self.motor
        back_emf = rotor_speed / m.Kv
        if self.neutral():
            current = -back_emf / m.R if self.brake else 0.0
        elif self.held_current is not None:
            # The current loop can't push more than the bus voltage allows.
            hi = (self.bus_voltage - back_emf) / m.R
            lo = (-self.bus_voltage - back_emf) / m.R
            current = max(lo, min(hi, self.held_current))
        else:
            current = m.current(rotor_speed, self.voltage)
        current = max(-self.stator_limit, min(self.stator_limit, current))
        voltage = current * m.R + back_emf
        if current * voltage > self.supply_limit * self.bus_voltage:
            # Supply limit: the largest current whose I*V stays under it.
            sign = math.copysign(1.0, current)
            emf = sign * back_emf
            current = sign * (-emf + math.sqrt(emf * emf + 4 * m.R * self.supply_limit * self.bus_voltage)) / (2 * m.R)
            voltage = current * m.R + back_emf
        self.last_current = current
        # Terminal voltage that drives this current (zero when coasting).
        self.last_voltage = 0.0 if current == 0.0 else voltage
        return current

    def supply_current(self):
        """Battery current (A); negative while regenerating."""
        if self.bus_voltage <= 0:
            return 0.0
        return self.last_current * self.last_voltage / self.bus_voltage

    def write(self, rotor_rotations, rotor_rps):
        """Report a rotor position (rotations, from mechanism zero) and velocity."""
        self.sim.set_raw_rotor_position(rotor_rotations + self.rotor_offset)
        self.sim.set_rotor_velocity(rotor_rps)


class Mechanism:
    """A rotating mechanism driven by a leader Talon and optional followers.

    Positions and velocities are in mechanism radians, in the leader's
    direction. gearing = rotor turns per mechanism turn. All motors are assumed
    to make the same torque as the leader (followers mirror its output).
    followers: list of (SimTalon, +1 aligned / -1 opposed).
    """

    def __init__(self, leader: SimTalon, gearing, moi, followers=(), friction_torque=0.0,
                 viscous=0.0, min_position=None, max_position=None, position=0.0):
        self.leader = leader
        self.followers = list(followers)
        self.gearing = gearing
        self.moi = moi
        self.friction_torque = friction_torque
        self.viscous = viscous
        self.min_position = min_position
        self.max_position = max_position
        self.position = position
        self.velocity = 0.0
        self.motor_count = 1 + len(self.followers)
        leader.rotor_offset = -position * gearing / TAU

    def sample(self, bus_voltage):
        self.leader.sample(self.velocity * self.gearing, bus_voltage)
        for talon, _ in self.followers:
            talon.sim.set_supply_voltage(bus_voltage)

    def step(self, dt):
        rotor_speed = self.velocity * self.gearing
        current = self.leader.current(rotor_speed)
        torque = self.motor_count * self.leader.motor.torque(current) * self.gearing
        torque -= self.viscous * self.velocity
        # Coulomb friction, which can stop the mechanism but not reverse it.
        self.velocity += torque / self.moi * dt
        friction_dv = math.copysign(self.friction_torque, -self.velocity) / self.moi * dt
        if (self.velocity + friction_dv) * self.velocity <= 0:
            self.velocity = 0.0
        else:
            self.velocity += friction_dv
        self.position += self.velocity * dt
        if self.min_position is not None and self.position < self.min_position:
            self.position, self.velocity = self.min_position, max(0.0, self.velocity)
        if self.max_position is not None and self.position > self.max_position:
            self.position, self.velocity = self.max_position, min(0.0, self.velocity)

    def write(self):
        rotor_rot = self.position * self.gearing / TAU
        rotor_rps = self.velocity * self.gearing / TAU
        self.leader.write(rotor_rot, rotor_rps)
        for talon, sign in self.followers:
            talon.write(sign * rotor_rot, sign * rotor_rps)

    def supply_current(self):
        return self.motor_count * self.leader.supply_current()

    @property
    def rps(self):
        """Mechanism rotations per second."""
        return self.velocity / TAU

    @property
    def rotations(self):
        return self.position / TAU
