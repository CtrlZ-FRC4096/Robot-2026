"""Four-direction voltage characterization of the 2026 swerve modules.

Positions and velocities are raw Talon rotor rotations and rotations/second.
These voltage gains therefore describe Phoenix's voltage control. Normal
driving now uses VelocityVoltage so its kS/kV gains can be applied directly.
"""

from commands2 import FunctionalCommand
from commands2.sysid import SysIdRoutine
from phoenix6 import controls
from wpilib.sysid import SysIdRoutineLog

import const


class SwerveSysId:
    def __init__(self, drivetrain, modules):
        self.modules = modules
        self.drivetrain = drivetrain
        self.angle = SysIdRoutine(
            SysIdRoutine.Config(
                const.SYSID_ANGLE_RAMP_RATE,
                const.SYSID_ANGLE_STEP_VOLTAGE,
                const.SYSID_ANGLE_TIMEOUT,
            ),
            SysIdRoutine.Mechanism(
                self._drive_angle, self._log_angle, drivetrain, "swerve-angle"
            ),
        )
        self.drive = SysIdRoutine(
            SysIdRoutine.Config(
                const.SYSID_DRIVE_RAMP_RATE,
                const.SYSID_DRIVE_STEP_VOLTAGE,
                const.SYSID_DRIVE_TIMEOUT,
            ),
            SysIdRoutine.Mechanism(
                self._drive_drive, self._log_drive, drivetrain, "swerve-drive"
            ),
        )

    def _drive_angle(self, volts):
        for module in self.modules:
            module.drive_motor.set_control(controls.VoltageOut(0))
            module.angle_motor.set_control(controls.VoltageOut(volts))

    def _drive_drive(self, volts):
        for module in self.modules:
            module.angle_motor.set_control(controls.PositionVoltage(0))
            module.drive_motor.set_control(controls.VoltageOut(volts))

    def _log_angle(self, log: SysIdRoutineLog):
        for module in self.modules:
            motor = module.angle_motor
            (log.motor(f"angle-{module.module_name}")
                .voltage(motor.get_motor_voltage().value)
                .position(motor.get_position().value)
                .velocity(motor.get_velocity().value))

    def _log_drive(self, log: SysIdRoutineLog):
        for module in self.modules:
            motor = module.drive_motor
            (log.motor(f"drive-{module.module_name}")
                .voltage(motor.get_motor_voltage().value)
                .position(motor.get_position().value)
                .velocity(motor.get_velocity().value))

    def hold_straight(self):
        return FunctionalCommand(
            lambda: None,
            lambda: self._drive_drive(0),
            lambda interrupted: self.stop(),
            lambda: False,
            self.drivetrain,
        )

    def stop(self):
        for module in self.modules:
            module.angle_motor.set_control(controls.VoltageOut(0))
            module.drive_motor.set_control(controls.VoltageOut(0))
