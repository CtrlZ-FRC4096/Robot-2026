"""Four-direction voltage characterization of the 2026 swerve modules.

Positions and velocities are raw Talon rotor rotations and rotations/second.
These voltage gains therefore describe Phoenix's voltage control. Normal
driving now uses VelocityVoltage so its kS/kV gains can be applied directly.
"""

import wpilib
from commands2 import FunctionalCommand
from commands2.sysid import SysIdRoutine
from phoenix6 import BaseStatusSignal, controls
from wpilib.sysid import SysIdRoutineLog

import const


class SwerveSysId:
    def __init__(self, drivetrain, modules):
        self.modules = modules
        self.drivetrain = drivetrain
        self._active_mechanism = None
        self._signals = {}
        for mechanism in ("angle", "drive"):
            self._signals[mechanism] = [
                (
                    module.module_name,
                    motor.get_motor_voltage(False),
                    motor.get_position(False),
                    motor.get_velocity(False),
                )
                for module in modules
                for motor in [module.angle_motor if mechanism == "angle"
                              else module.drive_motor]
            ]
        self._notifier = wpilib.Notifier(self._sample)
        self._notifier.setName("SwerveSysIdSampler")
        self.angle = SysIdRoutine(
            SysIdRoutine.Config(
                const.SYSID_ANGLE_RAMP_RATE,
                const.SYSID_ANGLE_STEP_VOLTAGE,
                const.SYSID_ANGLE_TIMEOUT,
            ),
            SysIdRoutine.Mechanism(
                self._drive_angle, lambda log: None, drivetrain, "swerve-angle"
            ),
        )
        self.drive = SysIdRoutine(
            SysIdRoutine.Config(
                const.SYSID_DRIVE_RAMP_RATE,
                const.SYSID_DRIVE_STEP_VOLTAGE,
                const.SYSID_DRIVE_TIMEOUT,
            ),
            SysIdRoutine.Mechanism(
                self._drive_drive, lambda log: None, drivetrain, "swerve-drive"
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

    def _sample(self):
        mechanism = self._active_mechanism
        if mechanism is None:
            return
        rows = self._signals[mechanism]
        BaseStatusSignal.refresh_all(
            *(signal for _, voltage, position, velocity in rows
              for signal in (voltage, position, velocity))
        )
        log: SysIdRoutineLog = self.angle if mechanism == "angle" else self.drive
        for name, voltage, position, velocity in rows:
            (log.motor(f"{mechanism}-{name}")
                .voltage(voltage.value)
                .position(position.value)
                .velocity(velocity.value))

    def command(self, mechanism, kind, direction):
        routine = self.angle if mechanism == "angle" else self.drive
        command = (routine.quasistatic(direction) if kind == "quasistatic"
                   else routine.dynamic(direction))
        return (command.beforeStarting(lambda: self._start_sampling(mechanism))
                .finallyDo(lambda interrupted: self._stop_sampling()))

    def _start_sampling(self, mechanism):
        rows = self._signals[mechanism]
        BaseStatusSignal.set_update_frequency_for_all(
            200, *(signal for _, voltage, position, velocity in rows
                   for signal in (voltage, position, velocity))
        )
        self._active_mechanism = mechanism
        self._notifier.startPeriodic(0.005)

    def _stop_sampling(self):
        self._notifier.stop()
        mechanism = self._active_mechanism
        self._active_mechanism = None
        if mechanism is not None:
            rows = self._signals[mechanism]
            BaseStatusSignal.set_update_frequency_for_all(
                100, *(signal for _, voltage, position, velocity in rows
                       for signal in (voltage, position, velocity))
            )

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
