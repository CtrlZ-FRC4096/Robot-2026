from timeit import Timer
import typing

from phoenix6 import controls, configs, hardware, signals

from phoenix6.hardware import TalonFX, Pigeon2, CANcoder
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

import const
from swerve import conversions, ctre_module_state
import math


class SwerveModule:
    module_name: str
    angle_offset: Rotation2d
    last_angle: Rotation2d

    angle_motor: TalonFX
    drive_motor: TalonFX
    angle_encoder: CANcoder

    def __init__(
        self,
        module_name: str,
        angle_offset: Rotation2d,
        drive_motor_id: int,
        angle_motor_id: int,
        cancoder_id: int,
        drive_invert: signals.InvertedValue,
        angle_invert: signals.InvertedValue,
    ):
        self.module_name = module_name
        self.angle_offset = angle_offset
        self.angle_encoder = hardware.CANcoder(cancoder_id, "carnivore")
        self.angle_motor = hardware.TalonFX(angle_motor_id, "carnivore")
        self.drive_motor = hardware.TalonFX(drive_motor_id, "carnivore")

        self.drive_invert = drive_invert
        self.angle_invert = angle_invert

        #        self.angle_encoder.configFactoryDefault()
        #        talonfx_configurator = self.talonfx.configurator
        swerve_can_coder_config = configs.CANcoderConfiguration()
        #        self.talonfx.configurator.apply(configs.TalonFXConfiguration())
        swerve_can_coder_config.magnet_sensor.sensor_direction = (
            const.SWERVE_INVERT_CANCODERS
        )

        self.angle_encoder.configurator.apply(
            swerve_can_coder_config  # type: ignore
        )  # Apply settings to angle encoder

        #        self.angle_motor.configFactoryDefault()
        #        swerve_angle_motor_configurator = self.talonfx.configurator
        swerve_angle_motor_config = configs.TalonFXConfiguration()
        #        self.talonfx.configurator.apply(configs.TalonFXConfiguration())
        #        swerve_angle_motor_config = hardware.TalonFXConfiguration()
        swerve_angle_motor_config.slot0.k_p = 2.4
        swerve_angle_motor_config.slot0.k_d = 0.1
        swerve_angle_motor_config.slot0.k_i = 0.0
        swerve_angle_motor_config.current_limits.supply_current_limit = (
            25  # I am not sure if this is correct
        )
        swerve_angle_motor_config.current_limits.supply_current_threshold = (
            40  # change back to 40
        )

        swerve_angle_motor_config.torque_current.peak_forward_torque_current = (
            25  # Up this to 40 for more zip
        )
        swerve_angle_motor_config.torque_current.peak_reverse_torque_current = -25
        ##Ramps
        swerve_angle_motor_config.closed_loop_ramps.torque_closed_loop_ramp_period = (
            0.02
        )
        swerve_angle_motor_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
        swerve_angle_motor_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = (
            0.02
        )
        swerve_angle_motor_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = (
            0.02
        )
        swerve_angle_motor_config.closed_loop_ramps.voltage_closed_loop_ramp_period = (
            0.02
        )
        swerve_angle_motor_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02

        swerve_angle_motor_config.current_limits.supply_time_threshold = 0.1
        swerve_angle_motor_config.current_limits.supply_current_limit_enable = True
        swerve_angle_motor_config.motor_output.inverted = (
            self.angle_invert
        )  # signals.InvertedValue(1)  # This is no longer a boolean; 0 for CCW 1 for CW
        swerve_angle_motor_config.motor_output.neutral_mode = signals.NeutralModeValue(
            1
        )  # set to brake
        swerve_angle_motor_config.current_limits.stator_current_limit = 100

        ## Add fused cancoder
        # swerve_angle_motor_config.feedback.feedback_remote_sensor_id = self.angle_encoder.device_id
        # swerve_angle_motor_config.feedback.feedback_sensor_source = signals.FeedbackSensorSourceValue.FUSED_CANCODER
        # swerve_angle_motor_config.feedback.sensor_to_mechanism_ratio = 1.0
        # swerve_angle_motor_config.feedback.rotor_to_sensor_ration = const.SWERVE_ANGLE_GEAR_RATIO

        self.angle_motor.configurator.apply(
            swerve_angle_motor_config  # type: ignore
        )  # Apply settings to angle motor
        # self.angle_motor.configAllSettings(swerve_angle_motor_config)
        # self.angle_motor.setInverted(const.SWERVE_ANGLE_MOTOR_INVERTED)
        # self.angle_motor.setNeutralMode(NeutralMode.Brake)

        #        self.angle_motor.configFactoryDefault()
        #        swerve_angle_motor_configurator = self.talonfx.configurator

        self.reset_to_absolute()

        swerve_drive_motor_config = configs.TalonFXConfiguration()
        # self.drive_motor.configurator.apply(swerve_drive_motor_config)  # type: ignore
        swerve_drive_motor_config.slot0.k_p = 2.2  # 2.2
        swerve_drive_motor_config.slot0.k_s = 5.6
        swerve_drive_motor_config.slot0.k_v = 0.24  # 0.24
        ## Feed Forward
        # swerve_drive_motor_config.slot0.k_v = const.SWERVE_DRIVE_KV
        # swerve_drive_motor_config.slot0.k_a = const.SWERVE_DRIVE_KA
        swerve_drive_motor_config.current_limits.supply_current_limit = (
            80  # I am not sure if this is correct
        )
        swerve_drive_motor_config.current_limits.supply_current_limit = (
            80  # change back to 40
        )

        swerve_drive_motor_config.torque_current.peak_forward_torque_current = (
            80  # Up this to 80 for more zip
        )
        swerve_drive_motor_config.torque_current.peak_reverse_torque_current = -80
        ##Ramps
        swerve_drive_motor_config.closed_loop_ramps.torque_closed_loop_ramp_period = (
            0.02
        )
        swerve_drive_motor_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
        swerve_drive_motor_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = (
            0.02
        )
        swerve_drive_motor_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = (
            0.02
        )
        swerve_drive_motor_config.closed_loop_ramps.voltage_closed_loop_ramp_period = (
            0.02
        )
        swerve_drive_motor_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02

        swerve_drive_motor_config.current_limits.supply_current_limit_enable = True
        swerve_drive_motor_config.motor_output.inverted = (
            self.drive_invert
        )  # signals.InvertedValue(1)  # This is no longer a boolean; 0 for CCW 1 for CW
        swerve_drive_motor_config.motor_output.neutral_mode = signals.NeutralModeValue(
            1
        )  # set to brake
        swerve_drive_motor_config.current_limits.stator_current_limit = 100

        self.drive_motor.configurator.apply(swerve_drive_motor_config)  # type: ignore
        self.feedforward = SimpleMotorFeedforwardMeters(
            const.SWERVE_DRIVE_KS, const.SWERVE_DRIVE_KV, const.SWERVE_DRIVE_KA
        )

    def set_desired_state(self, desired_state: SwerveModuleState, is_open_loop):
        desired_state = ctre_module_state.optimize(
            desired_state, self.get_state().angle
        )
        self.set_angle(desired_state)

        ## Add cosine compensation, wheels don't spin as fast when they are at the wrong angle
        desired_state.speed *= (desired_state.angle - self.get_state().angle).cos()

        self.set_speed(desired_state, is_open_loop)

    def set_speed(self, desired_state: SwerveModuleState, is_open_loop):
        if is_open_loop:
            percent_output = desired_state.speed / const.SWERVE_MAX_SPEED
            self.drive_motor.set_control(controls.DutyCycleOut(percent_output))
        else:
            wheel_RPS = desired_state.speed / const.SWERVE_WHEEL_CIRCUMFERENCE
            motor_RPS = wheel_RPS * const.SWERVE_DRIVE_GEAR_RATIO

            # print(motor_RPS)
            if (
                abs(desired_state.speed) <= 0.001
            ):  # if value is small, dont change the speed
                motor_RPS = 0.0

            self.drive_motor.set_control(
                controls.VelocityTorqueCurrentFOC(
                    motor_RPS,
                    acceleration=300,
                    # feed_forward=self.feedforward.calculate(desired_state.speed), #Remove Feedfoward for now
                )
            )

    def set_angle(self, desired_state: SwerveModuleState):
        if abs(desired_state.speed) <= 0.05:  # if value is small, dont change the angle
            angle = self.get_angle()
        else:
            angle = desired_state.angle

        self.angle_motor.set_control(
            controls.PositionVoltage(
                angle.degrees() / 360 * const.SWERVE_ANGLE_GEAR_RATIO, enable_foc=True
            )
            # conversions.degrees_to_falcon(
            #     angle.degrees(), const.SWERVE_ANGLE_GEAR_RATIO
            # ),
        )
        self.last_angle = angle

    def get_angle(self):
        # return self.get_angle_CANcoder()
        return Rotation2d.fromDegrees(
            self.angle_motor.get_position().value * 360 / const.SWERVE_ANGLE_GEAR_RATIO,
        )

    def get_angle_CANcoder(self):
        # print(self.angle_encoder.get_absolute_position().T)
        encoder_value_signal = self.angle_encoder.get_position()
        # print(encoder_value_signal.value)
        # print(encoder_value_signal.getValue())
        return Rotation2d.fromDegrees(
            float(encoder_value_signal.value) * 360  # * const.SWERVE_ANGLE_GEAR_RATIO,
        )  # this will run but its bad, need to convert cancoder value to degrees

    def get_state(self):
        return SwerveModuleState(
            self.drive_motor.get_velocity().value
            / const.SWERVE_DRIVE_GEAR_RATIO
            * const.SWERVE_WHEEL_CIRCUMFERENCE,
            self.get_angle(),
        )

    def get_position(self):
        return SwerveModulePosition(
            self.drive_motor.get_position().value
            / const.SWERVE_DRIVE_GEAR_RATIO
            * const.SWERVE_WHEEL_CIRCUMFERENCE,
            self.get_angle(),
        )

    def reset_to_absolute(self):
        cancoder_angle: float = typing.cast(float, self.get_angle_CANcoder().degrees())
        angle_offset: float = typing.cast(float, self.angle_offset.degrees())
        absolute_position = (
            (cancoder_angle - angle_offset) / 360 * const.SWERVE_ANGLE_GEAR_RATIO
        )
        self.angle_motor.set_position(absolute_position)

    # def reset_to_absolute(self):
    # cancoder_angle: float = typing.cast(float, self.get_angle_CANcoder().degrees())
    # angle_offset: float = typing.cast(float, self.angle_offset.degrees())
    # absolute_position = (
    # cancoder_angle - angle_offset
    # ) / 360  # * const.SWERVE_ANGLE_GEAR_RATIO
    # self.angle_motor.set_control(
    # controls.PositionVoltage(absolute_position * const.SWERVE_ANGLE_GEAR_RATIO),
    # conversions.degrees_to_falcon(
    #     angle.degrees(), const.SWERVE_ANGLE_GEAR_RATIO
    # ),
    # )  # setSelectedSensorPosition() -> set_position()
