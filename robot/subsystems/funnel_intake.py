from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot


import time
import math
from phoenix6.hardware import TalonFX
from wpilib import SmartDashboard, Timer
from phoenix6 import configs, hardware, controls, signals
from commands2 import Subsystem

import const

from collections import deque

from phoenix6.hardware import CANrange
from phoenix6.configs import CANcoderConfigurator
from phoenix6.configs.config_groups import ProximityParamsConfigs


class FunnelIntake(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.intake_motor = hardware.TalonFX(const.FUNNEL_INTAKE_MOTOR_CAN_ID, "rio")

        funnel_intake_config = configs.TalonFXConfiguration()  # apply config file
        funnel_intake_config.motor_output.inverted = signals.InvertedValue(1)
        funnel_intake_config.current_limits.supply_current_limit = 40
        funnel_intake_config.current_limits.supply_current_limit_enable = True
        funnel_intake_config.slot0.k_p = 2.5
        funnel_intake_config.slot0.k_i = 0.0
        funnel_intake_config.slot0.k_d = 0.0
        funnel_intake_config.slot0.k_v = 0.0

        funnel_intake_config.closed_loop_ramps.torque_closed_loop_ramp_period = 0.02
        funnel_intake_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
        funnel_intake_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = 0.02
        funnel_intake_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 0.02
        funnel_intake_config.closed_loop_ramps.voltage_closed_loop_ramp_period = 0.02
        funnel_intake_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02

        self.intake_motor.configurator.apply(funnel_intake_config)  # type: ignore

        self.commanded_speed = 0.0
        self.is_intaking = False
        self.piece_passing_through = False

        self.funnel_cannrange = CANrange(const.FUNNEL_CANRANGE, "rio")
        self.funnel_cannrange_config = configs.CANrangeConfiguration()
        self.funnel_cannrange_prox_config = ProximityParamsConfigs()
        self.funnel_cannrange_prox_config.proximity_threshold = 0.09
        self.funnel_cannrange_config.with_proximity_params(self.funnel_cannrange_prox_config)

        self.funnel_cannrange.configurator.apply(self.funnel_cannrange_config)

        funnel_piece_length = 1
        self.piece_detected_in_funnel = deque(maxlen=funnel_piece_length)
        for i in range(funnel_piece_length):
            self.piece_detected_in_funnel.append(False)

        self.outtake_speed = -30
        SmartDashboard.putNumber("Funnel Outtake", self.outtake_speed)

    def stop(self):
        self.intake_motor.set_control(controls.VelocityTorqueCurrentFOC(0.0))

    def intake(self, speed=150):
        self.commanded_speed = speed
        if abs(self.intake_motor.get_velocity().value - self.commanded_speed) <= 0.25:
            return
        self.intake_motor.set_control(controls.VelocityTorqueCurrentFOC(speed))

    def periodic(self):
        if self.robot.in_autonomous_mode or not self.robot.in_autonomous_mode:
            if self.robot.isSimulation():
                self.piece_detected_in_funnel.append(False)
            else:
                self.piece_detected_in_funnel.append(self.funnel_cannrange.get_is_detected().value)
            self.piece_passing_through = all(self.piece_detected_in_funnel)
            
        if self.is_intaking:
            self.intake(55)
        elif self.robot.score_piece:
            self.intake(SmartDashboard.getNumber("Funnel Outtake", -40))
        elif self.robot.mechanisms_at_default:
            self.piece_passing_through = False
            self.stop()

    def log(self):
        SmartDashboard.putBoolean("funnel is intaking", self.is_intaking)
        SmartDashboard.putBoolean("piece passing through funnel", self.piece_passing_through)
        SmartDashboard.putNumber("funnel intake speed", self.intake_motor.get_velocity().value)
        SmartDashboard.putNumber("funnel commanded intake speed", self.commanded_speed)
        SmartDashboard.putNumber("funnel canrange dist", self.funnel_cannrange.get_distance().value)
        SmartDashboard.putBoolean("funnel detected", self.funnel_cannrange.get_is_detected().value)
