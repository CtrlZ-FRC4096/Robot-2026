"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2022
Code for robot "swerve strain prototype"
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

"""
Prepend these to any port IDs.
DIO = Digital I/O
AIN = Analog Input
PWM = Pulse Width Modulation
CAN = Controller Area Network
PCM = Pneumatic Control Module
PDP = Power Distribution Panel
"""

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

class MotorWrapper(hardware.TalonFX):
    def __init__(self,  device_id : int,
                        canbus : str,
                        inverted : int = 0,
                        k_p : float = 0.0,
                        k_i : float = 0.0,
                        k_d : float = 0.0,
                        k_v : float = 0.0,
                        k_a : float = 0.0, 
                        k_g : float = 0.0,
                        k_s : float = 0.0,
                        ):
        super().__init__(device_id, canbus)
        motor_config = configs.TalonFXConfiguration()
        motor_config.motor_output.inverted = signals.InvertedValue(inverted)
        motor_config.current_limits.stator_current_limit = 100
        motor_config.current_limits.supply_current_limit_enable = True

        motor_config.slot0.k_p = k_p
        motor_config.slot0.k_i = k_i
        motor_config.slot0.k_d = k_d
        motor_config.slot0.k_v = k_v
        motor_config.slot0.k_a = k_a
        motor_config.slot0.k_g = k_g
        motor_config.slot0.k_s = k_s

        motor_config.closed_loop_ramps.torque_closed_loop_ramp_period = 0.02
        motor_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
        motor_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = 0.02
        motor_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 0.02
        motor_config.closed_loop_ramps.voltage_closed_loop_ramp_period = 0.02
        motor_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02

        motor_config.current_limits.supply_current_limit = 80
        motor_config.torque_current.peak_forward_torque_current = 80
        motor_config.torque_current.peak_reverse_torque_current = -80

        self.configurator.apply(motor_config)


        