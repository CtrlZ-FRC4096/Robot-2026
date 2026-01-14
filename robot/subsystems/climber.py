from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot


import time
import math
from phoenix6.hardware import TalonFX
from wpilib import SmartDashboard, Timer
from phoenix6 import configs, hardware, controls, signals
from commands2 import Subsystem
import wpilib
from collections import deque

from wpimath.geometry import (
    Translation2d,
    Pose2d,
    Rotation2d
)
from path_gen import PathGenerator
import const
from robot_scoring_positions import RobotScoringPositions

class Climber(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.climber_arm_motor = hardware.TalonFX(const.CLIMBER_ARM_MOTOR_CAN_ID, "rio")

        climber_arm_motor_config = configs.TalonFXConfiguration()

        ## First we will set everything to zero and then adjust k_g until the elevator is able to hold its position when we command a position
        climber_arm_motor_config.slot0.k_g = 0.0

        ## Next we will adjust k_s until the elevator just barely moves when we command a position (check both up and down)
        climber_arm_motor_config.slot0.k_s = 0.0

        ## Next we will adjust k_p until the elevator moves to the correct position and slightly overshoots/oscillates
        climber_arm_motor_config.slot0.k_p = 10.0
        ## Next we will adjust k_d until the elevator moves to the correct position without overshooting/oscillating
        climber_arm_motor_config.slot0.k_d = 0.0

        ## Adjust other stuff if we need it like k_v and k_a for feed forward
        climber_arm_motor_config.current_limits.supply_current_limit = 35
        climber_arm_motor_config.torque_current.peak_forward_torque_current = 35
        climber_arm_motor_config.torque_current.peak_reverse_torque_current = -35
        ##Ramps
        climber_arm_motor_config.closed_loop_ramps.torque_closed_loop_ramp_period = 0.02
        climber_arm_motor_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
        climber_arm_motor_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = 0.02
        climber_arm_motor_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 0.02
        climber_arm_motor_config.closed_loop_ramps.voltage_closed_loop_ramp_period = 0.02
        climber_arm_motor_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02
        climber_arm_motor_config.current_limits.supply_current_limit_enable = True
        climber_arm_motor_config.motor_output.neutral_mode = signals.NeutralModeValue(1)
        climber_arm_motor_config.current_limits.stator_current_limit = 100

        climber_arm_motor_config.motor_output.inverted = signals.InvertedValue(0)

        # We will adjust these values later to get the elevator moving faster
        climber_arm_motor_config.motion_magic.motion_magic_cruise_velocity = 100 # Recalc has us at 16 RPS, but starting slow
        climber_arm_motor_config.motion_magic.motion_magic_acceleration = 100 # Recalc has us at 100 RPS/s^2, but starting slow

        self.climber_arm_motor.configurator.apply(climber_arm_motor_config)

        self.gear_ratio = 315.0 # need to adjust if using something else
        self.command_position = 0.0
        self.request = controls.MotionMagicVoltage(0, enable_foc=True)

    def set_climber_position(self, position):
        self.command_position = position
        rotation = self.command_position * self.gear_ratio
        self.climber_arm_motor.set_control(self.request.with_position(rotation))

    def get_position(self):
        rotations = self.climber_arm_motor.get_position().value
        height = rotations / self.gear_ratio
        return height

    def climb(self):
        self.set_climber_position(0.0) # TODO: Find the correct position

    def stop(self):
        pass

    def periodic(self):
        if self.robot.is_climbing:
            # self.climb()
            if self.robot.retract_climber:
                self.set_climber_position(RobotScoringPositions.climber_climb_position)
            else:
                if (self.robot.end_effector.get_position() >= RobotScoringPositions.min_end_effector_position_to_climb) and (self.robot.elevator.get_height() >= RobotScoringPositions.min_elevator_climb_height):
                    self.set_climber_position(RobotScoringPositions.climber_up_position)
                else:
                    self.is_intaking = False
                    self.robot.manual_scoring = False
                    self.robot.running_pid_lineup = False
                    self.robot.score_intent = False
                    self.robot.at_scoring_position = False
                    self.robot.elevator.set_elevator_height(RobotScoringPositions.elevator_climb_height)
                    self.robot.end_effector.set_end_effector_position(RobotScoringPositions.end_effector_climbing_position)
        elif self.robot.mechanisms_at_default:
            if (self.robot.end_effector.get_position() >= RobotScoringPositions.min_end_effector_position_to_climb) and (self.robot.elevator.get_height() >= RobotScoringPositions.min_elevator_climb_height):
                self.set_climber_position(0.0)

    def log(self):
        SmartDashboard.putNumber("climber position", self.get_position())