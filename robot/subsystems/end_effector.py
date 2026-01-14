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
from phoenix6.configs.config_groups import ProximityParamsConfigs, ToFParamsConfigs, FovParamsConfigs
from phoenix6.signals import UpdateModeValue
from robot_scoring_positions import RobotScoringPositions
import wpilib

class EndEffector(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.end_effector_motor = hardware.TalonFX(const.END_EFFECTOR_MOTOR_CAN_ID, "rio")

        self.end_effector_config = configs.TalonFXConfiguration()  # apply config file
        self.end_effector_config.motor_output.inverted = signals.InvertedValue(0)
        self.end_effector_config.current_limits.supply_current_limit = 40
        self.end_effector_config.current_limits.supply_current_limit_enable = True

		## First we will set everything to zero and then adjust k_g until the elevator is able to hold its position when we command a position
        self.end_effector_config.slot0.k_g = 0.45

        ## Next we will adjust k_s until the elevator just barely moves when we command a position (check both up and down)
        self.end_effector_config.slot0.k_s = 0.5

        ## Next we will adjust k_p until the elevator moves to the correct position and slightly overshoots/oscillates
        self.end_effector_config.slot0.k_p = 8.0
        ## Next we will adjust k_d until the elevator moves to the correct position without overshooting/oscillating
        self.end_effector_config.slot0.k_d = 0.0

        ## Adjust other stuff if we need it like k_v and k_a for feed forward
        self.end_effector_config.current_limits.supply_current_limit = 80
        self.end_effector_config.torque_current.peak_forward_torque_current = 80
        self.end_effector_config.torque_current.peak_reverse_torque_current = -80
        ##Ramps
        self.end_effector_config.closed_loop_ramps.torque_closed_loop_ramp_period = 0.02
        self.end_effector_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
        self.end_effector_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = 0.02
        self.end_effector_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 0.02
        self.end_effector_config.closed_loop_ramps.voltage_closed_loop_ramp_period = 0.02
        self.end_effector_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02
        self.end_effector_config.current_limits.supply_current_limit_enable = True
        self.end_effector_config.motor_output.neutral_mode = signals.NeutralModeValue(1)
        self.end_effector_config.current_limits.stator_current_limit = 100

        self.end_effector_config.motion_magic.motion_magic_cruise_velocity = 100 # Recalc has us at 16 RPS, but starting slow
        self.end_effector_config.motion_magic.motion_magic_acceleration = 150 # Recalc has us at 100 RPS/s^2, but starting slow

        self.end_effector_motor.configurator.apply(self.end_effector_config)
        self.isRunning = False

        self.canrange_end_effector = CANrange(const.END_EFFECTOR_CANRANGE_ID, "rio")

        self.canrange_end_effector_config = configs.CANrangeConfiguration()
        self.canrange_end_effector_prox_config = ProximityParamsConfigs()
        # CHANGE PROXIMITY STUFF
        self.canrange_end_effector_prox_config.proximity_threshold = 0.08
        self.canrange_end_effector_config.with_proximity_params(self.canrange_end_effector_prox_config)

        self.canrange_end_effector.configurator.apply(self.canrange_end_effector_config)

        self.sprocket_diameter = 1.790 # in.
        self.gear_ratio = 4.0 # need to adjust if using something else
        self.command_position = 0.0

        ## self.request = controls.DynamicMotionMagicTorqueCurrentFOC(0.0) We can use dynamic motion magic to change the cruise velocity and acceleration on the fly, less acceration when the elevator comes down, etc.
        self.request = controls.MotionMagicVoltage(0, enable_foc=True)

        self.outtake_motor = hardware.TalonFX(const.END_EFFECTOR_OUTTAKE_MOTOR_CAN_ID, "rio")
        self.outtake_motor_config = configs.TalonFXConfiguration()  # apply config file
        self.outtake_motor_config.motor_output.inverted = signals.InvertedValue(0)
        self.outtake_motor_config.current_limits.supply_current_limit = 40
        self.outtake_motor_config.current_limits.supply_current_limit_enable = True
        self.outtake_motor_config.slot0.k_p = 2.0
        self.outtake_motor_config.slot0.k_i = 0.0
        self.outtake_motor_config.slot0.k_d = 0.0
        self.outtake_motor_config.slot0.k_s = 3.5
        self.outtake_motor_config.slot0.k_v = 0.24

        self.outtake_motor_config.closed_loop_ramps.torque_closed_loop_ramp_period = 0.02
        self.outtake_motor_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
        self.outtake_motor_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = 0.02
        self.outtake_motor_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 0.02
        self.outtake_motor_config.closed_loop_ramps.voltage_closed_loop_ramp_period = 0.02
        self.outtake_motor_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02
        self.outtake_motor.configurator.apply(self.outtake_motor_config)


        self.end_effector_reef_alignment_can_range = CANrange(const.END_EFFECTOR_REEF_ALIGNMENT_CANRANGE, "rio")
        self.end_effector_reef_alignment_can_range_config = configs.CANrangeConfiguration()
        self.end_effector_reef_alignment_can_range_t_of_config = ToFParamsConfigs()
        self.end_effector_reef_alignment_can_range_t_of_config.update_frequency = 20 # hz
        self.end_effector_reef_alignment_can_range_t_of_config.update_mode = UpdateModeValue.SHORT_RANGE_USER_FREQ
        self.end_effector_reef_alignment_can_range_config.with_to_f_params(self.end_effector_reef_alignment_can_range_t_of_config)

        self.end_effector_reef_alignment_can_range_fov_config = FovParamsConfigs()
        self.end_effector_reef_alignment_can_range_fov_config.fov_center_x = -2.0
        self.end_effector_reef_alignment_can_range_fov_config.fov_range_x = 6.75
        self.end_effector_reef_alignment_can_range_config.with_fov_params(self.end_effector_reef_alignment_can_range_fov_config)

        self.end_effector_reef_alignment_can_range.configurator.apply(self.end_effector_reef_alignment_can_range_config)

        self.set_end_effector_position(RobotScoringPositions.end_effector_travel_position)

        self.is_intaking = False

        self.commanded_outtake_motor_speed = 0.0

        self.piece_passing_through_previous_tick = False

        self.max_extension = 10.5

        deque_length = 1
        self.piece_detected = deque(maxlen=deque_length)
        for i in range(deque_length):
            self.piece_detected.append(False)

        reef_deque_length = 3
        self.reef_detected = deque(maxlen=reef_deque_length)
        for i in range(reef_deque_length):
            self.reef_detected.append(False)


        self.end_effector_extension_encoder = wpilib.DutyCycleEncoder(1)

        # value_at_0 = 10.0
        # self.end_effector_motor.set_position((self.end_effector_extension_encoder - value_at_0) * self.gear_ratio)
        self.end_effector_motor.set_position(0.0)

    def stop(self):
        # self.end_effector_motor.set_control(controls.PositionVoltage(0.0, enable_foc=True))
        self.outtake_motor.set_control(controls.VelocityTorqueCurrentFOC(0.0))
        self.outtake_motor.set_control(controls.StaticBrake())

    def set_end_effector_position(self, position):
        if (abs(self.get_position() - position) <= 0.02):
            return
        self.command_position = position
        sprocket_rotations = position / (math.pi * self.sprocket_diameter)
        rotation = sprocket_rotations * self.gear_ratio
        self.end_effector_motor.set_control(self.request.with_position(rotation))

    def get_position(self):
        if self.robot.isSimulation():
            return self.command_position
        else:
            rotations = self.end_effector_motor.get_position().value
            height = rotations / self.gear_ratio * math.pi * self.sprocket_diameter
            return height

    def set_outtake_motor_speed(self, speed):
        self.commanded_outtake_motor_speed = speed
        # if abs(self.outtake_motor.get_velocity().value - self.commanded_outtake_motor_speed) <= 0.25:
        #     return
        self.outtake_motor.set_control(controls.VelocityTorqueCurrentFOC(speed))

    def has_coral(self):
        return self.canrange_end_effector.get_is_detected().value

    def lined_up_with_reef(self):
        # once we get function working use this piece in EE periodic: (self.lined_up_with_reef() and (self.robot.manual_scoring or self.robot.score_intent)) or
        if self.robot.end_effector_canrange_for_reef_returning_bad_values:
            return False
        return (self.robot.score_state.number == 4) and (all(self.reef_detected)) and (len(self.reef_detected) > 0)

    def periodic(self):
        if self.robot.score_state.number == 4 and (self.robot.score_intent or self.robot.manual_scoring) and (abs(self.robot.score_state.elevator_height-self.robot.elevator.get_height()) <= 0.2) and (abs(self.robot.end_effector.get_position() - self.robot.score_state.end_effector_position) <= 0.2):
            self.reef_detected.appendleft(0.2 <= self.end_effector_reef_alignment_can_range.get_distance().value <= 0.42)

        if (self.robot.score_piece) or ((self.robot.at_scoring_position) and (abs(self.robot.score_state.elevator_height-self.robot.elevator.get_height()) <= 0.2) and (abs(self.robot.end_effector.get_position() - self.robot.score_state.end_effector_position) <= 0.2) and ((self.robot.isSimulation() and self.robot.has_coral) or not self.robot.isSimulation())): # manual vs automated
            self.set_outtake_motor_speed(self.robot.score_state.end_effector_outtake_speed)
            self.reef_detected.clear()
            self.robot.drivetrain.reset_pid_error()
            if self.robot.score_state.number == 1:
                self.robot.raise_elevator_slightly_for_L1 = True
                self.robot.strafe_for_L1 = True
            self.robot.has_coral = False
            if self.robot.score_state.number == 2:
                self.robot.score_state= RobotScoringPositions.L2_Scoring
            elif self.robot.score_state.number == 3:
                self.robot.score_state = RobotScoringPositions.L3_Scoring
            #face, level, right_branch
            closest_face = self.robot.poseEstimator.calculate_closest_reef_tag()[1]
            if [closest_face, self.robot.score_state.number, self.robot.right_branch] not in self.robot.sim_coral_scored:
                self.robot.sim_coral_scored.append([closest_face, self.robot.score_state.number, self.robot.right_branch])
        elif self.is_intaking:
            if self.robot.elevator.get_height() <= RobotScoringPositions.min_elevator_height_to_bring_in_end_effector:
                self.set_end_effector_position(RobotScoringPositions.end_effector_intake_position)
                self.set_outtake_motor_speed(15.0) # default outtake speed
                self.piece_detected.appendleft(self.canrange_end_effector.get_is_detected().value)
                if (all(self.piece_detected) and not self.robot.isSimulation()) or (self.robot.isSimulation() and self.robot.at_intake_position):
                    self.robot.mechanisms_at_default = True
                    self.is_intaking = False
                    self.robot.funnel_intake.is_intaking = False
                    self.robot.funnel_intake.stop()
                    self.stop()
                    self.set_end_effector_position(RobotScoringPositions.end_effector_travel_position)
                    self.robot.has_coral = True
                    self.robot.drivetrain.reset_pid_error()
            else:
                self.robot.score_intent = False
                self.robot.manual_scoring = False
                self.robot.raise_elevator_slightly_for_L1 = False
                self.robot.strafe_for_L1 = False
                self.robot.is_climbing = False
                self.robot.at_scoring_position = False
                self.robot.elevator.set_elevator_height(RobotScoringPositions.elevator_intake_height)
        elif self.robot.is_climbing:
            if self.robot.elevator.get_height() <= RobotScoringPositions.min_elevator_height_to_bring_in_end_effector:
                self.set_end_effector_position(RobotScoringPositions.end_effector_climbing_position)
            else:
                self.is_intaking = False
                self.robot.raise_elevator_slightly_for_L1 = False
                self.robot.strafe_for_L1 = False
                self.robot.manual_scoring = False
                self.robot.running_pid_lineup = False
                self.robot.drivetrain.at_inter_pose = False
                self.robot.score_intent = False
                self.robot.at_scoring_position = False
                self.robot.elevator.set_elevator_height(RobotScoringPositions.elevator_climb_height)
        elif self.robot.mechanisms_at_default:
            self.stop()
            if not self.robot.is_climbing and (self.robot.has_coral and self.robot.elevator.get_height() <= RobotScoringPositions.min_elevator_height_to_bring_in_end_effector) or (not self.robot.has_coral and self.robot.elevator.get_height() >= RobotScoringPositions.min_elevator_height_to_bring_in_end_effector):
                self.set_end_effector_position(RobotScoringPositions.end_effector_travel_position)
            elif self.robot.elevator.get_height() <= RobotScoringPositions.min_elevator_height_to_bring_in_end_effector:
                self.set_end_effector_position(RobotScoringPositions.end_effector_intake_position)


    def log(self):
        SmartDashboard.putBoolean("lined up with reef", self.lined_up_with_reef())
        SmartDashboard.putBoolean("ignore EE canrange for reef alignment", self.robot.end_effector_canrange_for_reef_returning_bad_values)
        SmartDashboard.putNumber("end effector reef canrange distance", self.end_effector_reef_alignment_can_range.get_distance().value)
        SmartDashboard.putNumber("end effector position (in)", self.get_position())
        SmartDashboard.putNumber("end effector command position (in)", self.command_position)
        SmartDashboard.putBoolean("end effector is intaking", self.is_intaking)
        SmartDashboard.putNumber("end effector outtake speed", self.outtake_motor.get_velocity().value)
        SmartDashboard.putNumber("end effector commanded outtake speed", self.commanded_outtake_motor_speed)
        SmartDashboard.putBoolean("robot is at scoring position", self.robot.at_scoring_position)
        SmartDashboard.putBoolean("end effector canrange detecting piece", self.canrange_end_effector.get_is_detected().value)
        SmartDashboard.putNumber("end effector canrange distance", self.canrange_end_effector.get_distance().value)

