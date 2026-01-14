"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2023
Code for robot ""
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

# This is to help vscode
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot  # type: ignore

from wpilib import SmartDashboard
from robot_scoring_positions import RobotScoringPositions
import wpilib
import wpilib.interfaces
import subsystems.leds
from commands2 import ParallelCommandGroup, Subsystem, WaitCommand

# from commands2.button import Button
from wpilibextra.customcontroller.custom_button import CustomButton as Button
from wpilib import DriverStation
from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
import math

from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.kinematics import ChassisSpeeds

import const


from pathplannerlib.auto import (
    AutoBuilder,
    PathPlannerAuto,
    NamedCommands,
    PathConstraints,
    PathPlannerPath,
)
from pathplannerlib.path import (
    GoalEndState,
    Waypoint,
    IdealStartingState,
)

from robotpy_apriltag import AprilTagField, AprilTagFieldLayout


# Controls
from wpilibextra.customcontroller import XboxCommandController

from field_const import FieldConstants
from path_gen import PathGenerator, PurePursuitController
from wpimath.units import inchesToMeters, degreesToRadians, radiansToDegrees
from phoenix5 import NeutralMode
from phoenix6.controls import CoastOut

###  IMPORTS ###


class OI:
    """
    Operator Input - This class ties together controls and commands.
    """

    def __init__(self, robot: "Robot"):
        self.robot = robot

        # Controllers
        self.driver1 = XboxCommandController(0)
        self.driver2 = XboxCommandController(1)

        # self.driver1.LEFT_JOY_Y.setInverted(True)

        self.driver1.LEFT_JOY_X.setDeadzone(0.02)
        self.driver1.LEFT_JOY_Y.setDeadzone(0.02)
        self.driver1.RIGHT_JOY_X.setDeadzone(0.1)
        self.driver1.RIGHT_JOY_Y.setDeadzone(0.1)

        self.driver2.LEFT_JOY_X.setDeadzone(0.02)
        self.driver2.LEFT_JOY_Y.setDeadzone(0.02)
        self.driver2.RIGHT_JOY_X.setDeadzone(0.1)
        self.driver2.RIGHT_JOY_Y.setDeadzone(0.1)

        # self.driver1.LEFT_JOY_X.setDeadzone(0.000)
        # self.driver1.LEFT_JOY_Y.setDeadzone(0.000)
        # self.driver1.RIGHT_JOY_X.setDeadzone(0.00)
        # self.driver1.RIGHT_JOY_Y.setDeadzone(0.00)

        ### Driving ###
        self.cardinal = 0
        self.cardinal_directing = False
        self.robot_oriented_angle = self.robot.poseEstimator.getYaw().degrees()

        self.rumble_button = Button(lambda: self.robot.has_coral)
        self.can_crash = False

        self.find_heading = True
        self.tick_count = 0
        self.tick_count_max = 5

        self.right_trigger_being_held = False
        self.left_trigger_being_held = False

        self.face = 1


        @self.rumble_button.whenPressed
        def _():
            timer = Timer()
            timer.start()
            self.driver2.setRumble(1)
            self.driver1.setRumble(1)
            while not timer.hasElapsed(0.5):
                yield
            self.driver2.setRumble(0)
            self.driver1.setRumble(0)

        @self.robot.drivetrain.setDefaultCommand
        def _():
            while True:
                yield
                def square(x):
                    return abs(x) * x

                forward_back = -square(self.driver1.LEFT_JOY_Y())
                left_right = -square(self.driver1.LEFT_JOY_X())

                if self.robot.fieldConstants.shouldFlip:
                    forward_back *= -1
                    left_right *= -1

                rotate = -self.driver1.RIGHT_JOY_X()
                # if self.robot.wheels_at_x:
                #     self.robot.drivetrain.turn_wheels_to_x()
                if self.robot.running_pid_lineup:
                    # Cancel drive with pid if robot is moving manually
                    if (
						abs(self.driver1.LEFT_JOY_X()) > 0.05
						or abs(self.driver1.LEFT_JOY_Y()) > 0.05
						or abs(self.driver1.RIGHT_JOY_X()) > 0.1
						or abs(self.driver1.RIGHT_JOY_Y()) > 0.1
					):
                        if self.robot.is_intaking:
                            forward_back *= 0.6
                            left_right *= 0.6
                            self.robot.drivetrain.go_to_pose_profiled_pid(self.robot.final_lineup_pose, forward_back, left_right, rotate)
                        else:
                            forward_back *= 0.4
                            left_right *= 0.4
                            self.robot.drivetrain.go_to_pose_angle_addition(self.robot.final_lineup_pose, forward_back, left_right, rotate)
                    else:
                        if self.robot.is_intaking:
                            self.robot.drivetrain.go_to_pose_profiled_pid(self.robot.final_lineup_pose)
                        else:
                            self.robot.drivetrain.go_to_pose_angle_addition(self.robot.final_lineup_pose)
                # elif (abs(const.SWERVE_KINEMATICS.toChassisSpeeds(self.robot.poseEstimator.get_module_states()).vx) <= 0.005 and
                #       abs(const.SWERVE_KINEMATICS.toChassisSpeeds(self.robot.poseEstimator.get_module_states()).vy) <= 0.005 and 
                #       abs(const.SWERVE_KINEMATICS.toChassisSpeeds(self.robot.poseEstimator.get_module_states()).omega_dps) <= 1):
                #     self.robot.drivetrain.turn_wheels_to_x()
                else:
                    if abs(rotate) >= 0.02:
                        self.cardinal_directing = False
                        self.find_heading = True
                        self.wait_one_tick = False
                        self.tick_count = 0
                        self.robot.drivetrain.drive(
                            Translation2d(forward_back, left_right)
                            * const.SWERVE_MAX_SPEED,
                            rotate * 4.25,
                            True,
                            False,
                        )
                        self.robot_oriented_angle = (
                            self.robot.poseEstimator.getYaw().degrees()
                        )
                        self.tick_count_max = 5
                    else:
                        # if not self.cardinal_directing:
                        #     if self.find_heading:
                        #         if self.wait_one_tick:
                        #             self.robot_oriented_angle = (
                        #                 self.robot.poseEstimator.getYaw().degrees()
                        #             )
                        #             self.find_heading = False
                        #         else:
                        #             self.wait_one_tick = True
                        # if self.robot.has_coral:
                        #     # self.robot_oriented_angle = self.robot.fieldConstants.Reef.centerFaces[self.robot.poseEstimator.calculate_closest_reef_tag()[1] - 1].rotation().degrees()
                        #     reef_center = self.robot.fieldConstants.Reef.center
                        #     cur_pose = self.robot.poseEstimator.curEstPose
                        #     vector_delta = cur_pose.translation() - reef_center
                        #     self.robot_oriented_angle = Rotation2d.fromDegrees(radiansToDegrees(math.atan2(vector_delta.y, vector_delta.x)) - 90).degrees()
                        #     self.find_heading = False
                        if not self.cardinal_directing:
                            if self.find_heading:
                                if self.tick_count <= self.tick_count_max:
                                    self.robot_oriented_angle = (
                                        self.robot.poseEstimator.getYaw().degrees()
                                    )
                                    self.tick_count += 1
                                else:
                                    self.find_heading = False
                        self.robot.drivetrain.drive_with_pid(
                            Translation2d(forward_back, left_right)
                            * const.SWERVE_MAX_SPEED,
                            self.robot_oriented_angle,
                        )

        # D1 - LETTERS

        @self.driver1.X.whenPressed
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.score_state = RobotScoringPositions.L1_Scoring
            else:
                self.robot.leds.mode = self.robot.leds.MODE_LOCKED_ON
                self.robot.raise_elevator_slightly_for_L1 = False
                self.robot.strafe_for_L1 = False
                self.robot.manual_scoring = True
                self.robot.mechanisms_at_default = False
        
        @self.driver1.A.whenPressed
        def _():
            if self.robot.one_driver_ctrl:
                if self.robot.is_climbing:
                    self.robot.retract_climber = True
                else:
                    self.robot.score_state = RobotScoringPositions.L2_Scoring
            else:
                self.robot.sim_coral_scored.clear()
        
        @self.driver1.B.whenPressed
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.score_state = RobotScoringPositions.L3_Scoring
            else:
                pass
        
        @self.driver1.Y.whenPressed
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.score_state = RobotScoringPositions.L4_Scoring
            else:
                self.robot.mechanisms_at_default = True
                self.robot.raise_elevator_slightly_for_L1 = False
                self.robot.strafe_for_L1 = False
                self.robot.manual_scoring = False
                self.robot.leds.mode = self.robot.leds.MODE_ODOMETRY
        
        # D1 - BUMPERS / TRIGGERS

        @self.driver1.RIGHT_BUMPER.whenPressed
        def _():
            if not self.robot.one_driver_ctrl:
                self.robot.is_intaking = True
                self.robot.descoring_algae = False
                self.robot.raise_elevator_slightly_for_L1 = False
                self.robot.strafe_for_L1 = False
                self.robot.leds.mode = self.robot.leds.MODE_INTAKING
                self.robot.mechanisms_at_default = False
                self.robot.funnel_intake.is_intaking = True
                self.robot.end_effector.is_intaking = True
                self.robot.at_scoring_position = False
                self.robot.at_intake_position = False
                self.robot.score_intent = False
                self.robot.score_piece = False
        @self.driver1.RIGHT_BUMPER.whenHeld
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.is_intaking = True
                self.robot.descoring_algae = False
                self.robot.raise_elevator_slightly_for_L1 = False
                self.robot.strafe_for_L1 = False
                self.robot.leds.mode = self.robot.leds.MODE_INTAKING
                self.robot.mechanisms_at_default = False
                self.robot.funnel_intake.is_intaking = True
                self.robot.end_effector.is_intaking = True
                self.robot.at_scoring_position = False
                self.robot.at_intake_position = False
                self.robot.score_intent = False
                self.robot.score_piece = False
        @self.driver1.RIGHT_BUMPER.whenReleased
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.is_intaking = False
                self.robot.leds.mode = self.robot.leds.MODE_ODOMETRY
                self.robot.funnel_intake.is_intaking = False
                self.robot.end_effector.is_intaking = False
                self.robot.mechanisms_at_default = True
        
        @self.driver1.LEFT_BUMPER.whenHeld
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.mechanisms_at_default = False
                self.robot.is_intaking = False
                self.robot.score_intent = False
                self.robot.running_pid_lineup = False
                self.robot.drivetrain.at_inter_pose = False
                self.robot.funnel_intake.is_intaking = False
                self.robot.end_effector.is_intaking = False
                self.robot.manual_scoring = True
                self.robot.score_piece = True
                self.robot.descoring_algae = True

                algae_height_at_closest_side = self.robot.poseEstimator.calculate_algae_height_at_closest_side()
                if algae_height_at_closest_side == 3:
                    self.robot.score_state = RobotScoringPositions.Descore_Algae_L3
                elif algae_height_at_closest_side == 2:
                    self.robot.score_state = RobotScoringPositions.Descore_Algae_L2
            else:
                self.robot.is_intaking = False
                self.robot.leds.mode = self.robot.leds.MODE_ODOMETRY
                self.robot.funnel_intake.is_intaking = False
                self.robot.end_effector.is_intaking = False
                self.robot.mechanisms_at_default = True
        @self.driver1.LEFT_BUMPER.whenReleased
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.descoring_algae = False
                self.robot.mechanisms_at_default = True
                self.robot.score_intent = False
                self.robot.manual_scoring = False
                self.robot.running_pid_lineup = False
                self.robot.drivetrain.at_inter_pose = False
                self.robot.funnel_intake.is_intaking = False
                self.robot.end_effector.is_intaking = False
                self.robot.manual_scoring = False
                self.robot.score_piece = False

                self.robot.score_state = RobotScoringPositions.L4_Scoring
        
        @self.driver1.RIGHT_TRIGGER_AS_BUTTON.whenHeld
        def _():
            if self.robot.one_driver_ctrl:
                if self.robot.has_coral or self.robot.funnel_intake.piece_passing_through:
                    self.robot.right_branch = True
                    self.robot.is_intaking = False
                    self.robot.descoring_algae = False
                    self.robot.leds.mode = self.robot.leds.MODE_LOCKED_ON
                    self.robot.funnel_intake.is_intaking = False
                    self.robot.end_effector.is_intaking = False
                    self.robot.funnel_intake.stop()
                    self.robot.end_effector.stop()
                    self.robot.raise_elevator_slightly_for_L1 = False
                    self.robot.at_scoring_position = False
                    self.robot.at_intake_position = False
                    self.robot.strafe_for_L1 = False
                    self.robot.score_piece = False
                    if self.robot.score_state.number == 1 and not self.robot.score_with_strafing:
                        self.robot.final_lineup_pose = self.robot.poseEstimator.get_path_to_reef(True, self.robot.poseEstimator.calculate_closest_reef_tag()[1], self.robot.right_branch, do_manip_offset=False, do_side_offset=False)
                    else:
                        self.robot.final_lineup_pose = self.robot.poseEstimator.get_path_to_reef(
                            True, # change to true if wanting to use calibrated field
                            self.robot.poseEstimator.calculate_closest_reef_tag()[1],
                            self.robot.right_branch,
                            do_manip_offset=True,
                        )
                    self.robot.mechanisms_at_default = False
                    self.robot.running_pid_lineup = True
                    self.robot.drivetrain.at_inter_pose = False
                    self.robot.score_intent = True
                else:
                    self.robot.final_lineup_pose = self.robot.poseEstimator.get_path_to_source(False, self.robot.position_on_source)
                    self.robot.score_intent = False
                    self.robot.running_pid_lineup = True
                    self.robot.drivetrain.at_inter_pose = False
                    self.robot.is_intaking = True
                    self.robot.descoring_algae = False
                    self.robot.raise_elevator_slightly_for_L1 = False
                    self.robot.strafe_for_L1 = False
                    self.robot.mechanisms_at_default = False
                    self.robot.at_scoring_position = False
                    self.robot.at_intake_position = False
                    self.robot.score_piece = False
                    self.robot.funnel_intake.is_intaking = True
                    self.robot.end_effector.is_intaking = True
                    self.robot.leds.mode = self.robot.leds.MODE_INTAKING
            else:
                self.robot.is_intaking = False
                self.robot.descoring_algae = False
                self.robot.leds.mode = self.robot.leds.MODE_LOCKED_ON
                self.robot.funnel_intake.is_intaking = False
                self.robot.end_effector.is_intaking = False
                self.robot.funnel_intake.stop()
                self.robot.end_effector.stop()
                self.robot.raise_elevator_slightly_for_L1 = False
                self.robot.at_scoring_position = False
                self.robot.at_intake_position = False
                self.robot.strafe_for_L1 = False
                self.robot.score_piece = False
                if self.robot.score_state.number == 1 and not self.robot.score_with_strafing:
                    self.robot.final_lineup_pose = self.robot.poseEstimator.get_path_to_reef(True, self.robot.poseEstimator.calculate_closest_reef_tag()[1], self.robot.right_branch, do_manip_offset=False, do_side_offset=False)
                else:
                    self.robot.final_lineup_pose = self.robot.poseEstimator.get_path_to_reef(
                        True, # change to true if wanting to use calibrated field
                        self.robot.poseEstimator.calculate_closest_reef_tag()[1],
                        self.robot.right_branch,
                        do_manip_offset=True,
                    )
                self.robot.mechanisms_at_default = False
                self.robot.running_pid_lineup = True
                self.robot.drivetrain.at_inter_pose = False
                self.robot.score_intent = True

        @self.driver1.RIGHT_TRIGGER_AS_BUTTON.whenReleased
        def _():
            if self.robot.one_driver_ctrl:
                if self.robot.has_coral or self.robot.funnel_intake.piece_passing_through:
                    self.robot.leds.mode = self.robot.leds.MODE_ODOMETRY
                    self.robot.raise_elevator_slightly_for_L1 = False
                    self.robot.strafe_for_L1 = False
                    self.robot.mechanisms_at_default = True
                    self.robot.running_pid_lineup = False
                    self.robot.drivetrain.at_inter_pose = False
                    self.robot.score_intent = False
                    self.robot.manual_scoring = False
                    self.robot.at_scoring_position = False
                    self.robot_oriented_angle = self.robot.poseEstimator.getYaw().degrees()
                else:
                    self.robot.is_intaking = False
                    self.robot.leds.mode = self.robot.leds.MODE_ODOMETRY
                    self.robot.mechanisms_at_default = True
                    self.robot.running_pid_lineup = False
                    self.robot.drivetrain.at_inter_pose = False
                    self.robot.score_intent = False
                    self.robot_oriented_angle = self.robot.poseEstimator.getYaw().degrees()
            else:
                self.robot.leds.mode = self.robot.leds.MODE_ODOMETRY
                self.robot.raise_elevator_slightly_for_L1 = False
                self.robot.strafe_for_L1 = False
                self.robot.mechanisms_at_default = True
                self.robot.running_pid_lineup = False
                self.robot.drivetrain.at_inter_pose = False
                self.robot.score_intent = False
                self.robot.manual_scoring = False
                self.robot.at_scoring_position = False
                self.robot_oriented_angle = self.robot.poseEstimator.getYaw().degrees()

        @self.driver1.LEFT_TRIGGER_AS_BUTTON.whenHeld
        def _():   
            if self.robot.one_driver_ctrl:
                if self.robot.has_coral or self.robot.funnel_intake.piece_passing_through:
                    self.robot.right_branch = False
                    self.robot.is_intaking = False
                    self.robot.descoring_algae = False
                    self.robot.leds.mode = self.robot.leds.MODE_LOCKED_ON
                    self.robot.funnel_intake.is_intaking = False
                    self.robot.end_effector.is_intaking = False
                    self.robot.funnel_intake.stop()
                    self.robot.end_effector.stop()
                    self.robot.raise_elevator_slightly_for_L1 = False
                    self.robot.at_scoring_position = False
                    self.robot.at_intake_position = False
                    self.robot.strafe_for_L1 = False
                    self.robot.score_piece = False
                    if self.robot.score_state.number == 1 and not self.robot.score_with_strafing:
                        self.robot.final_lineup_pose = self.robot.poseEstimator.get_path_to_reef(True, self.robot.poseEstimator.calculate_closest_reef_tag()[1], self.robot.right_branch, do_manip_offset=False, do_side_offset=False)
                    else:
                        self.robot.final_lineup_pose = self.robot.poseEstimator.get_path_to_reef(
                            True, # change to true if wanting to use calibrated field
                            self.robot.poseEstimator.calculate_closest_reef_tag()[1],
                            self.robot.right_branch,
                            do_manip_offset=True,
                        )
                    self.robot.mechanisms_at_default = False
                    self.robot.running_pid_lineup = True
                    self.robot.drivetrain.at_inter_pose = False
                    self.robot.score_intent = True
                else:
                    self.robot.final_lineup_pose = self.robot.poseEstimator.get_path_to_source(True, self.robot.position_on_source)
                    self.robot.score_intent = False
                    self.robot.running_pid_lineup = True
                    self.robot.drivetrain.at_inter_pose = False
                    self.robot.is_intaking = True
                    self.robot.descoring_algae = False
                    self.robot.raise_elevator_slightly_for_L1 = False
                    self.robot.strafe_for_L1 = False
                    self.robot.mechanisms_at_default = False
                    self.robot.at_scoring_position = False
                    self.robot.at_intake_position = False
                    self.robot.score_piece = False
                    self.robot.funnel_intake.is_intaking = True
                    self.robot.end_effector.is_intaking = True
                    self.robot.leds.mode = self.robot.leds.MODE_INTAKING
            else:
                self.robot.final_lineup_pose = self.robot.poseEstimator.get_path_to_source(self.robot.poseEstimator.calculate_closest_source()[0], self.robot.position_on_source)
                self.robot.score_intent = False
                self.robot.running_pid_lineup = True
                self.robot.drivetrain.at_inter_pose = False
                self.robot.is_intaking = True
                self.robot.descoring_algae = False
                self.robot.raise_elevator_slightly_for_L1 = False
                self.robot.strafe_for_L1 = False
                self.robot.mechanisms_at_default = False
                self.robot.at_scoring_position = False
                self.robot.at_intake_position = False
                self.robot.score_piece = False
                self.robot.funnel_intake.is_intaking = True
                self.robot.end_effector.is_intaking = True
                self.robot.leds.mode = self.robot.leds.MODE_INTAKING

        @self.driver1.LEFT_TRIGGER_AS_BUTTON.whenReleased
        def _():
            if self.robot.one_driver_ctrl:
                if self.robot.has_coral or self.robot.funnel_intake.piece_passing_through:
                    self.robot.leds.mode = self.robot.leds.MODE_ODOMETRY
                    self.robot.raise_elevator_slightly_for_L1 = False
                    self.robot.strafe_for_L1 = False
                    self.robot.mechanisms_at_default = True
                    self.robot.running_pid_lineup = False
                    self.robot.drivetrain.at_inter_pose = False
                    self.robot.score_intent = False
                    self.robot.manual_scoring = False
                    self.robot.at_scoring_position = False
                    self.robot_oriented_angle = self.robot.poseEstimator.getYaw().degrees()
                else:
                    self.robot.is_intaking = False
                    self.robot.leds.mode = self.robot.leds.MODE_ODOMETRY
                    self.robot.mechanisms_at_default = True
                    self.robot.running_pid_lineup = False
                    self.robot.drivetrain.at_inter_pose = False
                    self.robot.score_intent = False
                    self.robot_oriented_angle = self.robot.poseEstimator.getYaw().degrees()
            else:
                self.robot.is_intaking = False
                self.robot.leds.mode = self.robot.leds.MODE_ODOMETRY
                self.robot.mechanisms_at_default = True
                self.robot.running_pid_lineup = False
                self.robot.drivetrain.at_inter_pose = False
                self.robot.score_intent = False
                self.robot_oriented_angle = self.robot.poseEstimator.getYaw().degrees()


        ## D1 - POV

        @self.driver1.POV.LEFT.whenPressed
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.leds.mode = self.robot.leds.MODE_LOCKED_ON
                self.robot.raise_elevator_slightly_for_L1 = False
                self.robot.strafe_for_L1 = False
                self.robot.manual_scoring = True
                self.robot.mechanisms_at_default = False
        @self.driver1.POV.LEFT.whenHeld
        def _():
            if not self.robot.one_driver_ctrl:
                self.can_crash = True
        @self.driver1.POV.LEFT.whenReleased
        def _():
            if not self.robot.one_driver_ctrl:
                self.can_crash = False

        @self.driver1.POV.UP.whenPressed
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.mechanisms_at_default = True
                self.robot.raise_elevator_slightly_for_L1 = False
                self.robot.strafe_for_L1 = False
                self.robot.manual_scoring = False
                self.robot.leds.mode = self.robot.leds.MODE_ODOMETRY
                self.robot.is_intaking = False
                self.robot.score_intent = False
                self.robot.running_pid_lineup = False
                self.robot.drivetrain.at_inter_pose = False
                self.robot.funnel_intake.is_intaking = False
                self.robot.end_effector.is_intaking = False
                self.robot.descoring_algae = False
        
        @self.driver1.POV.RIGHT.whenHeld
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.mechanisms_at_default = False
                self.robot.score_piece = True
        
        @self.driver1.POV.RIGHT.whenReleased
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.score_piece = False
                self.robot.mechanisms_at_default = True
                self.robot.manual_scoring = False
        
        @self.driver1.POV.DOWN.whenPressed
        def _():
            if self.robot.one_driver_ctrl:
                if self.robot.isSimulation():
                    self.robot.sim_coral_scored.clear()
                else:
                    robot.poseEstimator.set_yaw(0.0)
                    self.robot_oriented_angle = 0.0
            else:
                self.robot.poseEstimator.set_yaw(0.0)
                self.robot_oriented_angle = 0.0

        ## D1 - START / BACK

        @self.driver1.START.whenPressed
        def _():
            if not self.robot.one_driver_ctrl:
                if self.can_crash:
                    4096 / 0
        
        @self.driver1.BACK.whenPressed
        def _():
            if not self.robot.one_driver_ctrl:
                self.robot.mechanisms_at_default = True
                self.robot.is_intaking = False
                self.robot.score_intent = False
                self.robot.manual_scoring = False
                self.robot.running_pid_lineup = False
                self.robot.drivetrain.at_inter_pose = False
                self.robot.funnel_intake.is_intaking = False
                self.robot.end_effector.is_intaking = False
                self.robot.descoring_algae = False
        
        ## D2 - LETTERS
        
        @self.driver2.Y.whenPressed
        def _():
            if not self.robot.one_driver_ctrl:
                if self.robot.is_climbing:
                    self.robot.retract_climber = False
                else:
                    self.robot.score_state = RobotScoringPositions.L4_Scoring
        
        @self.driver2.B.whenPressed
        def _():
            if not self.robot.one_driver_ctrl:
                self.robot.score_state = RobotScoringPositions.L3_Scoring
        
        @self.driver2.A.whenPressed
        def _():
            if not self.robot.one_driver_ctrl:
                if self.robot.is_climbing:
                    self.robot.retract_climber = True
                else:
                    self.robot.score_state = RobotScoringPositions.L2_Scoring
        
        @self.driver2.X.whenPressed
        def _():
            if not self.robot.one_driver_ctrl:
                self.robot.score_state = RobotScoringPositions.L1_Scoring
        
        ## D2 - BUMPERS / TRIGGERS

        @self.driver2.RIGHT_BUMPER.whenPressed
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.end_effector_canrange_for_reef_returning_bad_values = True
        @self.driver2.RIGHT_BUMPER.whenHeld
        def _():
            if not self.robot.one_driver_ctrl:
                self.robot.mechanisms_at_default = False
                self.robot.score_intent = False
                self.robot.manual_scoring = False
                self.robot.running_pid_lineup = False
                self.robot.funnel_intake.is_intaking = False
                self.robot.end_effector.is_intaking = False
                self.robot.is_climbing = True
        @self.driver2.RIGHT_BUMPER.whenReleased
        def _():
            if not self.robot.one_driver_ctrl:
                self.robot.mechanisms_at_default = True
                self.robot.score_intent = False
                self.robot.manual_scoring = False
                self.robot.running_pid_lineup = False
                self.robot.funnel_intake.is_intaking = False
                self.robot.end_effector.is_intaking = False
                self.robot.is_climbing = False
                self.robot.retract_climber = False
        
        @self.driver2.LEFT_BUMPER.whenPressed
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.end_effector_canrange_for_reef_returning_bad_values = False
        @self.driver2.LEFT_BUMPER.whenHeld
        def _():
            if not self.robot.one_driver_ctrl:
                self.robot.mechanisms_at_default = False
                self.robot.is_intaking = False
                self.robot.score_intent = False
                self.robot.manual_scoring = False
                self.robot.running_pid_lineup = False
                self.robot.drivetrain.at_inter_pose = False
                self.robot.funnel_intake.is_intaking = False
                self.robot.end_effector.is_intaking = False
                # self.robot.manual_scoring = True
                # self.robot.score_piece = True
                self.robot.descoring_algae = True

                algae_height_at_closest_side = self.robot.poseEstimator.calculate_algae_height_at_closest_side()
                if algae_height_at_closest_side == 3:
                    self.robot.score_state = RobotScoringPositions.Descore_Algae_L3
                elif algae_height_at_closest_side == 2:
                    self.robot.score_state = RobotScoringPositions.Descore_Algae_L2
        @self.driver2.LEFT_BUMPER.whenReleased
        def _():
            if not self.robot.one_driver_ctrl:
                self.robot.descoring_algae = False
                self.robot.mechanisms_at_default = True
                self.robot.score_intent = False
                self.robot.manual_scoring = False
                self.robot.running_pid_lineup = False
                self.robot.drivetrain.at_inter_pose = False
                self.robot.funnel_intake.is_intaking = False
                self.robot.end_effector.is_intaking = False
                self.robot.manual_scoring = False
                self.robot.score_piece = False

                self.robot.score_state = RobotScoringPositions.L4_Scoring
        
        @self.driver2.RIGHT_TRIGGER_AS_BUTTON.whenPressed
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.raise_setpoints += 1
                RobotScoringPositions.L1_Scoring.elevator_height += 0.5
                RobotScoringPositions.L2_Scoring.elevator_height += 0.5
                RobotScoringPositions.L3_Scoring.elevator_height += 0.5
                RobotScoringPositions.L4_Scoring.elevator_height += 0.5
                RobotScoringPositions.Descore_Algae_L3.elevator_height += 0.5
                RobotScoringPositions.Descore_Algae_L2.elevator_height += 0.5
            else:
                self.robot.right_branch = True
        
        @self.driver2.LEFT_TRIGGER_AS_BUTTON.whenPressed
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.raise_setpoints -= 1
                RobotScoringPositions.L1_Scoring.elevator_height -= 0.5
                RobotScoringPositions.L2_Scoring.elevator_height -= 0.5
                RobotScoringPositions.L3_Scoring.elevator_height -= 0.5
                RobotScoringPositions.L4_Scoring.elevator_height -= 0.5
                RobotScoringPositions.Descore_Algae_L3.elevator_height -= 0.5
                RobotScoringPositions.Descore_Algae_L2.elevator_height -= 0.5
            else:
                self.robot.right_branch = False
        
        ## D2 - POV

        @self.driver2.POV.LEFT.whenPressed
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.position_on_source = 1
            else:
                self.robot.position_on_source = 1
        
        @self.driver2.POV.UP.whenPressed
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.position_on_source = 2
            else:
                self.robot.position_on_source = 2
        
        @self.driver2.POV.RIGHT.whenPressed
        def _():
            if self.robot.one_driver_ctrl:
                self.robot.position_on_source = 3
            else:
                self.robot.position_on_source = 3
        
        @self.driver2.POV.DOWN.whenHeld
        def _():
            if not self.robot.one_driver_ctrl:
                self.robot.mechanisms_at_default = False
                self.robot.score_piece = True
        @self.driver2.POV.DOWN.whenReleased
        def _():
            if not self.robot.one_driver_ctrl:
                self.robot.score_piece = False
                self.robot.mechanisms_at_default = True
                self.robot.manual_scoring = False
        
        ## D2 - START / BACK
        
        @self.driver2.BACK.whenPressed
        def _():
            if not self.robot.one_driver_ctrl:
                self.robot.raise_setpoints += 1
                RobotScoringPositions.L1_Scoring.elevator_height += 0.5
                RobotScoringPositions.L2_Scoring.elevator_height += 0.5
                RobotScoringPositions.L3_Scoring.elevator_height += 0.5
                RobotScoringPositions.L4_Scoring.elevator_height += 0.5
                RobotScoringPositions.Descore_Algae_L3.elevator_height += 0.5
                RobotScoringPositions.Descore_Algae_L2.elevator_height += 0.5

        @self.driver2.START.whenPressed
        def _():
            if not self.robot.one_driver_ctrl:
                self.robot.raise_setpoints -= 1
                RobotScoringPositions.L1_Scoring.elevator_height -= 0.5
                RobotScoringPositions.L2_Scoring.elevator_height -= 0.5
                RobotScoringPositions.L3_Scoring.elevator_height -= 0.5
                RobotScoringPositions.L4_Scoring.elevator_height -= 0.5
                RobotScoringPositions.Descore_Algae_L3.elevator_height -= 0.5
                RobotScoringPositions.Descore_Algae_L2.elevator_height -= 0.5
        
        ## D2 - RIGHT JOYSTICK
        
        @self.driver2.RIGHT_JOY_DOWN.whenPressed
        def _():
            if not self.robot.one_driver_ctrl:
                self.robot.end_effector_canrange_for_reef_returning_bad_values = not self.robot.end_effector_canrange_for_reef_returning_bad_values