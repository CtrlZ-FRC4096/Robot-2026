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
from wpimath.units import inchesToMeters, degreesToRadians, radiansToDegrees
# from phoenix5 import NeutralMode
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

        ## D1 - POV
        
        @self.driver1.POV.DOWN.whenPressed
        def _():
            robot.poseEstimator.set_yaw(0.0)
            self.robot_oriented_angle = 0.0
        
        @self.driver1.A.whenPressed
        def _():
            self.robot.is_intaking = not self.robot.is_intaking
            