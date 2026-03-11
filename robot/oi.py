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
from wpimath.filter import SlewRateLimiter

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
        self.robot_oriented_angle = self.robot.poseEstimator.curEstPose.rotation().degrees()

        self.rumble_button = Button(lambda: self.robot.has_coral)
        self.can_crash = False

        self.find_heading = True
        self.tick_count = 0
        self.tick_count_max = 5

        self.right_trigger_being_held = False
        self.left_trigger_being_held = False

        self.accel_shoot_limiter = SlewRateLimiter(0.2, -3)

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

                if not self.robot.shoot_intent:
                    cur_speeds = self.robot.drivetrain.get_field_relative_speeds()
                    raw_mag = Translation2d(cur_speeds.vx, cur_speeds.vy).norm()
                    dummy_val = self.accel_shoot_limiter.calculate(raw_mag)   
                
                if not self.robot.running_pid_lineup and not (self.robot.track_fuel and self.robot.poseEstimator.active_intake_tgt is not None):
                    cur_speeds = self.robot.drivetrain.get_field_relative_speeds()
                    raw_mag_2 = Translation2d(cur_speeds.vx, cur_speeds.vy).norm()
                    dummy_val_2 = self.robot.drivetrain.accel_shoot_limiter.calculate(raw_mag_2)

                # if self.robot.wheels_at_x:
                #     self.robot.drivetrain.turn_wheels_to_x()
                SmartDashboard.putNumber("Test/Limit accel", self.accel_shoot_limiter.lastValue())
                if self.robot.running_pid_lineup:
                    # Cancel drive with pid if robot is moving manually
                    if (
						abs(self.driver1.LEFT_JOY_X()) > 0.05
						or abs(self.driver1.LEFT_JOY_Y()) > 0.05
						or abs(self.driver1.RIGHT_JOY_X()) > 0.1
						or abs(self.driver1.RIGHT_JOY_Y()) > 0.1
					):
                        forward_back *= 0.6
                        left_right *= 0.6
                    else:
                        forward_back = 0.0
                        left_right = 0.0
                        rotate = 0
                    if self.robot.shoot_intent:
                        rotation = self.robot.drivetrain.get_target_angle(self.robot.time_of_flight, self.robot.static_target)
                        lineup  = Pose2d(self.robot.final_lineup_pose.X(), self.robot.final_lineup_pose.Y(), rotation)
                    else:
                        lineup = self.robot.final_lineup_pose
                    self.robot.drivetrain.go_to_pose_profiled_pid(lineup, forward_back, left_right, rotate)
                # elif (abs(const.SWERVE_KINEMATICS.toChassisSpeeds(self.robot.poseEstimator.get_module_states()).vx) <= 0.005 and
                #       abs(const.SWERVE_KINEMATICS.toChassisSpeeds(self.robot.poseEstimator.get_module_states()).vy) <= 0.005 and 
                #       abs(const.SWERVE_KINEMATICS.toChassisSpeeds(self.robot.poseEstimator.get_module_states()).omega_dps) <= 1):
                #     self.robot.drivetrain.turn_wheels_to_x()
                elif self.robot.shoot_intent: #and self.robot.should_hub_track:
                    rotation_2d = self.robot.drivetrain.get_target_angle(self.robot.time_of_flight, self.robot.static_target)
                    rotation = rotation_2d.degrees()
                    mag_vel = Translation2d(forward_back, left_right).norm()
                    if mag_vel <= 1e-4 and abs((self.robot.poseEstimator.curEstPose.rotation() - rotation_2d).degrees()) <= 3:
                        self.robot.poseEstimator.set_wheels_to_x()
                    else:

                        if mag_vel > 1e-6:
                            direction = Translation2d(forward_back, left_right) / mag_vel
                        else:
                            direction = Translation2d(0, 0)

                        if mag_vel >= 0.25:
                            forward_back = (forward_back / mag_vel) * 0.25
                            left_right = (left_right / mag_vel) * 0.25
                        new_mag_vel = Translation2d(forward_back, left_right).norm()

                        limit_mag = self.accel_shoot_limiter.calculate(new_mag_vel)
                        forward_back = direction.X() * limit_mag
                        left_right = direction.Y() * limit_mag

                        self.robot_oriented_angle = rotation
                        self.robot.drivetrain.drive_with_pid(
                                Translation2d(forward_back, left_right)
                                * const.SWERVE_MAX_SPEED,
                                rotation)
                elif False and self.robot.is_intaking and ((self.robot.snake_intake and abs(rotate) <= 0.02) or (self.robot.track_fuel and self.robot.poseEstimator.active_intake_tgt is not None and False)):
                    if self.robot.snake_intake and abs(rotate) <= 0.02:
                        self.robot.drivetrain.drive_with_pid(
                                Translation2d(forward_back, left_right)
                                * const.SWERVE_MAX_SPEED,
                                self.robot.intake.get_snake_intake_angle()
                            )
                        self.robot_oriented_angle = self.robot.intake.get_snake_intake_angle()
                    elif self.robot.track_fuel and self.robot.poseEstimator.active_intake_tgt is not None and False:
                        diff_vec = self.robot.poseEstimator.active_intake_tgt - self.robot.poseEstimator.curEstPose.translation() 
                        rotation = Rotation2d(math.atan2(diff_vec.Y(), diff_vec.X()))
                        final_pose = Pose2d(
                            self.robot.poseEstimator.active_intake_tgt.X(),
                            self.robot.poseEstimator.active_intake_tgt.Y(),
                            rotation
                        )
                        SmartDashboard.putNumber("Test/Intake Track Rot", (rotation).degrees())
                        SmartDashboard.putNumber("Test/Diff X", diff_vec.X())
                        SmartDashboard.putNumber("Test/Diff Y", diff_vec.Y())
                        self.robot.drivetrain.go_to_pose_profiled_pid(final_pose)
                else:
                    if abs(rotate) >= 0.02:
                        self.cardinal_directing = False
                        self.find_heading = True
                        self.wait_one_tick = False
                        self.tick_count = 0
                        self.robot.drivetrain.drive(
                            Translation2d(forward_back, left_right)
                            * const.SWERVE_MAX_SPEED,
                            rotate * 3,
                            True,
                            False,
                        )
                        self.robot_oriented_angle = (
                            self.robot.poseEstimator.curEstPose.rotation().degrees()
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
                                        self.robot.poseEstimator.curEstPose.rotation().degrees()
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
        
        @self.driver1.LEFT_TRIGGER_AS_BUTTON.whenHeld
        def _():
            self.robot.is_intaking = True
            self.robot.intake_at_default = False
        
        @self.driver1.LEFT_TRIGGER_AS_BUTTON.whenReleased
        def _():
            self.robot.is_intaking = False
        
        @self.driver1.A.whenPressed
        def _():
            self.robot.snake_intake = not self.robot.snake_intake
            self.robot_oriented_angle = self.robot.poseEstimator.curEstPose.rotation().degrees()
        
        @self.driver1.Y.whenPressed
        def _():
            self.robot.shooter_at_default = True
            self.robot.intake_at_default = True
            self.robot.is_intaking = False
            self.robot.shoot_fuel = False
            self.robot.shoot_intent = False
        

        @self.driver1.X.whenHeld
        def _():
            self.robot.shoot_fuel = True
            self.robot.shooter_at_default = False
            self.robot.shoot_intent = False
            self.robot.is_climbing = False
        
        @self.driver1.X.whenReleased
        def _():
            self.robot.shoot_fuel = False
            self.robot.shoot_intent = False

        @self.driver1.POV.UP.whenPressed
        def _():
            self.robot.is_intaking = False
            self.robot.intake_at_default = True
            self.robot.pulse_pivot = False

        @self.driver1.POV.LEFT.whenPressed
        def _():
            self.robot.is_intaking = False
            self.robot.pulse_pivot = False
            self.robot.intake_at_default = True
            self.robot.shoot_intent = False
            self.robot.shoot_fuel = False
            self.robot.is_climbing = False
        
        @self.driver1.START.whenPressed
        def _():
            self.robot.is_intaking = True
            self.robot.intake_at_default = False
            self.robot.pulse_pivot = False
            self.robot.is_climbing = False
            self.robot.snake_intake = False
            self.robot.track_fuel = True

        @self.driver1.BACK.whenPressed
        def _():
            self.robot.is_intaking = False
            self.robot.track_fuel = False
            self.robot.pulse_pivot = False
            self.robot.is_climbing = False
            
        # @self.driver1.B.whenPressed
        # def _():
        #     self.robot.shoot_intent = not self.robot.shoot_intent
        #     self.robot.mechanisms_at_default = not self.robot.shoot_intent
        #     self.robot_oriented_angle = self.robot.poseEstimator.curEstPose.rotation().degrees()

        @self.driver1.RIGHT_TRIGGER_AS_BUTTON.whenHeld #shoot
        def _():
            self.robot.shooter_at_default = False
            self.robot.shoot_intent = True
            self.robot.shoot_fuel = False
            self.robot.is_climbing = False
        @self.driver1.RIGHT_TRIGGER_AS_BUTTON.whenReleased
        def _():
            self.robot.shoot_intent = False
            self.robot.shoot_fuel = False
            self.robot.shooter_at_default = True
            self.robot.is_climbing = False
            self.robot.shooter.shoot_ready = False
            self.robot.shooter.accel_good = False
            self.robot.pulse_pivot = False
            self.robot.intake_at_default = True

        @self.driver1.RIGHT_BUMPER.whenHeld
        def _():
            self.robot.final_lineup_pose = self.robot.poseEstimator.get_path_to_trench()
            self.robot.running_pid_lineup = True
        
        @self.driver1.RIGHT_BUMPER.whenReleased
        def _():
            self.robot.running_pid_lineup = False
            self.robot_oriented_angle = self.robot.poseEstimator.curEstPose.rotation().degrees()

        @self.driver2.RIGHT_BUMPER.whenPressed
        def _():
            self.robot.shooter.test_accelerator_speed += 1
        
        @self.driver2.LEFT_BUMPER.whenPressed
        def _():
            self.robot.shooter.test_accelerator_speed -= 1
        
        @self.driver2.POV.UP.whenPressed
        def _():
            self.robot.shooter.test_fly_speed += 1
        
        @self.driver2.POV.DOWN.whenPressed
        def _():
            self.robot.shooter.test_fly_speed -= 1
        
        @self.driver2.B.whenPressed
        def _():
            self.robot.shooter.test_hood_position += 1
        
        @self.driver2.X.whenPressed
        def _():
            self.robot.shooter.test_hood_position -= 1
            if self.robot.did_autonomous:
                self.robot.known_auto_win = False
        
        @self.driver2.Y.whenPressed
        def _():
            if self.robot.did_autonomous:
                self.robot.known_auto_win = True
        
        @self.driver2.POV.RIGHT.whenPressed
        def _():
            self.robot.intake.test_intake_speed += 1
        
        @self.driver2.POV.LEFT.whenPressed
        def _():
            self.robot.intake.test_intake_speed -= 1

        @self.driver2.A.whenPressed
        def _():
            self.robot.should_hub_track = not self.robot.should_hub_track
        
        @self.driver2.RIGHT_TRIGGER_AS_BUTTON.whenPressed
        def _():
            self.robot.is_climbing = True
            self.robot.at_climbing_position = False
            self.robot.intake_at_default = True
            self.robot.is_intaking = False
            self.robot.pulse_pivot = False
            self.robot.track_fuel = False
            self.robot.shooter_at_default = True
            self.robot.shoot_fuel = False
            self.robot.shoot_intent = False

        @self.driver2.LEFT_TRIGGER_AS_BUTTON.whenPressed
        def _():
            self.robot.is_climbing = True
            self.robot.at_climbing_position = True
            self.robot.intake_at_default = True
            self.robot.is_intaking = False
            self.robot.pulse_pivot = False
            self.robot.track_fuel = False
            self.robot.shooter_at_default = True
            self.robot.shoot_fuel = False
            self.robot.shoot_intent = False
        
        @self.driver2.START.whenPressed
        def _():
            self.robot.is_climbing = False
            self.robot.at_climbing_position = False
            self.robot.intake_at_default = True
            self.robot.is_intaking = False
            self.robot.pulse_pivot = False
            self.robot.track_fuel = False
            self.robot.shooter_at_default = True
            self.robot.shoot_fuel = False
            self.robot.shoot_intent = False