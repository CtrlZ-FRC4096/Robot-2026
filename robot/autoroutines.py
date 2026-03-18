# This is to help vscode
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot

import math
# from limelight import Limelight

from wpimath.trajectory import TrapezoidProfile
from wpilibextra.coroutine.coroutine_command import autoroutine2command
from wpilib import Timer
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.units import degreesToRadians
import const
from pathplannerlib.path import PathConstraints, PathPlannerPath

# from commands import autonomous
# from commands.autonomous import DriveTrajectory
from coroutines import Coroutines
from commands2 import (
    Command,
    ParallelCommandGroup,
    ParallelRaceGroup,
    SequentialCommandGroup,
)

# Example for when we begin working on autonomous mode
class AutoRoutines:
    """
    A class to hold all the autonomous routines.
    """

    def __init__(self, robot: "Robot"):
        self.robot = robot

    def trench_left_auto(self):
        return SequentialCommandGroup(
            ParallelCommandGroup(self.robot.P1_T_L,
                                 self.robot.coroutines.intake),
            self.robot.coroutines.drive_to_zone_trench.withTimeout(3),
            ParallelCommandGroup(self.robot.P2_T_L,
                                 self.robot.coroutines.intake_2),
            self.robot.coroutines.drive_to_zone_trench_2.withTimeout(3),
        )
    def trench_left_safe_auto(self):
        if self.robot.fieldConstants.shouldFlip:
            gyro_offset = 90
        else:
            gyro_offset = -90
        self.robot.poseEstimator.gyro.set_yaw(gyro_offset)
        self.robot.poseEstimator.poseEst.resetPose(Pose2d(self.robot.fieldConstants.flip_Translation2d(Translation2d(4.471, 7.381)), Rotation2d.fromDegrees(gyro_offset)))
        self.robot.poseEstimator.curEstPose = Pose2d(self.robot.fieldConstants.flip_Translation2d(Translation2d(4.471, 7.381)), Rotation2d.fromDegrees(gyro_offset))
        return SequentialCommandGroup(
            ParallelCommandGroup(self.robot.P1_T_L_SAFE,
                                 self.robot.coroutines.intake),
            self.robot.coroutines.drive_to_zone_trench.withTimeout(3),
            ParallelCommandGroup(self.robot.P2_T_L,
                                 self.robot.coroutines.intake_2),
            self.robot.coroutines.drive_to_zone_trench_2
        )
    def trench_bump_left_auto(self):
        return SequentialCommandGroup(
            ParallelCommandGroup(
                self.robot.P1_B_L,
                self.robot.coroutines.intake),
            self.robot.coroutines.shoot_in_place.withTimeout(3),
            ParallelCommandGroup(
                self.robot.P2_B_L,
                self.robot.coroutines.intake_2
            ),
            self.robot.coroutines.drive_to_zone_trench
        )

    def bump_left_depot_outpost_auto(self):
        # self.robot.poseEstimator.poseEst.resetPose(Pose2d(4.440, 7.587, Rotation2d.fromDegrees(-90)))
        # self.robot.poseEstimator.curEstPose = Pose2d(4.440, 7.587, Rotation2d.fromDegrees(-90))
        return SequentialCommandGroup(
            ParallelCommandGroup(
                self.robot.P1_B_L,
                self.robot.coroutines.intake
            ),
            self.robot.coroutines.shoot_in_place,
            ParallelCommandGroup(
                self.robot.LB_DEPOT,
                self.robot.coroutines.intake_2
            ),
            self.robot.coroutines.shoot_in_place_2
        )
        