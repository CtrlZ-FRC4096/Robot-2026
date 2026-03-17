# This is to help vscode
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot

import math
# from limelight import Limelight

from wpimath.trajectory import TrapezoidProfile
from wpilibextra.coroutine.coroutine_command import autoroutine2command
from wpilib import Timer
from wpimath.geometry import Rotation2d, Pose2d
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
            ParallelCommandGroup(self.robot.getPathCommand(PathPlannerPath.fromPathFile("P1_T_L")),
                                 self.robot.coroutines.intake),
            self.robot.coroutines.drive_to_zone_trench.withTimeout(4),
            ParallelCommandGroup(self.robot.getPathCommand(PathPlannerPath.fromPathFile("P2_T_L")),
                                 self.robot.coroutines.intake_2),
            self.robot.coroutines.drive_to_zone_trench_2.withTimeout(4),
        )

    def trench_bump_left_auto(self):
        return SequentialCommandGroup(
            ParallelCommandGroup(
                self.robot.getPathCommand(PathPlannerPath.fromPathFile("P1_B_L")),
                self.robot.coroutines.intake),
            self.robot.coroutines.shoot_in_place.withTimeout(4),
            ParallelCommandGroup(
                self.robot.getPathCommand(PathPlannerPath.fromPathFile("P2_B_L")),
                self.robot.coroutines.intake_2
            ),
            self.robot.coroutines.drive_to_zone_trench
        )

    def bump_left_depot_outpost_auto(self):
        # self.robot.poseEstimator.poseEst.resetPose(Pose2d(4.440, 7.587, Rotation2d.fromDegrees(-90)))
        # self.robot.poseEstimator.curEstPose = Pose2d(4.440, 7.587, Rotation2d.fromDegrees(-90))
        return SequentialCommandGroup(
            ParallelCommandGroup(
                self.robot.getPathCommand(PathPlannerPath.fromPathFile("P1_B_L")),
                self.robot.coroutines.intake
            ),
            self.robot.coroutines.shoot_in_place,
            ParallelCommandGroup(
                self.robot.getPathCommand(PathPlannerPath.fromPathFile("LB_DEPOT")),
                self.robot.coroutines.intake_2
            ),
            self.robot.coroutines.shoot_in_place_2
        )
        