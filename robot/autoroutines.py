# This is to help vscode
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot

import math
# from limelight import Limelight

from wpimath.trajectory import TrapezoidProfile
from wpilibextra.coroutine.coroutine_command import autoroutine2command
from wpilib import Timer
import wpimath.geometry
from wpimath.units import degreesToRadians
import const
from pathplannerlib.path import PathConstraints, PathPlannerPath

# from commands import autonomous
# from commands.autonomous import DriveTrajectory
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
        #path_constraints = PathConstraints()# adjust these max speeds and accelerations for each path
        #self.p_1 = self.robot.followPathCommand("3P_1")
        
    #     )
    def move_forwards(self):
        return SequentialCommandGroup(
            self.robot.coroutines.move_forward.withTimeout(1.5)
        )
    # def one_piece_f4(self):
    #     return SequentialCommandGroup(
    #         # self.robot.coroutines.score_piece_1.withTimeout(9.0),
    #         # self.robot.coroutines.reset_robot_after_scoring_1,
    #     )
    def three_piece_f1(self):
        return SequentialCommandGroup(
            self.robot.coroutines.score_piece_1.withTimeout(2.5),
            self.robot.coroutines.reset_robot_after_scoring_1,
            self.robot.p_for_3p_f1[0],
            self.robot.coroutines.intake_coral_1,
            self.robot.p_for_3p_f1[1],
            self.robot.coroutines.score_piece_2,
            self.robot.coroutines.reset_robot_after_scoring_2,
            self.robot.p_for_3p_f1[2],
            self.robot.coroutines.intake_coral_2,
            self.robot.p_for_3p_f1[3],
            self.robot.coroutines.score_piece_3,
            self.robot.coroutines.reset_robot_after_scoring_3,
            self.robot.p_for_3p_f1[4],
            self.robot.coroutines.intake_coral_3,
        )

    def two_piece_delayed(self):
        return SequentialCommandGroup(
            self.robot.coroutines.score_piece_1.withTimeout(1.4),
            self.robot.coroutines.reset_robot_after_scoring_1,
            self.robot.coroutines.wait_2p_after_score_1,
            self.robot.p_for_2p[0],
            self.robot.coroutines.intake_coral_2p_1,
            self.robot.p_for_2p[1],
            self.robot.coroutines.score_piece_2,
            self.robot.coroutines.reset_robot_after_scoring_2,
            self.robot.p_for_2p[2],
        )

    def three_piece_auto(self):
        return SequentialCommandGroup(
            self.robot.coroutines.score_piece_1.withTimeout(1.37),
            self.robot.coroutines.reset_robot_after_scoring_1,
            self.robot.p_for_3p[0],
            self.robot.coroutines.intake_coral_1,
            self.robot.p_for_3p[1],
            self.robot.coroutines.score_piece_2,
            self.robot.coroutines.reset_robot_after_scoring_2,
            self.robot.coroutines.intake_coral_2,
            self.robot.p_for_3p[2],
            self.robot.coroutines.score_piece_3,
            self.robot.coroutines.reset_robot_after_scoring_3,
            self.robot.coroutines.intake_coral_3,
            self.robot.p_for_3p[3],
            self.robot.coroutines.score_piece_4,
        )

    def three_piece_to_f5(self):
        return SequentialCommandGroup(
            self.robot.coroutines.score_piece_1.withTimeout(3.25), #CHANGE TIME
            self.robot.coroutines.reset_robot_after_scoring_1,
            self.robot.coroutines.intake_coral_1,
            self.robot.coroutines.score_piece_2.withTimeout(3.25),
            self.robot.coroutines.reset_robot_after_scoring_2,
            self.robot.coroutines.intake_coral_2,
            self.robot.coroutines.score_piece_3.withTimeout(3.25),
            self.robot.coroutines.reset_robot_after_scoring_3,
            self.robot.coroutines.intake_coral_3,
            self.robot.coroutines.score_piece_4.withTimeout(3.25),
            self.robot.coroutines.reset_robot_after_scoring_4
        )

    def tush_push_auto(self):
        return SequentialCommandGroup(
            self.robot.coroutines.drive_for_tush_push.withTimeout(0.4),
            self.robot.coroutines.score_piece_1.withTimeout(4.25), #CHANGE TIME
            self.robot.coroutines.reset_robot_after_scoring_1,
            self.robot.coroutines.intake_coral_1,
            self.robot.coroutines.score_piece_2.withTimeout(4.0),
            self.robot.coroutines.reset_robot_after_scoring_2,
            self.robot.coroutines.intake_coral_2,
            self.robot.coroutines.score_piece_3.withTimeout(4.0),
            self.robot.coroutines.reset_robot_after_scoring_3,
            self.robot.coroutines.intake_coral_3,
            self.robot.coroutines.score_piece_4.withTimeout(3.0),
            self.robot.coroutines.reset_robot_after_scoring_4
        )