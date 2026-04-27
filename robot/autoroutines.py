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
from pathplannerlib.events import EventTrigger

# from commands import autonomous
# from commands.autonomous import DriveTrajectory
from coroutines import Coroutines
from commands2 import (
    Command,
    ParallelCommandGroup,
    ParallelRaceGroup,
    SequentialCommandGroup,
    WaitCommand
)

# Example for when we begin working on autonomous mode
class AutoRoutines:
    """
    A class to hold all the autonomous routines.
    """

    def __init__(self, robot: "Robot"):
        self.robot = robot

    def right_trench_bump_robust(self): # citrus type path # IN AUTO CHOOSER
        return SequentialCommandGroup(
            ParallelCommandGroup(
                self.robot.P1_T_B_R_ROBUST,
                self.robot.coroutines.intake),
            self.robot.coroutines.p1_over_right_bump,
            self.robot.coroutines.drive_to_zone_trench.withTimeout(4.5),
            ParallelCommandGroup(
                self.robot.P2_B_R,
                self.robot.coroutines.intake_2
            ),
            self.robot.coroutines.p2_over_right_bump,
            self.robot.coroutines.drive_to_zone_trench_2.withTimeout(4.25)
        )
    
    def left_trench_bump_3bot_safe(self, time_to_wait=0.1):
        return SequentialCommandGroup(
            ParallelCommandGroup(
                SequentialCommandGroup(
                    WaitCommand(1),
                    ParallelCommandGroup(
                    self.robot.SLOW_LEFT_STEAL_OUT_PP,
                    WaitCommand(6)
                    ),
                    ParallelCommandGroup(
                    self.robot.SLOW_LEFT_SAFE_PP,
                    self.robot.coroutines.intake
                    )),
                WaitCommand(12)
            ),
            self.robot.coroutines.p1_over_left_bump_3bot,
            self.robot.coroutines.drive_to_zone_trench.withTimeout(4),
            ParallelCommandGroup(
                self.robot.SLOW_LEFT_STEAL_DEPOT_PP,
                self.robot.coroutines.intake_2
            ),
            self.robot.coroutines.drive_to_zone_trench_2.withTimeout(4)
        )
    
    def right_trench_bump_robust_new(self): # not citrus type path 2 # IN AUTO CHOOSER
        return SequentialCommandGroup(
            ParallelCommandGroup(
                self.robot.P1_T_B_R_ROBUST,
                self.robot.coroutines.intake),
            self.robot.coroutines.p1_over_right_bump,
            self.robot.coroutines.drive_to_zone_trench.withTimeout(3.75),
            ParallelCommandGroup(
                self.robot.P2_B_R_NEW,
                self.robot.coroutines.intake_2
            ),
            self.robot.coroutines.p2_over_right_bump,
            self.robot.coroutines.drive_to_zone_trench_2.withTimeout(3.75),
            ParallelCommandGroup(
                self.robot.P3_B_R_BNZ,
                self.robot.coroutines.intake_3
            )
        )
    
    def right_trench_bump_robust_new_mvr(self):
        return SequentialCommandGroup(
            ParallelCommandGroup(
                self.robot.P1_T_B_R_ROBUST_MVR,
                self.robot.coroutines.intake),
            self.robot.coroutines.p1_over_right_bump,
            self.robot.coroutines.drive_to_zone_trench.withTimeout(3.75),
            ParallelCommandGroup(
                self.robot.P2_B_R_NEW_MVR,
                self.robot.coroutines.intake_2
            ),
            self.robot.coroutines.p2_over_right_bump,
            self.robot.coroutines.drive_to_zone_trench_2.withTimeout(3.75),
            ParallelCommandGroup(
                self.robot.P3_B_R_BNZ,
                self.robot.coroutines.intake_3
            )
        )
    
    def left_trench_bump_robust(self): # citrus type path # IN AUTO CHOOSER
        return SequentialCommandGroup(
            ParallelCommandGroup(
                self.robot.P1_T_B_L_ROBUST,
                self.robot.coroutines.intake
            ),
            self.robot.coroutines.p1_over_left_bump,
            self.robot.coroutines.drive_to_zone_trench.withTimeout(4.5),
            ParallelCommandGroup(
                self.robot.P2_B_L,
                self.robot.coroutines.intake_2
            ),
            self.robot.coroutines.p2_over_left_bump,
            self.robot.coroutines.drive_to_zone_trench_2.withTimeout(4.25)
        )

    def left_trench_bump_robust_new(self): # not citrus type path # IN AUTO CHOOSER
        return SequentialCommandGroup(
            ParallelCommandGroup(
                self.robot.P1_T_B_L_ROBUST,
                self.robot.coroutines.intake
            ),
            self.robot.coroutines.p1_over_left_bump,
            self.robot.coroutines.drive_to_zone_trench.withTimeout(4),
            ParallelCommandGroup(
                self.robot.P2_B_L_NEW,
                self.robot.coroutines.intake_2
            ),
            self.robot.coroutines.p2_over_left_bump,
            self.robot.coroutines.drive_to_zone_trench_2.withTimeout(5)
        )
