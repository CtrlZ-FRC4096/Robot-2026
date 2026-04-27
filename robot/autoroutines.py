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

    def bline_default_right_main_bot(self): # IN AUTO CHOOSER
        return SequentialCommandGroup(
            ParallelCommandGroup(self.robot.P1_T_B_R_ROBUST_BL,
                                 self.robot.coroutines.intake),
            self.robot.coroutines.drive_to_zone_trench.withTimeout(4.5),
            ParallelCommandGroup(
                self.robot.P2_B_R_NEW_BL,
                self.robot.coroutines.intake_2
            ),
            self.robot.coroutines.drive_to_zone_trench_2.withTimeout(4.5)
        )
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
    

    def test_path(self):
        return SequentialCommandGroup(
            self.robot.new_path
        )

    def right_trench_bump_safe(self, time_to_wait=0.1): # IN AUTO CHOOSER
        return SequentialCommandGroup(
            WaitCommand(0.5),
            ParallelCommandGroup(
                self.robot.SLOW_RIGHT_STEAL_OUT_BL,
                WaitCommand(time_to_wait)
            ),
            ParallelCommandGroup(
                self.robot.SLOW_RIGHT_SAFE_BL,
                self.robot.coroutines.intake
            ),
            self.robot.coroutines.drive_to_zone_trench.withTimeout(4.5),
            ParallelCommandGroup(
                self.robot.SLOW_RIGHT_STEAL_BACK_BL,
                self.robot.coroutines.intake_2
            )
        )
    
    def left_trench_bump_3bot_safe(self, time_to_wait=5):
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

    def right_trench_wait_steal(self, time_to_wait=0.1): # IN AUTO CHOOSER
        return SequentialCommandGroup(
            ParallelCommandGroup(
                self.robot.SLOW_RIGHT_STEAL_OUT_BL,
                WaitCommand(time_to_wait)
            ),
            ParallelCommandGroup(
                self.robot.SLOW_RIGHT_STEAL_BL,
                self.robot.coroutines.intake
            ),
            self.robot.coroutines.drive_to_zone_trench.withTimeout(4.5),
            ParallelCommandGroup(
                self.robot.SLOW_RIGHT_STEAL_BACK_BL,
                self.robot.coroutines.intake_2
            )
        )
    
    def left_trench_wait_steal(self, time_to_wait=0.1): # IN AUTO CHOOSER
        return SequentialCommandGroup(
            ParallelCommandGroup(
                self.robot.SLOW_LEFT_STEAL_OUT_BL,
                WaitCommand(time_to_wait)
            ),
            ParallelCommandGroup(
                self.robot.SLOW_LEFT_STEAL_BL,
                self.robot.coroutines.intake
            ),
            self.robot.coroutines.drive_to_zone_trench.withTimeout(4.5),
            ParallelCommandGroup(
                self.robot.SLOW_LEFT_STEAL_DEPOT_BL,
                self.robot.coroutines.intake_2
            ),
            self.robot.coroutines.drive_to_zone_trench_2.withTimeout(4.5)
        )
    
    def left_trench_back_wait_steal(self, time_to_wait=0.1):
        return SequentialCommandGroup(
            WaitCommand(0.5),
            ParallelCommandGroup(
                self.robot.SLOW_LEFT_STEAL_OUT_BL,
                WaitCommand(time_to_wait)
            ),
            ParallelCommandGroup(
                self.robot.SLOW_LEFT_STEAL_TRENCH_BL,
                self.robot.coroutines.intake
            ),
            self.robot.coroutines.drive_to_zone_trench.withTimeout(4.5),
            ParallelCommandGroup(
                self.robot.SLOW_LEFT_STEAL_DEPOT_BL,
                self.robot.coroutines.intake_2
            ),
            self.robot.coroutines.drive_to_zone_trench_2.withTimeout(4.5)
        )
    
    def right_trench_back_wait_steal(self, time_to_wait=0.1):
        return SequentialCommandGroup(
            WaitCommand(0.5),
            ParallelCommandGroup(
                self.robot.SLOW_RIGHT_STEAL_OUT_BL,
                WaitCommand(time_to_wait)
            ),
            ParallelCommandGroup(
                self.robot.SLOW_RIGHT_STEAL_TRENCH_BL,
                self.robot.coroutines.intake
            ),
            self.robot.coroutines.drive_to_zone_trench.withTimeout(4.5),
            ParallelCommandGroup(
                self.robot.SLOW_RIGHT_STEAL_BACK_BL,
                self.robot.coroutines.reset_after_shooting
            )
        )
    
    def default_slow_depot(self):
        return SequentialCommandGroup(
            ParallelCommandGroup(
                self.robot.DEFAULT_SLOW_DEPOT_BL,
                self.robot.coroutines.intake
            ),
            self.robot.coroutines.drive_to_zone_trench.withTimeout(6.0)
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
