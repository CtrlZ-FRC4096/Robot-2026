# This is to help vscode
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot
# ------------------------------------------------

from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Translation2d,
)

import math
from wpilib import Timer

from wpilibextra.coroutine import commandify
import oi
# from robot_scoring_positions import RobotScoringPositions
from field_const import FieldConstants
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.commands import FollowPathCommand 


class Coroutines:
    """
    Coroutines - This class defines coroutines commands that are used in robot code.
    """

    def __init__(self, robot: "Robot"):

        @commandify
        def sprint_out():
            pass

        @commandify
        def intake():
            robot.intake_at_default = False
            robot.is_intaking = True
            yield

        

        @commandify
        def drive_to_zone_no_intake():
            robot.is_intaking = False
            robot.intake_at_default = True
            robot.final_lineup_pose = Pose2d(3.368, 8.1-0.709, Rotation2d(math.pi))
            robot.running_pid_lineup = True
            robot.shoot_intent = True
            robot.shooter_at_default = False
            yield

        self.drive_to_zone_no_intake = (drive_to_zone_no_intake)
        self.intake = (intake)
