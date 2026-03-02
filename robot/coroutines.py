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
        def intake_2():
            robot.pulse_pivot = False
            robot.shoot_intent = False
            robot.shooter_at_default = True
            robot.running_pid_lineup = False
            robot.intake_at_default = False
            robot.is_intaking = True
            yield

        

        @commandify
        def drive_to_zone_no_intake():
            robot.is_intaking = False
            robot.intake_at_default = True
            robot.final_lineup_pose = Pose2d(3.368, 8.1-0.709, Rotation2d())
            robot.running_pid_lineup = True
            robot.shoot_intent = True
            robot.shooter_at_default = False
            while robot.fuel_in_hopper > 0:
                yield
        
        @commandify
        def drive_to_zone_no_intake_2():
            robot.is_intaking = False
            robot.intake_at_default = True
            if robot.fuel_in_hopper < 10:
                robot.fuel_in_hopper = 10
            Pose2d(3.368, 8.1-0.709, Rotation2d())
            robot.running_pid_lineup = True
            robot.shoot_intent = True
            robot.shooter_at_default = False
            while robot.fuel_in_hopper > 0:
                yield

        @commandify
        def climb_from_left():
            robot.is_intaking = False
            robot.pulse_pivot = False
            robot.intake_at_default = True
            robot.shoot_intent = False
            robot.shoot_fuel = False
            robot.shooter_at_default = True
            robot.final_lineup_pose = Pose2d(1.003, 4.637, Rotation2d(math.pi / 2))
            robot.running_pid_lineup = True
            while robot.running_pid_lineup:
                yield
        
        @commandify
        def climb():
            yield

        self.drive_to_zone_no_intake = (drive_to_zone_no_intake)
        self.drive_to_zone_no_intake_2 = (drive_to_zone_no_intake_2)
        self.intake = (intake)
        self.intake_2 = (intake_2)

        self.climb_from_left = (climb_from_left)
        self.climb = (climb)
