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
            robot.pulse_pivot = False
            robot.shoot_intent = False
            robot.shooter_at_default = True
            robot.running_pid_lineup = False
            robot.intake_at_default = False
            robot.is_intaking = True
            yield
        
        @commandify
        def intake_2():
            robot.should_rotate_trench_auto = False
            robot.pulse_pivot = False
            robot.shoot_intent = False
            robot.shooter_at_default = True
            robot.running_pid_lineup = False
            robot.intake_at_default = False
            robot.is_intaking = True
            yield

        @commandify
        def reline_up_with_right_trench():
            robot.final_lineup_pose = robot.fieldConstants.flip_Pose2d(Pose2d(5.844, 0.628, Rotation2d.fromDegrees(90)))
            robot.is_intaking = False
            robot.intake_at_default = False
            robot.shoot_intent = False
            robot.pulse_pivot = False
            robot.running_pid_lineup = True
            robot.lining_with_trench = True
            while not robot.at_trench_position:
                yield


        
        @commandify
        def spin_for_trench():
            robot.intake_at_default = False
            robot.shooter_at_default = True
            robot.shoot_intent = False
            robot.pulse_pivot = False
            robot.running_pid_lineup = False
            robot.should_rotate_trench_auto = True
            while abs(robot.poseEstimator.curEstPose.rotation().degrees() - robot.auto_rotation_trench) >= 10:
                yield
            robot.drivetrain.stop()
        
        @commandify
        def stop_drive():
            robot.drivetrain.stop()
            yield

        @commandify
        def drive_to_zone_trench():
            robot.is_intaking = False
            robot.intake_at_default = False
            # robot.final_lineup_pose = Pose2d(3.368, 8.1-0.709, Rotation2d.fromDegrees(-90))
            # robot.running_pid_lineup = True
            robot.shoot_intent = True
            robot.shooter_at_default = False
            while robot.fuel_in_hopper > 0:
                yield
        
        @commandify
        def drive_to_zone_trench_2():
            robot.is_intaking = False
            robot.intake_at_default = False
            # robot.final_lineup_pose = Pose2d(3.368, 8.1-0.709, Rotation2d.fromDegrees(-90))
            # robot.running_pid_lineup = True
            robot.shoot_intent = True
            robot.shooter_at_default = False
            while robot.fuel_in_hopper > 0:
                yield

        @commandify
        def shoot_in_place():
            robot.is_intaking = False
            robot.intake_at_default = False
            robot.shoot_intent = True
            robot.shooter_at_default = False
            while robot.fuel_in_hopper > 0:
                yield
        
        @commandify
        def shoot_in_place_2():
            robot.is_intaking = False
            robot.intake_at_default = False
            robot.shoot_intent = True
            robot.shooter_at_default = False
            while robot.fuel_in_hopper > 0:
                yield

        self.drive_to_zone_trench = (drive_to_zone_trench)
        self.drive_to_zone_trench_2 = (drive_to_zone_trench_2)
        self.intake = (intake)
        self.intake_2 = (intake_2)

        self.shoot_in_place = (shoot_in_place)
        self.shoot_in_place_2 = (shoot_in_place_2)

        self.spin_for_trench = (spin_for_trench)
        self.stop_drive = (stop_drive)

        self.reline_up_with_right_trench = (reline_up_with_right_trench)

