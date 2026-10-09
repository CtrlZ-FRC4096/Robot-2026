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
from wpilib import Timer, RobotController

from wpilibextra.coroutine import commandify
import oi
# from robot_scoring_positions import RobotScoringPositions
from field_const import FieldConstants
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.commands import FollowPathCommand
from state_machine import Event


class Coroutines:
    """
    Coroutines - This class defines coroutines commands that are used in robot code.

    Each autonomous step sends an Event to robot.state (state_machine.py);
    the subsystems then act on the new states.
    """

    def __init__(self, robot: "Robot"):
        state = robot.state

        @commandify
        def sprint_out():
            pass

        @commandify
        def intake():
            state.handle(Event.AUTO_INTAKE)
            yield

        @commandify
        def intake_2():
            # Intake while the shooter spins down from the previous volley.
            state.handle(Event.AUTO_INTAKE_AND_SPIN_DOWN)
            yield

        @commandify
        def intake_3():
            state.handle(Event.AUTO_INTAKE_AND_SPIN_DOWN)
            yield

        @commandify
        def stop_drive():
            robot.drivetrain.stop()
            yield

        @commandify
        def drive_to_zone_trench():
            # robot.final_lineup_pose = Pose2d(3.368, 8.1-0.709, Rotation2d.fromDegrees(-90))
            state.handle(Event.AUTO_SHOOT)
            while robot.fuel_in_hopper > 0:
                yield

        @commandify
        def drive_to_zone_trench_2():
            # robot.final_lineup_pose = Pose2d(3.368, 8.1-0.709, Rotation2d.fromDegrees(-90))
            state.handle(Event.AUTO_SHOOT)
            while robot.fuel_in_hopper > 0:
                yield

        @commandify
        def shoot_in_place():
            state.handle(Event.AUTO_SHOOT)
            while robot.fuel_in_hopper > 0:
                yield

        @commandify
        def shoot_in_place_2():
            state.handle(Event.AUTO_SHOOT)
            while robot.fuel_in_hopper > 0:
                yield


        self.drive_to_zone_trench = (drive_to_zone_trench)
        self.drive_to_zone_trench_2 = (drive_to_zone_trench_2)
        self.intake = (intake)
        self.intake_2 = (intake_2)
        self.intake_3 = (intake_3)

        self.shoot_in_place = (shoot_in_place)
        self.shoot_in_place_2 = (shoot_in_place_2)

        self.stop_drive = (stop_drive)

        def lineup_over_bump(target, zone_depth):
            """Line up on target until there (or inside the zone), then aim."""
            state.start_lineup(Event.AUTO_LINEUP_STARTED, robot.fieldConstants.flip_Pose2d(target))
            while not (state.lineup_arrived or robot.poseEstimator.cur_pos_in_zone(zone_depth)):
                yield
            robot.auto_time_since_ended_p = RobotController.getFPGATime() / 1000
            state.handle(Event.AUTO_LINEUP_FINISHED)

        @commandify
        def p1_over_right_bump():
            yield from lineup_over_bump(Pose2d(3.0, 2.513, Rotation2d.fromDegrees(-90)), 3.2)

        self.p1_over_right_bump = (p1_over_right_bump)

        @commandify
        def p2_over_right_bump():
            yield from lineup_over_bump(Pose2d(3.0, 2.352, Rotation2d.fromDegrees(-90)), 3.25)

        self.p2_over_right_bump = (p2_over_right_bump)


        @commandify
        def p1_over_left_bump():
            yield from lineup_over_bump(Pose2d(3.0, 5.556, Rotation2d.fromDegrees(-90)), 3.2)

        self.p1_over_left_bump = (p1_over_left_bump)

        @commandify
        def p1_over_left_bump_3bot():
            robot.time_3_bot = RobotController.getFPGATime() / 1000
            state.start_lineup(Event.AUTO_LINEUP_STARTED,
                               robot.fieldConstants.flip_Pose2d(Pose2d(1.396, 5.149, Rotation2d.fromDegrees(60)))) # 1.396, 5.149 60
            while ((RobotController.getFPGATime() / 1000) - robot.time_3_bot) < 3200:  #not (robot.done_p1 or robot.poseEstimator.cur_pos_in_zone(3.2)):
                yield
            robot.auto_time_since_ended_p = RobotController.getFPGATime() / 1000
            state.handle(Event.AUTO_LINEUP_FINISHED)

        self.p1_over_left_bump_3bot = (p1_over_left_bump_3bot)

        @commandify
        def p2_over_left_bump():
            yield from lineup_over_bump(Pose2d(3.0, 5.626, Rotation2d.fromDegrees(-90)), 3.25)

        self.p2_over_left_bump = (p2_over_left_bump)


        @commandify
        def drive_to_zone_trench_pid():
            # robot.final_lineup_pose = Pose2d(3.368, 8.1-0.709, Rotation2d.fromDegrees(-90))
            state.handle(Event.AUTO_SHOOT)
            while not robot.shooter.shoot_ready:
                yield
            state.start_lineup(Event.AUTO_LINEUP_STARTED,
                               robot.fieldConstants.flip_Pose2d(Pose2d(2.783, 1.039, Rotation2d())),
                               speed_limited=True)
            while robot.fuel_in_hopper > 0:
                yield

        self.drive_to_zone_trench_pid = (drive_to_zone_trench_pid)

        @commandify
        def intake_2_pid():
            state.handle(Event.AUTO_INTAKE)
            yield

        self.intake_2_pid = (intake_2_pid)


        @commandify
        def reset_after_shooting():
            state.handle(Event.AUTO_STOP_SHOOTING)
            yield

        self.reset_after_shooting = (reset_after_shooting)

