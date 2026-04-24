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
            robot.pulse_pivot = False
            robot.shoot_intent = True
            robot.spin_down = True
            robot.shooter.shoot_ready = False
            robot.shooter.accel_good = False
            # robot.shooter_at_default = True
            robot.running_pid_lineup = False
            robot.intake_at_default = False
            robot.is_intaking = True
            yield

        @commandify
        def intake_3():
            robot.pulse_pivot = False
            robot.shoot_intent = True
            robot.spin_down = True
            robot.shooter.shoot_ready = False
            robot.shooter.accel_good = False
            robot.running_pid_lineup = False
            robot.intake_at_default = False
            robot.is_intaking = True
            yield
        
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
        self.intake_3 = (intake_3)

        self.shoot_in_place = (shoot_in_place)
        self.shoot_in_place_2 = (shoot_in_place_2)

        self.stop_drive = (stop_drive)

        @commandify
        def p1_over_right_bump():
            robot.is_intaking = False
            robot.intake_at_default = False
            robot.pulse_pivot = False

            robot.final_lineup_pose = robot.fieldConstants.flip_Pose2d(Pose2d(3.0, 2.513, Rotation2d.fromDegrees(135)))
            robot.running_pid_lineup = True
            robot.run_p1 = True
            while not (robot.done_p1 or robot.poseEstimator.cur_pos_in_zone(3.2)):
                yield
            robot.running_pid_lineup = False
            robot.auto_time_since_ended_p = RobotController.getFPGATime() / 1000
            robot.run_p1 = False
            robot.done_p1 = False
            robot.shoot_intent = True

        self.p1_over_right_bump = (p1_over_right_bump)
        
        @commandify
        def p2_over_right_bump():
            robot.is_intaking = False
            robot.intake_at_default = False
            robot.pulse_pivot = False

            robot.final_lineup_pose = robot.fieldConstants.flip_Pose2d(Pose2d(3.0, 2.352, Rotation2d.fromDegrees(135)))
            robot.running_pid_lineup = True
            
            robot.run_p2 = True
            while not (robot.done_p2 or robot.poseEstimator.cur_pos_in_zone(3.25)):
                yield
            robot.running_pid_lineup = False
            robot.run_p2 = False
            robot.auto_time_since_ended_p = RobotController.getFPGATime() / 1000
            robot.done_p2 = False
            robot.shoot_intent = True
            
        
        self.p2_over_right_bump = (p2_over_right_bump)


        @commandify
        def p1_over_left_bump():
            robot.is_intaking = False
            robot.intake_at_default = False
            robot.pulse_pivot = False

            robot.final_lineup_pose = robot.fieldConstants.flip_Pose2d(Pose2d(2.807, robot.fieldConstants.fieldWidth - 2.513, Rotation2d.fromDegrees(-90)))
            robot.running_pid_lineup = True
            robot.shoot_intent = True
            robot.run_p1 = True
            while not (robot.done_p1 or robot.poseEstimator.cur_pos_in_zone(2.95)):
                yield
            robot.running_pid_lineup = False
            robot.run_p1 = False
            robot.done_p1 = False

        self.p1_over_left_bump = (p1_over_left_bump)

        @commandify
        def p1_over_left_bump_3bot():
            robot.is_intaking = False
            robot.intake_at_default = False
            robot.pulse_pivot = False

            robot.final_lineup_pose = robot.fieldConstants.flip_Pose2d(Pose2d(1.396, 5.149, Rotation2d.fromDegrees(60)))
            robot.running_pid_lineup = True
            robot.shoot_intent = True
            robot.run_p1 = True
            while not (robot.done_p1 or robot.poseEstimator.cur_pos_in_zone(2)):
                yield
            robot.running_pid_lineup = False
            robot.run_p1 = False
            robot.done_p1 = False

        self.p1_over_left_bump_3bot = (p1_over_left_bump_3bot)

        @commandify
        def p2_over_left_bump():
            robot.is_intaking = False
            robot.intake_at_default = False
            robot.pulse_pivot = False

            robot.final_lineup_pose = robot.fieldConstants.flip_Pose2d(Pose2d(2.807, robot.fieldConstants.fieldWidth - 2.48, Rotation2d.fromDegrees(45)))
            robot.running_pid_lineup = True
            robot.shoot_intent = True
            robot.run_p2 = True
            while not (robot.done_p2 or robot.poseEstimator.cur_pos_in_zone(3.25)):
                yield
            robot.running_pid_lineup = False
            robot.run_p2 = False
            robot.done_p2 = False
        
        self.p2_over_left_bump = (p2_over_left_bump)


        @commandify
        def drive_to_zone_trench_pid():
            robot.is_intaking = False
            robot.intake_at_default = False
            robot.velocity_constrain_pid = True
            # robot.final_lineup_pose = Pose2d(3.368, 8.1-0.709, Rotation2d.fromDegrees(-90))
            # robot.running_pid_lineup = True
            robot.shoot_intent = True
            robot.shooter_at_default = False
            while not robot.shooter.shoot_ready:
                yield
            robot.final_lineup_pose = robot.fieldConstants.flip_Pose2d(Pose2d(2.783, 1.039, Rotation2d()))
            robot.running_pid_lineup = True
            robot.velocity_constrain_pid = True
            while robot.fuel_in_hopper > 0:
                yield

        self.drive_to_zone_trench_pid = (drive_to_zone_trench_pid)

        @commandify
        def intake_2_pid():
            robot.pulse_pivot = False
            robot.shoot_intent = False
            robot.shooter_at_default = True
            robot.running_pid_lineup = False
            robot.intake_at_default = False
            robot.is_intaking = True
            robot.velocity_constrain_pid = False
            yield
        
        self.intake_2_pid = (intake_2_pid)

        @commandify
        def p1_right_trench():
            robot.pulse_pivot = False
            robot.shoot_intent = False
            robot.shooter_at_default = True
            robot.running_pid_lineup = False
            robot.intake_at_default = False
            robot.is_intaking = True

            robot.final_lineup_pose = robot.fieldConstants.flip_Pose2d(Pose2d(5.842, 0.652, Rotation2d.fromDegrees(-90)))
            # robot.running_pid_lineup = True
            robot.run_p1 = True
            while not robot.done_p1:
                yield
            robot.run_p1 = False
            robot.done_p1 = False
        
        @commandify
        def p2_right_trench():
            robot.final_lineup_pose = robot.fieldConstants.flip_Pose2d(Pose2d(7.735, 1.106, Rotation2d.fromDegrees(-150)))
            robot.run_p2 = True

            while not robot.done_p2:
                yield
            robot.run_p2 = False
            robot.done_p2 = False

        self.p1_right_trench = (p1_right_trench)
        self.p2_right_trench = (p2_right_trench)


        @commandify
        def reset_after_shooting():
            robot.should_rotate_trench_auto = False
            robot.pulse_pivot = False
            robot.shoot_intent = True
            robot.spin_down = True
            robot.shooter.shoot_ready = False
            robot.shooter.accel_good = False
            # robot.shooter_at_default = True
            robot.running_pid_lineup = False
            robot.intake_at_default = False
            robot.is_intaking = False
            yield

        self.reset_after_shooting = (reset_after_shooting)
        
