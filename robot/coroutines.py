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
from robot_scoring_positions import RobotScoringPositions
from field_const import FieldConstants


class Coroutines:
    """
    Coroutines - This class defines coroutines commands that are used in robot code.
    """

    def __init__(self, robot: "Robot"):
        
        @commandify
        def move_forward():
            # start in the middle, and face battery directly towards DS wall
            yield
            vx = 3.0 if robot.fieldConstants.shouldFlip else -3.0
            robot.drivetrain.drive(Translation2d(vx, 0), 0, True, False)
            yield from robot.wait(1.0)
        self.move_forward = (move_forward)
        
        @commandify
        def tush_push_towards_yaw():
            # yaw = robot.poseEstimator.getYaw().radians()
            # vx = math.cos(yaw) * 3.0
            # vy = math.sin(yaw) * 3.0
            vx = -3.0 if robot.fieldConstants.shouldFlip else 3.0
            timer = Timer()
            timer.start()
            while not timer.hasElapsed(1.0):
                robot.drivetrain.drive(Translation2d(vx, 0), 0, True, False)
                yield
        self.tush_push_towards_yaw = (tush_push_towards_yaw)

        @commandify
        def reset_robot_after_scoring_1():
            robot.score_piece = True
            i = 0
            while i < 4:
                yield
                i += 1
            robot.mechanisms_at_default = True
            robot.score_piece = False
            robot.running_pid_lineup = False
            robot.drivetrain.at_inter_pose = False
            robot.score_intent = False
            robot.score_piece = False
            robot.at_scoring_position = False
            robot.at_intake_position = False
            robot.end_effector.stop()
            robot.has_coral = False

        @commandify
        def reset_robot_after_scoring_2():
            robot.score_piece = True
            i = 0
            while i < 4:
                yield
                i += 1
            robot.mechanisms_at_default = True
            robot.score_piece = False
            robot.running_pid_lineup = False
            robot.score_intent = False
            robot.drivetrain.at_inter_pose = False
            robot.at_scoring_position = False
            robot.at_intake_position = False
            robot.end_effector.stop()
            robot.has_coral = False

        @commandify
        def reset_robot_after_scoring_3():
            robot.score_piece = True
            i = 0
            while i < 4:
                yield
                i += 1
            robot.mechanisms_at_default = True
            robot.score_piece = False
            robot.running_pid_lineup = False
            robot.drivetrain.at_inter_pose = False
            robot.score_intent = False
            robot.at_scoring_position = False
            robot.at_intake_position = False
            robot.end_effector.stop()
            robot.has_coral = False
        
        @commandify
        def reset_robot_after_scoring_4():
            robot.score_piece = True
            i = 0
            while i < 4:
                yield
                i += 1
            robot.mechanisms_at_default = True
            robot.score_piece = False
            robot.running_pid_lineup = False
            robot.drivetrain.at_inter_pose = False
            robot.score_intent = False
            robot.at_scoring_position = False
            robot.at_intake_position = False
            robot.end_effector.stop()
            robot.has_coral = False

        self.reset_robot_after_scoring_1 = (reset_robot_after_scoring_1)
        self.reset_robot_after_scoring_2 = (reset_robot_after_scoring_2)
        self.reset_robot_after_scoring_3 = (reset_robot_after_scoring_3)
        self.reset_robot_after_scoring_4 = (reset_robot_after_scoring_4)

        @commandify
        def wait_2p_after_score_1():
            yield from robot.wait(robot.wait_score_1_2p)

        self.wait_2p_after_score_1 = (wait_2p_after_score_1)

        @commandify
        def drive_for_tush_push():
            dist_add = -9.0 if robot.fieldConstants.shouldFlip else 9.0
            robot.final_lineup_pose = Pose2d(robot.poseEstimator.curEstPose.X() + dist_add, robot.poseEstimator.curEstPose.Y(), robot.poseEstimator.getYaw())
            robot.running_pid_lineup = True
            i = 0
            while i < 8:
                i += 1
                yield
        
        self.drive_for_tush_push = (drive_for_tush_push)

        @commandify
        def score_piece_1():
            robot.is_intaking = False
            robot.score_state = RobotScoringPositions.L4_Scoring
            robot.funnel_intake.is_intaking = False
            robot.end_effector.is_intaking = False
            robot.at_scoring_position = False
            robot.at_intake_position = False
            robot.score_piece = False
            robot.final_lineup_pose = robot.poseEstimator.get_path_to_reef(
                True, # CHANGE WHEN WE HAVE FIELD CALIBRATED
                robot.score_1_face,
            	robot.score_1_right_branch,
                do_manip_offset=True
            )
            robot.right_branch = robot.score_1_right_branch
            robot.mechanisms_at_default = False
            robot.running_pid_lineup = True
            robot.drivetrain.at_inter_pose = False
            robot.score_intent = True
            while robot.has_coral:
                yield

        @commandify
        def score_piece_2():
            robot.is_intaking = False
            robot.final_lineup_pose = robot.poseEstimator.get_path_to_reef(
                True, # CHANGE WHEN WE HAVE FIELD CALIBRATED
                robot.score_2_face,
            	robot.score_2_right_branch,
                do_manip_offset=True
            )
            robot.right_branch = robot.score_2_right_branch
            robot.score_state = RobotScoringPositions.L4_Scoring
            robot.running_pid_lineup = True
            robot.drivetrain.at_inter_pose = False
            while not robot.has_coral:
                yield # wait til piece hits EE
            robot.funnel_intake.is_intaking = False
            robot.end_effector.is_intaking = False
            robot.at_scoring_position = False
            robot.at_intake_position = False
            robot.score_piece = False
            robot.mechanisms_at_default = False
            robot.running_pid_lineup = True
            robot.score_intent = True
            while robot.has_coral:
                yield

        @commandify
        def score_piece_3():
            robot.is_intaking = False
            robot.final_lineup_pose = robot.poseEstimator.get_path_to_reef(
                True, # CHANGE WHEN WE HAVE FIELD CALIBRATED
                robot.score_3_face,
            	robot.score_3_right_branch,
                do_manip_offset=True
            )
            robot.score_state = RobotScoringPositions.L4_Scoring
            robot.running_pid_lineup = True
            robot.drivetrain.at_inter_pose = False
            robot.right_branch = robot.score_3_right_branch
            while not robot.has_coral:
                yield # wait til piece hits EE
            robot.funnel_intake.is_intaking = False
            robot.end_effector.is_intaking = False
            robot.at_scoring_position = False
            robot.at_intake_position = False
            robot.score_piece = False
            robot.mechanisms_at_default = False
            robot.score_intent = True
            while robot.has_coral:
                yield

        @commandify
        def score_piece_4():
            robot.is_intaking = False
            robot.final_lineup_pose = robot.poseEstimator.get_path_to_reef(
                True, # CHANGE WHEN WE HAVE FIELD CALIBRATED
                robot.score_4_face,
            	robot.score_4_right_branch,
                do_manip_offset=True
            )
            robot.score_state = RobotScoringPositions.L4_Scoring
            robot.running_pid_lineup = True
            robot.drivetrain.at_inter_pose = False
            robot.right_branch = robot.score_4_right_branch
            while not robot.has_coral:
                yield # wait til piece hits EE
            robot.funnel_intake.is_intaking = False
            robot.end_effector.is_intaking = False
            robot.at_scoring_position = False
            robot.at_intake_position = False
            robot.score_piece = False
            robot.mechanisms_at_default = False
            robot.score_intent = True
            while robot.has_coral:
                yield

        self.score_piece_1 = (score_piece_1)
        self.score_piece_2 = (score_piece_2)
        self.score_piece_3 = (score_piece_3)
        self.score_piece_4 = (score_piece_4)

        @commandify
        def intake_coral_1():
            robot.final_lineup_pose = robot.poseEstimator.get_path_to_source(robot.left_source_auto, robot.auto_position_source, extra_dist_offset=-3.0)
            robot.mechanisms_at_default = False
            robot.at_scoring_position = False
            robot.at_intake_position = False
            robot.score_piece = False
            robot.is_intaking = True
            robot.funnel_intake.is_intaking = True
            robot.end_effector.is_intaking = True
            robot.running_pid_lineup = True
            robot.drivetrain.at_inter_pose = False
            robot.score_intent = False
            while True:
                yield
                if (robot.funnel_intake.piece_passing_through and not robot.isSimulation()) or robot.has_coral:
                    robot.final_lineup_pose = robot.poseEstimator.get_path_to_reef(
                        True,
                        robot.score_2_face,
            	        robot.score_2_right_branch,
                        do_manip_offset=True)
                    robot.is_intaking = False
                    robot.drivetrain.reset_pid_error()
                    break

        @commandify
        def intake_coral_2():
            robot.final_lineup_pose = robot.poseEstimator.get_path_to_source(robot.left_source_auto, robot.auto_position_source)
            robot.mechanisms_at_default = False
            robot.at_scoring_position = False
            robot.at_intake_position = False
            robot.score_piece = False
            robot.is_intaking = True
            robot.funnel_intake.is_intaking = True
            robot.end_effector.is_intaking = True
            robot.running_pid_lineup = True
            robot.drivetrain.at_inter_pose = False
            robot.score_intent = False
            while True:
                yield
                if (robot.funnel_intake.piece_passing_through  and not robot.isSimulation()) or robot.has_coral:
                    robot.final_lineup_pose = robot.poseEstimator.get_path_to_reef(
                        True,
                        robot.score_3_face,
            	        robot.score_3_right_branch,
                        do_manip_offset=True
                        )
                    robot.is_intaking = False
                    robot.drivetrain.reset_pid_error()
                    break

        @commandify
        def intake_coral_3():
            robot.final_lineup_pose = robot.poseEstimator.get_path_to_source(robot.left_source_auto, robot.auto_position_source)
            robot.mechanisms_at_default = False
            robot.at_scoring_position = False
            robot.at_intake_position = False
            robot.score_piece = False
            robot.is_intaking = True
            robot.funnel_intake.is_intaking = True
            robot.end_effector.is_intaking = True
            robot.running_pid_lineup = True
            robot.drivetrain.at_inter_pose = False
            robot.score_intent = False
            while True:
                yield
                if (robot.funnel_intake.piece_passing_through and not robot.isSimulation()) or robot.has_coral:
                    robot.final_lineup_pose = robot.poseEstimator.get_path_to_reef(
                        True,
                        robot.score_4_face,
            	        robot.score_4_right_branch,
                        do_manip_offset=True
                        )
                    robot.is_intaking = False
                    robot.drivetrain.reset_pid_error()
                    break

        @commandify
        def intake_coral_2p_1():
            robot.final_lineup_pose = robot.poseEstimator.get_path_to_source(robot.left_source_auto, robot.auto_position_source)
            robot.mechanisms_at_default = False
            robot.at_scoring_position = False
            robot.at_intake_position = False
            robot.score_piece = False
            robot.funnel_intake.is_intaking = True
            robot.end_effector.is_intaking = True
            robot.running_pid_lineup = True
            robot.drivetrain.at_inter_pose = False
            robot.score_intent = False
            while True:
                yield
                if (robot.funnel_intake.piece_passing_through and not robot.isSimulation()) or robot.has_coral:
                    robot.running_pid_lineup = False
                    robot.is_intaking = False
                    break

        self.intake_coral_1 = (intake_coral_1)
        self.intake_coral_2 = (intake_coral_2)
        self.intake_coral_3 = (intake_coral_3)
        self.intake_coral_2p_1 = (intake_coral_2p_1)

        @commandify
        def reset_robot_after_intaking_1():
            yield
            robot.funnel_intake.is_intaking = False
            robot.end_effector.is_intaking = False
            robot.mechanisms_at_default = True
            robot.running_pid_lineup = False
            robot.score_intent = False
            robot.oi.robot_oriented_angle = robot.poseEstimator.getYaw().degrees()
            # robot.drivetrain.stop()
            robot.has_coral = True

        @commandify
        def reset_robot_after_intaking_2():
            yield
            robot.funnel_intake.is_intaking = False
            robot.end_effector.is_intaking = False
            robot.mechanisms_at_default = True
            robot.running_pid_lineup = False
            robot.score_intent = False
            robot.oi.robot_oriented_angle = robot.poseEstimator.getYaw().degrees()
            # robot.drivetrain.stop()
            robot.has_coral = True

        self.reset_robot_after_intaking_1 = (reset_robot_after_intaking_1)
        self.reset_robot_after_intaking_2 = (reset_robot_after_intaking_2)


       