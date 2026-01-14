import time
import math
import numpy as np

from collections import deque

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot

from phoenix6.hardware import Pigeon2, TalonFX
from wpilib import (
    DriverStation,
    SmartDashboard,
    Timer,
    Field2d,
    AnalogAccelerometer,
    BuiltInAccelerometer,
)
from wpimath.geometry import (
    Pose2d,
    Pose3d,
    Rotation2d,
    Translation2d,
    Translation3d,
    Transform3d,
    Transform2d,
    Rotation3d,
)
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
)

from pathplannerlib.path import (
    Waypoint,
    PathPlannerPath,
    GoalEndState,
    IdealStartingState,
)
from phoenix6 import configs

# from pathplannerlib.commands import PathfindHolonomic


import const
from field_const import FieldConstants

# from leds import LEDs
# from shooter import Shooter

# from pathplannerlib.path import PathConstraints

# from commands2 import SubsystemBase
from wpilibextra.coroutine.subsystem import Subsystem
from swerve.swervemodule import SwerveModule
from wpimath.controller import PIDController
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.config import (
    # HolonomicPathFollowerConfig,
    # ReplanningConfig,
    PIDConstants,
)

from wpilib import BuiltInAccelerometer
from wpimath.filter import LinearFilter
from path_gen import PathGenerator

from pathplannerlib.path import PathPlannerTrajectory
from pathplannerlib.path import PathPlannerPath, PathConstraints
from wpimath.estimator import SwerveDrive4PoseEstimator
from photoncamera import WrapperedPhotonCamera
from wpimath.units import degreesToRadians, inchesToMeters
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

from robot_scoring_positions import RobotScoringPositions


class PoseEstimator(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot

        self.gyro = Pigeon2(const.SWERVE_PIGEON_ID, "carnivore")

        if self.robot.fieldConstants.shouldFlip:
            self.gyro_offset = 90
        else:
            self.gyro_offset = 270

        self.gyro.set_yaw(self.gyro_offset)

        self.field = Field2d()
        self.field_for_single_tag = Field2d()

        # bl, fl, br, fr

        # fl, fr, bl, br
        self.camera_X = {}
        self.camera_Y = {}
        self.camera_theta = {}


        self.modules = (
            SwerveModule(
                "front_left",
                const.SWERVE_ANGLE_OFFSET_FRONT_LEFT,
                const.SWERVE_DRIVE_MOTOR_ID_FRONT_LEFT,
                const.SWERVE_ANGLE_MOTOR_ID_FRONT_LEFT,
                const.SWERVE_CANCODER_ID_FRONT_LEFT,
                const.SWERVE_DRIVE_INVERT_FRONT_LEFT,
                const.SWERVE_ANGLE_INVERT_FRONT_LEFT,
            ),
            SwerveModule(
                "front_right",
                const.SWERVE_ANGLE_OFFSET_FRONT_RIGHT,
                const.SWERVE_DRIVE_MOTOR_ID_FRONT_RIGHT,
                const.SWERVE_ANGLE_MOTOR_ID_FRONT_RIGHT,
                const.SWERVE_CANCODER_ID_FRONT_RIGHT,
                const.SWERVE_DRIVE_INVERT_FRONT_RIGHT,
                const.SWERVE_ANGLE_INVERT_FRONT_RIGHT,
            ),
            SwerveModule(
                "back_left",
                const.SWERVE_ANGLE_OFFSET_BACK_LEFT,
                const.SWERVE_DRIVE_MOTOR_ID_BACK_LEFT,
                const.SWERVE_ANGLE_MOTOR_ID_BACK_LEFT,
                const.SWERVE_CANCODER_ID_BACK_LEFT,
                const.SWERVE_DRIVE_INVERT_BACK_LEFT,
                const.SWERVE_ANGLE_INVERT_BACK_LEFT,
            ),
            SwerveModule(
                "back_right",
                const.SWERVE_ANGLE_OFFSET_BACK_RIGHT,
                const.SWERVE_DRIVE_MOTOR_ID_BACK_RIGHT,
                const.SWERVE_ANGLE_MOTOR_ID_BACK_RIGHT,
                const.SWERVE_CANCODER_ID_BACK_RIGHT,
                const.SWERVE_DRIVE_INVERT_BACK_RIGHT,
                const.SWERVE_ANGLE_INVERT_BACK_RIGHT,
            ),
        )

        # This is to give CANCoders time to settle. Normally sleep calls are very bad but they're okay
        # here in the constructor/init.
        time.sleep(1.0)
        self.reset_modules_to_absolute()

        self.odometry = SwerveDrive4Odometry(
            const.SWERVE_KINEMATICS, self.getYaw(), self.get_module_positions()  # type: ignore
        )


        self.curEstPose = Pose2d(self.robot.fieldConstants.flip_Translation2d(Translation2d(7.170, 3.944)), self.getYaw())
        self.curEstPoseSingleTag = self.curEstPose
        self.curEstPoseGlobal = self.curEstPose

        # self.lastPeriodicEstPose = self.curEstPose

        self.poseEst = SwerveDrive4PoseEstimator(
            const.SWERVE_KINEMATICS, self.getYaw(), self.get_module_positions(), self.curEstPoseGlobal  # type: ignore
        )

        self.poseEstSingleTag = SwerveDrive4PoseEstimator(
            const.SWERVE_KINEMATICS,
            self.getYaw(),
            self.get_module_positions(),
            self.curEstPoseSingleTag,
        )

        # self.poseEst.setVisionMeasurementStdDevs((0.0001, 0.0001, 0.5))
        self.xystd = 0.3
        self.thetastd = 10.0  # .15

        self.xystd_single_tag = 0.01
        self.thetastd_single_tag = 1000.0

        # test position of camera 1 on front right module
        ROBOT_TO_CAM1 = Transform3d(
            Translation3d(-0.287, -0.3004, 0.214),  # X  # Y  # Z
            Rotation3d(
                0.0, np.deg2rad(-10.0), np.deg2rad(20.0 - 90.0)
            ),  # Roll  # Pitch  # Yaw
        )

        # Update with positionon robot
        ROBOT_TO_CAM2 = Transform3d(
            Translation3d(0.327, -0.195, 0.215),  # X  # Y  # Z
            Rotation3d(
                0.0, np.deg2rad(-15.0), np.deg2rad(-25.0 - 90)
            ),  # Roll  # Pitch  # Yaw
        )

        # # Update with positionon robot
        # # Update with positionon robot
        ROBOT_TO_CAM3 = Transform3d(
            Translation3d(0.334, 0.193, 0.732),  # X  # Y (0.282 need to change on robot)  # Z
            Rotation3d(
                np.deg2rad(-3.3), np.deg2rad(24.9), np.deg2rad(-104.8)
            ),  # Roll  # Pitch  # Yaw
        )

        # # Update with positionon robot
        ROBOT_TO_CAM4 = Transform3d(
            Translation3d(-0.302, 0.214, 0.773),  # X  # Y  # Z
            Rotation3d(
                np.deg2rad(5.0),
                np.deg2rad(-37.8),
                np.deg2rad(75.6),
            ),  # Roll  # Pitch  # Yaw
        )

        self.cams = [
            WrapperedPhotonCamera("camera_1", ROBOT_TO_CAM1),
            WrapperedPhotonCamera("camera_2", ROBOT_TO_CAM2),
            WrapperedPhotonCamera("camera_3", ROBOT_TO_CAM3),
            # WrapperedPhotonCamera("camera_4", ROBOT_TO_CAM4),
        ]

        self.poseConverge = True

        self.last_periodic_accel_x = 0
        self.last_periodic_accel_y = 0

        self.temp_rotation_check = Rotation2d()

        self.tag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)
        self.single_tag = False
        self.possible_pose_gbl = Pose2d()

        # for face in range(1,7):
        #     for side in range(2):
        #         object = self.field.getObject("l1 score " + str(face) + " " + ("right side" if side else "left side"))
        #         object.setPose(self.get_path_to_reef(True, face, side))

    def stop(self):
        print("sike this aint stoppin")

    def set_module_states(self, desired_states):
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desired_states, const.SWERVE_MAX_SPEED
        )

        for idx, module in enumerate(self.modules):
            module.set_desired_state(desired_states[idx], False)

    def get_module_states(self):
        return tuple([module.get_state() for module in self.modules])

    def get_module_positions(self):
        return tuple([module.get_position() for module in self.modules])

    def zero_gyro(self):
        self.set_yaw(0)

    def set_yaw(self, yaw):
        if self.robot.isSimulation():
            self.poseEstSingleTag.resetPose(Pose2d(self.curEstPose.X(), self.curEstPose.Y(), Rotation2d.fromDegrees(yaw)))
        self.gyro.set_yaw(yaw)
        SmartDashboard.putNumber("Gyro/Set Yaw", yaw)

    def getYaw(self):
        if const.SWERVE_INVERT_GYRO:
            return Rotation2d.fromDegrees((360 - self.gyro.get_yaw().value) % 360)
        else:
            return Rotation2d.fromDegrees(self.gyro.get_yaw().value % 360)

    def reset_modules_to_absolute(self):
        for module in self.modules:
            module.reset_to_absolute()

    def swerve_state_to_vel_vector(self, swerve_module_state: SwerveModuleState):
        return Translation2d(swerve_module_state.speed, swerve_module_state.angle)

    def is_moving(self):
        swerve_chassis = const.SWERVE_KINEMATICS.toChassisSpeeds(
            self.get_module_states()
        )
        return swerve_chassis.vx > 0.01 or swerve_chassis.vy > 0.01

    def get_skidding_ratio(self):
        if not (self.is_moving()):
            return 1  # is this ok?
        swerve_module_states = self.get_module_states()
        self.angular_velocity = const.SWERVE_KINEMATICS.toChassisSpeeds(
            swerve_module_states
        ).omega
        self.swerve_state_rotations = const.SWERVE_KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds(0, 0, self.angular_velocity)
        )
        self.swerve_states_translation_magnitudes = []

        for idx in range(len(swerve_module_states)):
            swerve_state_vector = self.swerve_state_to_vel_vector(
                swerve_module_states[idx]
            )
            swerve_state_rotation_vector = self.swerve_state_to_vel_vector(
                self.swerve_state_rotations[idx]
            )
            self.swerve_states_translation_magnitudes.append(
                (swerve_state_vector - swerve_state_rotation_vector).norm()
            )
        self.max_trans_speed = max(self.swerve_states_translation_magnitudes)
        self.min_trans_speed = min(self.swerve_states_translation_magnitudes)

        if self.min_trans_speed > 0.0:
            return self.max_trans_speed / self.min_trans_speed
        else:
            return 0.0

    def get_jerk_val(self):
        cur_accel_x = self.gyro.get_acceleration_x().value
        cur_accel_y = self.gyro.get_acceleration_y().value

        cur_jerk_x = abs(cur_accel_x - self.last_periodic_accel_x) / 0.05
        cur_jerk_y = abs(cur_accel_y - self.last_periodic_accel_y) / 0.05

        self.last_period_accel_x = cur_accel_x
        self.last_period_accel_y = cur_accel_x

        return np.sqrt(cur_jerk_x**2 + cur_jerk_y**2)

    def poseIsOffField(self, pose: Pose2d):
        trans = pose.translation()
        x = trans.X()
        y = trans.Y()
        inY = -0.5 < y < self.robot.fieldConstants.fieldWidth + 0.5
        inX = -0.5 < x < self.robot.fieldConstants.fieldLength + 0.5
        return not (inX and inY)

    def candidate_pose_OK(self, candidate_pose: Pose2d):
        if self.poseIsOffField(candidate_pose):  # Check if the robot is on the field
            return False
        elif (
            self.get_skidding_ratio() > const.SKIDDING_RATIO_MAX
        ):  # TODO: Tune this in shop
            return False
        elif self.get_jerk_val() > const.COLLISION_JERK_MAX:
            return False
        # add more elifs as conditions
        else:
            return True

    def get_path_to_closest_L1(self):
        closest_face = self.calculate_closest_reef_tag()[1]
        L1_score_right_side = self.get_path_to_L1(closest_face, True)
        L1_score_left_side = self.get_path_to_L1(closest_face, False)

        dist_to_L1_right_side = (self.curEstPose.translation() - L1_score_right_side.translation()).norm()
        dist_to_L1_left_side = (self.curEstPose.translation() - L1_score_left_side.translation()).norm()

        if dist_to_L1_left_side < dist_to_L1_right_side:
            # left side is closer
            return L1_score_left_side
        else:
            # right side is closer
            return L1_score_right_side

    def get_path_to_L1(self, face_to_score : int, right_side : bool):
        target_face = (face_to_score - 1) % 6 if right_side else (face_to_score + 1) % 6
        target_translation = self.get_path_to_reef(False, target_face, not right_side, margin_dist_offset=8.625, do_side_offset=True, do_manip_offset=False).translation()
        face_angle = self.robot.fieldConstants.flip_Rotation2d(self.robot.fieldConstants.Reef.centerFaces[target_face - 1].rotation())
        target_angle = Rotation2d()
        if right_side:
            target_angle = face_angle + Rotation2d.fromDegrees(-14 - 65)
        else:
            target_angle = face_angle + Rotation2d.fromDegrees(194 + 65)
        target_pose = Pose2d(target_translation, target_angle)
        return target_pose


    def calculate_closest_reef_tag(self):
        min_distance_to_tag = math.inf
        closest_reef_tag = None
        for tagID in self.robot.fieldConstants.reef_tags:
            tag_pose = self.tag_layout.getTagPose(tagID).toPose2d()
            distance = (self.curEstPose - tag_pose).translation().norm()
            if distance < min_distance_to_tag:
                min_distance_to_tag = distance
                closest_reef_tag = tagID
        return [closest_reef_tag, self.robot.fieldConstants.tag_to_face[closest_reef_tag]]
    
    def calculate_closest_tag_on_the_move(self):
        cur_speeds = const.SWERVE_KINEMATICS.toChassisSpeeds(self.get_module_states())
        # x seconds into the future (cur_speeds is distance over 1 second)
        future_pose = Pose2d(self.curEstPose.X() + cur_speeds.vx * 0.75, self.curEstPose.Y() + cur_speeds.vy * 0.75, Rotation2d.fromDegrees(self.curEstPose.rotation().degrees() + cur_speeds.omega_dps * 0.75))
        
        min_distance_to_tag = math.inf
        closest_reef_tag = None
        for tagID in self.robot.fieldConstants.reef_tags:
            tag_pose = self.tag_layout.getTagPose(tagID).toPose2d()
            distance = (future_pose - tag_pose).translation().norm()
            if distance < min_distance_to_tag:
                min_distance_to_tag = distance
                closest_reef_tag = tagID
        return [closest_reef_tag, self.robot.fieldConstants.tag_to_face[closest_reef_tag]]


    def get_path_to_reef(self, use_calibrated_field, face: int, right_branch: bool, margin_dist_offset=1.0, do_side_offset=True, do_manip_offset=True):
        manip_offset = 3.25

        side_offset = (inchesToMeters(6.47) if not do_manip_offset else (inchesToMeters(6.47 + manip_offset) if right_branch else inchesToMeters(6.47 - manip_offset)))  # distance b/w center of face to branch
        dist_offset = (
            (inchesToMeters(29.5) / 2) + (inchesToMeters(7.25) / 2) + inchesToMeters(margin_dist_offset)
        )  # robot size + bumper addition + error protection

        center_face_pose = self.robot.fieldConstants.flip_Pose2d(self.robot.fieldConstants.Reef.centerFaces[face - 1])
        angle_face = center_face_pose.rotation()

        # manip_distance = 38
        center_face_x = (
            center_face_pose.X() #+ inchesToMeters(manip_distance)*math.sin(angle_face.radians())
        )  # pose of center face (this is directly on the side of the reef)
        center_face_y = center_face_pose.Y() #+ inchesToMeters(manip_distance)*math.cos(angle_face.radians())
        x_offset = (
            math.cos(angle_face.radians()) * dist_offset
        )  # offsetting that pose by a set offset that extends the pose as if there's a vector from the center face with angle: angle_face
        y_offset = math.sin(angle_face.radians()) * (dist_offset)
        target_pose_face = Pose2d(
            center_face_x + x_offset, center_face_y + y_offset, angle_face
        )

        if do_side_offset:
            angle_to_branch = (
                (angle_face.degrees() + 90) % 360
                if right_branch
                else (angle_face.degrees() - 90) % 360
            )  # angle change needed to do math to get to the branch, right branch needs + 90 degrees (CCW), left_branch needs -90 (CW)

            x_offset_branch = (
                math.cos(degreesToRadians(angle_to_branch)) * side_offset
            )  # same as above, extending the pose from the point outside of the reef in the direction of the desired branch
            y_offset_branch = math.sin(degreesToRadians(angle_to_branch)) * side_offset

            target_pose_3 = Pose2d(
                target_pose_face.X() + x_offset_branch,
                target_pose_face.Y() + y_offset_branch,
                Rotation2d.fromDegrees(
                    angle_face.degrees() - 90
                ),  # don't know if this + 90 is needed, because our battery is facing forward and we want the camera side (scoring side) to face reef
            )

            if use_calibrated_field:
                alliance_color = "red" if self.robot.fieldConstants.shouldFlip else "blue"
                branch = "right" if right_branch else "left"
                target_pose_3 = self.robot.fieldConstants.ReefCalibratedToField.calibrated_data[alliance_color][branch][face]

            return target_pose_3
        else:
            return Pose2d(target_pose_face.translation(), Rotation2d.fromDegrees(angle_face.degrees() - 90))

    def calculate_closest_source(self):
        '''
        returns list [is_left_source_closest : bool, tag_of_closest_source : 12 | 13]
        tag 12, right source
        tag 13 left source
        FOR BLUE SIDE
        tag 2, right source
        tag 1, left source
        FOR RED SIDE
        '''
        curPose = self.curEstPose
        right_source_tag = 2 if self.robot.fieldConstants.shouldFlip else 12
        left_source_tag = 1 if self.robot.fieldConstants.shouldFlip else 13
        dist_to_right_source = (self.robot.fieldConstants.flip_Pose2d(self.robot.fieldConstants.CoralStation.rightCenterFace).translation() - curPose.translation()).norm()
        dist_to_left_source = (self.robot.fieldConstants.flip_Pose2d(self.robot.fieldConstants.CoralStation.leftCenterFace).translation() - curPose.translation()).norm()
        left_source_closer = True if (dist_to_left_source <= dist_to_right_source) else False
        SmartDashboard.putBoolean("left source closer", left_source_closer)
        SmartDashboard.putNumber("dist to right source", dist_to_right_source)
        SmartDashboard.putNumber("dist to left source", dist_to_left_source)
        proper_tag = left_source_tag if left_source_closer else right_source_tag

        return [left_source_closer, proper_tag]

    def get_path_to_source(self, left_source : bool, place_on_source=1, extra_dist_offset=0.0):
        '''
        Use calculate_closest source
        Place on source (default 2):
        1 - closest towards DS wall
        2- center source
        3 - closest to PROCESSOR WALL
        '''
        dist_offset = (inchesToMeters(29.5) / 2) + (inchesToMeters(7.25) / 2) + (inchesToMeters(extra_dist_offset))
        side_offset = inchesToMeters(24) + inchesToMeters(1) # test for 2

        if left_source:
            source_pose = self.robot.fieldConstants.flip_Pose2d(self.robot.fieldConstants.CoralStation.leftCenterFace)
            source_rotation = source_pose.rotation()
            x_offset = math.cos(source_rotation.radians()) * dist_offset  # offsetting that pose by a set offset that extends the pose as if there's a vector from the center face with angle: angle_face
            y_offset = math.sin(source_rotation.radians()) * dist_offset

            offset_pose = Pose2d(source_pose.X() + x_offset, source_pose.Y() + y_offset, source_rotation.rotateBy(Rotation2d.fromDegrees(90)))
            if place_on_source == 2:
                return offset_pose
            elif place_on_source == 1 or place_on_source == 3:
                x_side_offset = math.cos(degreesToRadians(source_rotation.degrees() + (-1 * 90 if place_on_source == 1 else 90))) * side_offset
                y_side_offset = math.sin(degreesToRadians(source_rotation.degrees() + (-1 * 90 if place_on_source == 1 else 90))) * side_offset
                target_pose = Pose2d(offset_pose.X() + x_side_offset, offset_pose.Y() + y_side_offset, source_rotation.rotateBy(Rotation2d.fromDegrees(90)))
                return target_pose
        else:
            source_pose = self.robot.fieldConstants.flip_Pose2d(self.robot.fieldConstants.CoralStation.rightCenterFace)
            source_rotation = source_pose.rotation()
            x_offset = math.cos(source_rotation.radians()) * dist_offset  # offsetting that pose by a set offset that extends the pose as if there's a vector from the center face with angle: angle_face
            y_offset = math.sin(source_rotation.radians()) * dist_offset

            offset_pose = Pose2d(source_pose.X() + x_offset, source_pose.Y() + y_offset, source_rotation + Rotation2d.fromDegrees(90))
            target_pose = Pose2d()
            if place_on_source == 2:
                target_pose = offset_pose
            elif place_on_source == 1 or place_on_source == 3:
                x_side_offset = math.cos(degreesToRadians(source_rotation.degrees() + (90 if place_on_source == 1 else -90))) * side_offset
                y_side_offset = math.sin(degreesToRadians(source_rotation.degrees() + (90 if place_on_source == 1 else -90))) * side_offset
                target_pose = Pose2d(offset_pose.X() + x_side_offset, offset_pose.Y() + y_side_offset, source_rotation + Rotation2d.fromDegrees(90))
            return target_pose

    def useSingleTag(self, distance=2):
        return True #Only use single tag
        return (self.curEstPose - self.tag_layout.getTagPose(self.calculate_closest_reef_tag()[0]).toPose2d()).translation().norm() < distance

    def calculate_algae_height_at_closest_side(self):
        closest_face = self.calculate_closest_reef_tag()[1]
        return (closest_face % 2) + 2 # CHANGE THIS BACK TO + 2

    def periodic(self):
        if self.robot.descoring_algae:
            algae_height_at_closest_side = self.robot.poseEstimator.calculate_algae_height_at_closest_side()
            if algae_height_at_closest_side == 3:
                self.robot.score_state = RobotScoringPositions.Descore_Algae_L3
            elif algae_height_at_closest_side == 2:
                self.robot.score_state = RobotScoringPositions.Descore_Algae_L2

        if (self.robot.score_intent) and (self.robot.previous_right_branch != self.robot.right_branch) and not self.robot.in_autonomous_mode:
            self.robot.previous_right_branch = self.robot.right_branch
            self.robot.final_lineup_pose = self.get_path_to_reef(
                    True, # change to true if wanting to use calibrated field
                    self.calculate_closest_reef_tag()[1],
                    self.robot.right_branch,
                    do_manip_offset=True,
                )
        
        if (self.robot.is_intaking) and (self.robot.previous_position_on_source != self.robot.position_on_source) and not self.robot.in_autonomous_mode:
            self.robot.previous_position_on_source = self.robot.position_on_source
            self.robot.final_lineup_pose = self.get_path_to_source(
                self.robot.poseEstimator.calculate_closest_source()[0], 
                self.robot.position_on_source, 
                extra_dist_offset=-3.0
            )


        allianceColor = DriverStation.getAlliance()
        self.single_tag_IDs = set()
        single_tag_poses = []

        for idx, cam in enumerate(self.cams):
            cam.update(
                self.curEstPoseGlobal,
                self.curEstPoseSingleTag,
                allianceColor=allianceColor,
                yaw=self.getYaw(),
            )

            # observations = cam.getPoseEstimates()
            tags = cam.getTagPositions()
            single_tag_poses = cam.getPoseSingleTag()
            self.single_tag_IDs.update(cam.getSingleTagIDs())
            observations = cam.getPoseEstimates()
            tag_distances = cam.getTagDistances()
            # filter by closest based on global pose

            self.tag_dist = 0.0
            self.theta_modifier = 1.0
            self.xy_modifier = 1.0

            self.theta_modifier_single_tag = 1.0
            self.xy_modifier_single_tag = 1.0

            if len(tags) > 0:
                self.tag_dist = sum(tag_distances) / len(tag_distances)

            if len(tags) == 1:
                self.theta_modifier = 1000.0
            # if (
            #     self.tag_dist > 3.5
            # ):  # if the robot is more than 4 meters away from the target
            #     self.xy_modifier = 100.0
            #     self.theta_modifier = 100.0

            for observation in observations:
                self.poseEst.addVisionMeasurement(
                    observation.estFieldPose,
                    observation.time,
                    (
                        self.xystd
                        * (self.tag_dist**2)
                        * self.xy_modifier,  # * (min_ambiguity / 0.4),
                        self.xystd
                        * (self.tag_dist**2)
                        * self.xy_modifier,  # * (min_ambiguity / 0.4),
                        self.thetastd
                        * (self.tag_dist**2)
                        * self.theta_modifier,  # * (min_ambiguity / 0.4),
                    ),
                )
                if not (
                    (observation.estFieldPose - self.poseEst.getEstimatedPosition())
                    .translation()
                    .norm()
                    <= 0.5
                ):
                    self.robot.leds.mode = self.robot.leds.MODE_ODOMETRY
                    self.poseConverge = False
                else:
                    self.robot.leds.mode = self.robot.leds.MODE_LOST_ODOMETRY
                self.camTargetsVisible = True
            # self.telemetry.addVisionObservations(observations) #Might need later https://github.com/RobotCasserole1736/RobotCasserole2024/blob/fa033322e6f4efe87e8b1af938d8a3f69599f29b/drivetrain/poseEstimation/drivetrainPoseTelemetry.py#L15

            for pose in single_tag_poses:
                self.camera_X[cam.camName] = pose.X()
                self.camera_Y[cam.camName] = pose.Y()
                self.camera_theta[cam.camName] = pose.rotation().degrees()
                # if abs(pose.rotation().degrees() - self.getYaw().degrees()) > 
                self.poseEstSingleTag.addVisionMeasurement(
                    pose,
                    cam.getObsTime(),
                    (
                        self.xystd_single_tag
                        * self.xy_modifier_single_tag,  # * (min_ambiguity / 0.4),
                        self.xystd_single_tag
                        * self.xy_modifier_single_tag,  # * (min_ambiguity / 0.4),
                        self.thetastd_single_tag
                        * self.theta_modifier_single_tag,  # * (min_ambiguity / 0.4),
                    ),
                )

        # Update poses with drivetrain information
        self.poseEst.update(self.getYaw(), self.get_module_positions())
        self.poseEstSingleTag.update(self.getYaw(), self.get_module_positions())

        possible_pose_global = self.poseEst.getEstimatedPosition()
        self.possible_pose_gbl = possible_pose_global

        possible_pose_single_tag = self.poseEstSingleTag.getEstimatedPosition()

        self.single_tag = False
        if self.candidate_pose_OK(possible_pose_global):
            self.curEstPoseGlobal = possible_pose_global
        if self.candidate_pose_OK(possible_pose_single_tag):
            self.curEstPoseSingleTag = possible_pose_single_tag


        # ALWAYS USING SINGLE
        if not self.robot.isSimulation():
            if self.robot.running_pid_lineup or True:
                if self.useSingleTag() or self.robot.is_intaking:
                    # (((self.robot.final_lineup_pose.translation() - self.robot.fieldConstants.flip_Translation2d(self.robot.fieldConstants.CoralStation.rightCenterFace.translation())).norm() < 0.75) or ((self.robot.final_lineup_pose.translation() - self.robot.fieldConstants.flip_Translation2d(self.robot.fieldConstants.CoralStation.leftCenterFace.translation())).norm() < 0.75))
                    self.curEstPose = self.curEstPoseSingleTag
                    self.single_tag = True
                else:
                    self.curEstPose = self.curEstPoseGlobal
                    self.single_tag = False
            else:
                self.curEstPose = self.curEstPoseGlobal
                self.single_tag = False

        # if (self.robot.leds.mode == self.robot.leds.MODE_LOST_ODOMETRY) or (
        #     self.robot.leds.mode == self.robot.leds.MODE_ODOMETRY
        # ):
        #     if not self.poseConverge:
        #         self.robot.leds.set_mode(self.robot.leds.MODE_LOST_ODOMETRY)
        #     elif self.poseConverge:
        #         self.robot.leds.set_mode(self.robot.leds.MODE_ODOMETRY)
        self.poseConverge = True

        self.odometry.update(self.getYaw(), self.get_module_positions())


    def log(self):
        for idx in range(len(self.cams)):
            try:
                SmartDashboard.putNumber(self.cams[idx].camName + " X", self.camera_X[self.cams[idx].camName])
                SmartDashboard.putNumber(self.cams[idx].camName + " Y", self.camera_Y[self.cams[idx].camName])
                SmartDashboard.putNumber(self.cams[idx].camName + " THETA", self.camera_theta[self.cams[idx].camName])
            except:
                continue

        SmartDashboard.putBoolean("single tag :3", self.single_tag)
        SmartDashboard.putBoolean(
            "pose 4 u :3", self.candidate_pose_OK(self.possible_pose_gbl)
        )

        SmartDashboard.putNumber("gyro voltage", self.gyro.get_supply_voltage().value)

        SmartDashboard.putNumber(
            "Single/Global Pose Diff",
            self.curEstPose.translation().norm()
            - self.curEstPoseSingleTag.translation().norm(),
        )

        SmartDashboard.putNumber("Camera/Odometry X", self.curEstPose.x)
        SmartDashboard.putNumber("Camera/Odometry Y", self.curEstPose.y)
        SmartDashboard.putNumber(
            "Camera/Odometry Theta", self.curEstPose.rotation().degrees()
        )

        SmartDashboard.putNumber(
            "Swerve/Odometry X", self.odometry.getPose().x_feet * 0.305
        )
        SmartDashboard.putNumber(
            "Swerve/Odometry Y", self.odometry.getPose().y_feet * 0.305
        )
        SmartDashboard.putNumber(
            "Swerve/Odometry Theta", self.odometry.getPose().rotation().degrees()
        )
        SmartDashboard.putNumber("Gyro/Yaw", self.getYaw().degrees())
        SmartDashboard.putNumber("Gyro/Roll", self.gyro.get_roll().value)

        SmartDashboard.putData("Field", self.field)
        self.field.setRobotPose(self.curEstPose)
        final_lineup = self.field.getObject("target pose")
        final_lineup.setPose(self.robot.final_lineup_pose)

        SmartDashboard.putData("Field w/ Single Tag", self.field_for_single_tag)
        self.field_for_single_tag.setRobotPose(
            self.poseEstSingleTag.getEstimatedPosition()
        )

        SmartDashboard.putNumber("closest reef tag", self.calculate_closest_reef_tag()[1])

        SmartDashboard.putNumber(
            "rotation of target pose: ", self.temp_rotation_check.degrees()
        )

        SmartDashboard.putBoolean("right branch", self.robot.right_branch)
        SmartDashboard.putNumber("face to path ", self.robot.oi.face)

        SmartDashboard.putNumber("skidding ratio", self.get_skidding_ratio())
        SmartDashboard.putNumber("jerk val", self.get_jerk_val())

        SmartDashboard.putNumber("pose x", self.curEstPoseSingleTag.translation().X())
        SmartDashboard.putNumber("pose y", self.curEstPoseSingleTag.translation().Y())
        SmartDashboard.putNumber("pose theta", self.curEstPoseSingleTag.rotation().degrees())

        for module in self.modules:
            SmartDashboard.putNumber(f"Swerve/{module.module_name}/Cancoder Angle", module.get_angle_CANcoder().degrees())  # type: ignore
            SmartDashboard.putNumber(f"Swerve/{module.module_name}/Motor Angle", module.get_position().angle.degrees())  # type: ignore
            SmartDashboard.putNumber(
                f"Swerve/{module.module_name}/Velcoity", module.get_state().speed
            )