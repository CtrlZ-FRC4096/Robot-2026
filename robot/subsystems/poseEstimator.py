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

from pathplannerlib.path import PathConstraints

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

from pathplannerlib.path import PathPlannerTrajectory
from pathplannerlib.path import PathPlannerPath, PathConstraints
from wpimath.estimator import SwerveDrive4PoseEstimator
from photoncamera import WrapperedPhotonCameraTag, WrapperedPhotonCameraIntakeFuel
from wpimath.units import degreesToRadians, inchesToMeters
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout



class PoseEstimator(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot

        self.gyro = Pigeon2(const.SWERVE_PIGEON_ID, "carnivore")
        self.gyro.configurator.apply(configs.Pigeon2Configuration().with_mount_pose(configs.MountPoseConfigs().with_mount_pose_roll(180)))
        

        if self.robot.fieldConstants.shouldFlip:
            self.gyro_offset = 90
        else:
            self.gyro_offset = -90

        self.gyro.set_yaw(self.gyro_offset)
        # self.gyro.set_yaw(0)

        self.field = Field2d()

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

        # self.odometry = SwerveDrive4Odometry(
        #     const.SWERVE_KINEMATICS, self.getYaw(), self.get_module_positions()  # type: ignore
        # )


        # self.curEstPose = Pose2d(4.44, 8.1-0.641, math.pi)

        # self.curEstPose = Pose2d(self.robot.fieldConstants.flip_Translation2d(Translation2d(3.524, 4.064)), Rotation2d())
        
        # self.curEstPose = Pose2d(4.414, 7.587, self.getYaw())
        self.curEstPose = Pose2d(self.robot.fieldConstants.flip_Translation2d(Translation2d(4.471, 7.381)), self.getYaw())
        self.estZ = 0

        self.poseEst = SwerveDrive4PoseEstimator(
            const.SWERVE_KINEMATICS, self.getYaw(), self.get_module_positions(), self.curEstPose # type: ignore
        )

        self.xystd_single_tag = 0.05
        self.thetastd_single_tag = 1000.0

        ROBOT_TO_CAM1 = Transform3d(
            Translation3d(-0.277, 0.274, 0.491),
            Rotation3d.fromDegrees(0.0, 0.0, -165.0)
        ) # CLIMBER SIDE CAMERA
        
        ROBOT_TO_CAM2 = Transform3d(
            Translation3d(0.130, 0.335, 0.311),
            Rotation3d.fromDegrees(0, -10, 90)
        ) # SHOOTER BACK CAMERA (TO DO)

        ROBOT_TO_CAM3 = Transform3d(
            Translation3d(0.0, 0.068, 0.514),
            Rotation3d.fromDegrees(0, -20, -90)
        ) # FLYWHEEL BAR CAMERA

        # ROBOT_TO_COLOR_1 = Transform3d() # TO DO
        # ROBOT_TO_COLOR_2 = Transform3d() # TO DO

        self.cams = [
            WrapperedPhotonCameraTag("flywheel", ROBOT_TO_CAM1),
            WrapperedPhotonCameraTag("shooter", ROBOT_TO_CAM2),
            WrapperedPhotonCameraTag("climber", ROBOT_TO_CAM3)
        ]

        # camera4 - spare1
        # camera5 - spare2
        # camera6 - spare3
        # camera7 - spare4

        # self.intake_cam = WrapperedPhotonCameraIntakeFuel("color1", ROBOT_TO_COLOR_1) # WRONG NAME MAYBE
        # self.hopper_cam = WrapperedPhotonCameraFuel("color2", ROBOT_TO_COLOR_2)
        # self.fuel_map = []

        self.poseConverge = True

        self.last_periodic_accel_x = 0
        self.last_periodic_accel_y = 0

        self.temp_rotation_check = Rotation2d()

        self.tag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)
        
        self.active_intake_tgt = Translation2d()
        self.active_intake_tgt_score = 0

    def cur_pos_in_zone(self, alt_pos=4.4):
        cur_pos = self.curEstPose
        return (cur_pos.X() <= alt_pos and not self.robot.fieldConstants.shouldFlip) or (cur_pos.X() >= self.robot.fieldConstants.fieldLength - alt_pos and self.robot.fieldConstants.shouldFlip)

    def get_path_to_trench(self):
        if self.cur_pos_in_zone(): #from zone
            good_rotation = self.robot.fieldConstants.flip_Rotation2d(Rotation2d.fromDegrees(90))
            can_rotate = (self.curEstPose.X() <= 2.7 and not self.robot.fieldConstants.shouldFlip) or (self.curEstPose.X() >= self.robot.fieldConstants.fieldLength - 2.7 and self.robot.fieldConstants.shouldFlip) 
            if (self.curEstPose.Y() >= self.robot.fieldConstants.fieldWidth / 2):  #(blue origin) left side
                target_pose = Pose2d(self.robot.fieldConstants.flip_X_coord(4.621 - 1.25), 7.44, (good_rotation if can_rotate else self.curEstPose.rotation()))
            else: #(blue origin) right side
                target_pose = Pose2d(self.robot.fieldConstants.flip_X_coord(4.621 - 1.25), self.robot.fieldConstants.fieldWidth - 7.44, (good_rotation if can_rotate else self.curEstPose.rotation()))
        else: # from neutral
            can_rotate = (self.curEstPose.X() >= 6.421 and not self.robot.fieldConstants.shouldFlip) or (self.curEstPose.X() <= self.robot.fieldConstants.fieldLength - 6.421 and self.robot.fieldConstants.shouldFlip)
            if self.curEstPose.Y() >= self.robot.fieldConstants.fieldWidth / 2: # (blue origin) left side
                good_rotation = Rotation2d.fromDegrees(-90)
                target_pose = Pose2d(self.robot.fieldConstants.flip_X_coord(4.621 + 1.25), 7.44, (good_rotation if can_rotate else self.curEstPose.rotation()))
            else:
                good_rotation = Rotation2d.fromDegrees(90)
                target_pose = Pose2d(self.robot.fieldConstants.flip_X_coord(4.621 + 1.25), self.robot.fieldConstants.fieldWidth - 7.44, (good_rotation if can_rotate else self.curEstPose.rotation()))
        return target_pose
    
    def get_path_to_outpost(self):
        target_pose = self.robot.fieldConstants.flip_Pose2d(Pose2d(0.742, 0.649, Rotation2d.fromDegrees(-90)))
        return target_pose

    def update_fuel_intake_tgt(self):
        res = self.intake_cam.getBestPtIntake(self.curEstPose)
        if res is None:
            return None
        else:
            cur_best_pt = res[0]
            cur_best_score = res[1]
        
        if self.active_intake_tgt is None:
            self.active_intake_tgt = cur_best_pt
            self.active_intake_tgt_score = cur_best_score
            return self.active_intake_tgt
        
        still_exists = False
        for pos, timestamp in self.intake_cam.getFuelMemory():
            if pos.distance(self.active_intake_tgt) < 0.4:
                still_exists = True
                self.active_intake_tgt_score = self.intake_cam.calculate_single_score(pos, self.curEstPose)
                break
        
        if not still_exists: # all balls in 0.4 meers are poofed
            self.active_intake_tgt = cur_best_pt
            self.active_intake_tgt_score = cur_best_score
        elif cur_best_pt is not None:
            if cur_best_score > (self.active_intake_tgt_score * 4):
                self.active_intake_tgt = cur_best_pt
                self.active_intake_tgt_score = cur_best_score
        
        if self.active_intake_tgt and (self.curEstPose.translation().distance(self.active_intake_tgt)) < 0.2:
            self.active_intake_tgt = None
        
        return self.active_intake_tgt

    def stop(self):
        pass
        # print("sike this aint stoppin")

    def set_module_states(self, desired_states):
        desired_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
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
            self.poseEst.resetPose(Pose2d(self.curEstPose.X(), self.curEstPose.Y(), Rotation2d.fromDegrees(yaw)))
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
    
    def get_weight_by_accel(self): # GENERATED BY COPILOT, with some modifications
        # If running in simulation, return an empty weight profile immediately.
        if self.robot.isSimulation():
            return 0 # make empty (no fuel) weight profile

        # Reject if skidding or collision-like jerk detected.
        if (self.get_skidding_ratio() > const.SKIDDING_RATIO_MAX) or (self.get_jerk_val() > const.COLLISION_JERK_MAX):
            return 0 # make empty (no fuel) weight profile

        total = 0.0
        count = 0
        # small threshold to avoid division by tiny accelerations
        EPS = 1e-6
        for module in self.modules:
            dm = module.drive_motor
            accel = dm.get_acceleration().value
            if abs(accel) > EPS:
                total += dm.get_torque_current().value / accel
                count += 1

        if count > 0:
            return total / count
        else:
            return 0 # make empty (no fuel) weight profile
    
    def poseIsOffField(self, pose: Pose2d):
        trans = pose.translation()
        x = trans.X()
        y = trans.Y()
        inY = -0.5 < y < self.robot.fieldConstants.fieldWidth + 0.5
        inX = -0.5 < x < self.robot.fieldConstants.fieldLength + 0.5
        return not (inX and inY)

    def candidate_pose_OK(self, candidate_pose: Pose2d):
        if self.poseIsOffField(candidate_pose) and False:  # Check if the robot is on the field
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
        
    def set_wheels_to_x(self):
        fl = SwerveModuleState(0, Rotation2d.fromDegrees(45))
        fr = SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        bl = SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        br = SwerveModuleState(0, Rotation2d.fromDegrees(45))
        desired_states = (fl, fr, bl, br)
        self.set_module_states(desired_states)        

    def periodic(self):
        self.single_tag_IDs = set()
        single_tag_poses = []

        z_sum = 0
        z_count = 0
        for cam in self.cams:
            cam.update(
                self.curEstPose,
                cam.getObsTime()
            )
            single_tag_poses : list[(Pose2d, int)] = cam.getPoseSingleTag()
            self.single_tag_IDs.update(cam.getSingleTagIDs())
            zEstimates = cam.getZEstimates()
            z_sum += sum(zEstimates)
            z_count += len(zEstimates)
            valid_poses = []
           
            for combined in single_tag_poses:
                pose : Pose2d = combined[0]
                # tgt_id = combined[1]
                # ambiguity = combined[2]
                # # print(f"tgtZEst: {tgtZEst}")
                # # print(f"ambiguity : {ambiguity}")
                # tag_pose = self.tag_layout.getTagPose(tgt_id)
                # distance = tag_pose.translation().toTranslation2d().distance(pose.translation())
            
                # self.poseEst.addVisionMeasurement(
                #     pose,
                #     cam.getObsTime(),
                #     (
                #         self.xystd_single_tag * (distance ** 2),  # * (min_ambiguity / 0.4),
                #         self.xystd_single_tag * (distance ** 2),  # * (min_ambiguity / 0.4),
                #         self.thetastd_single_tag * (distance ** 2),  # * (min_ambiguity / 0.4),
                #     ),
                # )
                valid_poses.append(pose)
                
            if len(valid_poses) > 0:
                avg_x = sum(pose.X() for pose in valid_poses) / len(valid_poses)
                avg_y = sum(pose.Y() for pose in valid_poses) / len(valid_poses)

                avg_cos = sum(pose.rotation().cos() for pose in valid_poses) / len(valid_poses)
                avg_sin = sum(pose.rotation().sin() for pose in valid_poses) / len(valid_poses)

                avg_rot = Rotation2d(avg_cos, avg_sin)
                avg_pose = Pose2d(avg_x, avg_y, avg_rot)
                
                self.camera_X[cam.camName] = avg_x
                self.camera_Y[cam.camName] = avg_y
                self.camera_theta[cam.camName] = avg_pose.rotation()

                self.poseEst.addVisionMeasurement(
                    avg_pose,
                    cam.getObsTime(),
                    (
                        self.xystd_single_tag,  # * (min_ambiguity / 0.4),
                        self.xystd_single_tag,  # * (min_ambiguity / 0.4),
                        self.thetastd_single_tag,  # * (min_ambiguity / 0.4),
                    ),
                )
                    
        # if z_count > 0:
        #     self.estZ = z_sum / z_count
        
        # UPDATING OBJECT DETECTION CAMERAS
        # self.intake_cam.update(self.curEstPose, self.estZ, self.gyro.getRotation3d())
        # self.fuel_map = [pose for (pose, timestamp) in self.intake_cam.getFuelMemory()]
        # if self.robot.is_intaking:
        #     self.update_fuel_intake_tgt()




        # Update poses with drivetrain information
        self.poseEst.update(self.getYaw(), self.get_module_positions())

        possible_pose = self.poseEst.getEstimatedPosition()

        if not self.robot.isSimulation() and self.candidate_pose_OK(possible_pose):
            self.curEstPose = possible_pose

        self.poseConverge = True

        # self.odometry.update(self.getYaw(), self.get_module_positions())


    def log(self):
        for idx in range(len(self.cams)):
            try:
                SmartDashboard.putNumber(self.cams[idx].camName + " X", self.camera_X[self.cams[idx].camName])
                SmartDashboard.putNumber(self.cams[idx].camName + " Y", self.camera_Y[self.cams[idx].camName])
                SmartDashboard.putNumber(self.cams[idx].camName + " THETA", self.camera_theta[self.cams[idx].camName])
            except:
                continue

        SmartDashboard.putNumber("Gyro/gyro voltage", self.gyro.get_supply_voltage().value)

        SmartDashboard.putNumber("Camera/Odometry X", self.curEstPose.x)
        SmartDashboard.putNumber("Camera/Odometry Y", self.curEstPose.y)
        SmartDashboard.putNumber(
            "Camera/Odometry Theta", self.curEstPose.rotation().degrees()
        )
        SmartDashboard.putNumber("Est Z", self.estZ)

        # SmartDashboard.putNumber(
        #     "Swerve/Odometry X", self.odometry.getPose().x_feet * 0.305
        # )
        # SmartDashboard.putNumber(
        #     "Swerve/Odometry Y", self.odometry.getPose().y_feet * 0.305
        # )
        # SmartDashboard.putNumber(
        #     "Swerve/Odometry Theta", self.odometry.getPose().rotation().degrees()
        # )
        SmartDashboard.putNumber("Gyro/Yaw", self.getYaw().degrees())
        SmartDashboard.putNumber("Gyro/Roll", self.gyro.get_roll().value)

        SmartDashboard.putData("Field", self.field)
        self.field.setRobotPose(self.poseEst.getEstimatedPosition())
        final_lineup = self.field.getObject("target pose")
        final_lineup.setPose(self.robot.final_lineup_pose)

        SmartDashboard.putNumber("Skidding Ratio", self.get_skidding_ratio())
        for module in self.modules:
            SmartDashboard.putNumber(f"Swerve/{module.module_name}/Cancoder Angle", (module.get_angle_CANcoder().degrees() - module.angle_offset.degrees()) % 360)  # type: ignore
            SmartDashboard.putNumber(f"Swerve/{module.module_name}/Motor Angle", module.get_position().angle.degrees())  # type: ignore
            SmartDashboard.putNumber(
                f"Swerve/{module.module_name}/Velcoity", module.get_state().speed
            )
            SmartDashboard.putNumber(f"Swerve/{module.module_name}/Motor Position", module.get_position().distance)
        
        SmartDashboard.putNumber("Test/Weight by Acceleration", abs(self.get_weight_by_accel()))