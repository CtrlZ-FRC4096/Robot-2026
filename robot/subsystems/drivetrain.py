from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot


import time
import math
from phoenix6.hardware import Pigeon2, TalonFX
from wpilib import DriverStation, SmartDashboard, Timer, Field2d
from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Translation2d,
    Translation3d,
    Transform3d,
    Rotation3d,
    Twist2d,
    Transform2d
)
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState
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
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.config import PIDConstants

from pathplannerlib.path import PathPlannerTrajectory
from pathplannerlib.path import PathPlannerPath, PathConstraints, DriveFeedforwards
from wpimath.estimator import SwerveDrive4PoseEstimator
from photoncamera import WrapperedPhotonCameraTag
from wpimath.filter import SlewRateLimiter
from wpimath.units import degreesToRadians, inchesToMeters, radiansToDegrees
from collections import deque
from lookup_table import LookupTableAll, LookupTableAngle, LookupTableVel
import wpilib

# from shapely import Polygon, Point
# from shapely.affinity import translate, rotate

class Drivetrain(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot

        self.angle_pid = PIDController(0.0275, 0.0, 0.0001)
        self.angle_pid.enableContinuousInput(0, 360)
        self.angle_pid.setTolerance(2)  # Set position tolerance to 0.5 degrees

        self.angle_pid_close = PIDController(0.04, 0.0, 0.00015)
        self.angle_pid_close.enableContinuousInput(0, 360)
        self.angle_pid_close.setTolerance(2)  # Set position tolerance to 0.5 degrees

        self.angle_pid_far_sotm = PIDController(0.06, 0.0, 0.005)
        self.angle_pid_far_sotm.enableContinuousInput(0, 360)
        self.angle_pid_far_sotm.setTolerance(2)

        self.x_controller = PIDController(3, 0, 0) #0.01
        self.y_controller = PIDController(3, 0, 0) #0.01
        self.xy_controller = ProfiledPIDController(2.3, 0.0, 0.025, TrapezoidProfile.Constraints(4.0, 4.0))
        self.theta_controller = PIDController(0.07, 0.01, 0.0015)
        
        self.inter_max_vel = 4.0
        self.inter_max_acc = 4.0
        constraints = TrapezoidProfile.Constraints(self.inter_max_vel, self.inter_max_acc)
        # self.xy_controller = ProfiledPIDController(2.0, 0.01, 0.025)#, constraints, period=0.05)
        self.xy_inter_controller = ProfiledPIDController(1.9, 0.0, 0.0, constraints)


        self.previous_sim_speeds = ChassisSpeeds()
        self.previous_chassisspeeds = ChassisSpeeds()
        self.two_previous_sim_speeds = ChassisSpeeds()
        self.damping_accel = False

        self.dist_lookup_table = LookupTableAll()
        self.create_lookup_table()
        self.vel_lookup_table = LookupTableVel()
        self.create_launch_vel_table()
        self.angle_lookup_table = LookupTableAngle()
        self.create_launch_angle_table()


        ## Need to check these tolerances
        self.x_controller.setTolerance(0.5, 1) #0.025, 0.1
        self.y_controller.setTolerance(0.5, 1) #0.025, 0.1
        self.xy_controller.setTolerance(0.0225, 0.15)
        self.theta_controller.enableContinuousInput(0, 360)
        self.theta_controller.setTolerance(2.8, 2.0) #3.0, 0.1
        self.xy_inter_controller.setTolerance(0.15, 2.0)

        self.log_chassis = ChassisSpeeds()

        self.final_velo = Translation2d()

        self.cur_accel = Translation2d()
        self.accel_shoot_limiter = SlewRateLimiter(0.2, -3)

        # SIM STUFF

        blue_hub_pts = [
            (inchesToMeters(158.406), inchesToMeters(135.344)),
            (inchesToMeters(158.406), inchesToMeters(182.344)),
            (inchesToMeters(205.406), inchesToMeters(182.344)),
            (inchesToMeters(205.406), inchesToMeters(135.344))
        ]
        
        blue_trench_right_pts = [
            (inchesToMeters(158.406), inchesToMeters(50.344)),
            (inchesToMeters(158.406), inchesToMeters(62.344)),
            (inchesToMeters(205.406), inchesToMeters(62.344)),
            (inchesToMeters(205.406), inchesToMeters(50.344))
        ]

        blue_trench_left_pts = [
            (inchesToMeters(158.406), inchesToMeters(255.344)),
            (inchesToMeters(158.406), inchesToMeters(267.344)),
            (inchesToMeters(205.406), inchesToMeters(267.344)),
            (inchesToMeters(205.406), inchesToMeters(255.344))
        ]

        blue_tower_pts = [
            (inchesToMeters(38.358), inchesToMeters(129.759)),
            (inchesToMeters(44.858), inchesToMeters(129.759)),
            (inchesToMeters(44.858), inchesToMeters(165.178)),
            (inchesToMeters(38.358), inchesToMeters(165.178)),            
        ]

        red_hub_pts = [
            (inchesToMeters(445.406), inchesToMeters(135.344)),
            (inchesToMeters(445.406), inchesToMeters(182.344)),
            (inchesToMeters(492.406), inchesToMeters(182.344)),
            (inchesToMeters(492.406), inchesToMeters(135.344))
        ]

        red_trench_right_pts = [
            (inchesToMeters(445.406), inchesToMeters(50.344)),
            (inchesToMeters(445.406), inchesToMeters(62.344)),
            (inchesToMeters(492.406), inchesToMeters(62.344)),
            (inchesToMeters(492.406), inchesToMeters(50.344))
        ]

        red_trench_left_pts = [
            (inchesToMeters(445.406), inchesToMeters(255.344)),
            (inchesToMeters(445.406), inchesToMeters(267.344)),
            (inchesToMeters(492.406), inchesToMeters(267.344)),
            (inchesToMeters(492.406), inchesToMeters(255.344))
        ]

        red_tower_pts = [
            (inchesToMeters(605.955), inchesToMeters(129.759)),
            (inchesToMeters(612.455), inchesToMeters(129.759)),
            (inchesToMeters(612.455), inchesToMeters(165.178)),
            (inchesToMeters(605.955), inchesToMeters(165.178))
        ]

        field_boundary_points = [
            (0,0),
            (inchesToMeters(650.813), 0),
            (inchesToMeters(650.813), inchesToMeters(317.688)),
            (0, inchesToMeters(317.688))
        ]
        
        
    #     if self.robot.isSimulation():    
    #         field_boundary = Polygon(field_boundary_points)

    #         blue_hub = Polygon(blue_hub_pts)
    #         blue_tower = Polygon(blue_tower_pts)
    #         blue_trench_left = Polygon(blue_trench_left_pts)
    #         blue_trench_right = Polygon(blue_trench_right_pts)

    #         red_hub = Polygon(red_hub_pts)
    #         red_tower = Polygon(red_tower_pts)
    #         red_trench_left = Polygon(red_trench_left_pts)
    #         red_trench_right = Polygon(red_trench_right_pts)

    #         self.sim_obstacles = [
    #             (field_boundary, "within"),
    #             (blue_hub, "overlaps"),
    #             (blue_tower, "overlaps"),
    #             (blue_trench_left, "overlaps"),
    #             (blue_trench_right, "overlaps"),
    #             (red_hub, "overlaps"),
    #             (red_tower, "overlaps"),
    #             (red_trench_left, "overlaps"),
    #             (red_trench_right, "overlaps")
    #         ]
    # def get_robot_shape(self):
    #     cur_pose : Pose2d = self.robot.poseEstimator.curEstPose
    #     half_length = inchesToMeters(26 + 7.25) / 2.0
    #     half_width = inchesToMeters(28.5 + 7.25) / 2.0
    #     p1 = (-half_length, -half_width)
    #     p2 = (-half_length, half_width)
    #     p3  = (half_length, half_width)
    #     p4 = (half_length, -half_width)

    #     base_robot = Polygon([p1, p2, p3, p4])
    #     rotated_robot = rotate(base_robot, cur_pose.rotation().degrees())
    #     final_robot = translate(rotated_robot, xoff=cur_pose.X(), yoff=cur_pose.Y())
    #     return final_robot

    # def in_obstacle(self, pose : Translation2d):
    #     robot = self.get_robot_shape()
    #     for obstacle in self.sim_obstacles:
    #         match obstacle[1]:
    #             case "overlaps":
    #                 if obstacle[0].overlaps(robot):
    #                     return True
    #             case "within":
    #                 if not robot.within(obstacle[0]):
    #                     return True
    #     return False
    
    def drive(self, translation: Translation2d, rotation, field_relative, is_open_loop):
        SmartDashboard.putNumber("Swerve/Translation X", translation.x)
        SmartDashboard.putNumber("Swerve/Translation Y", translation.y)
        SmartDashboard.putNumber("Swerve/Rotation", rotation)
        SmartDashboard.putBoolean("Swerve/With PID", False)

        self.cur_accel = Translation2d(self.robot.poseEstimator.gyro.get_acceleration_x().value, self.robot.poseEstimator.gyro.get_acceleration_y().value)
        SmartDashboard.putNumber("Swerve/ Commanded* Acceleration", self.cur_accel.norm())
        
        if field_relative and not self.robot.isSimulation():
            module_states = const.SWERVE_KINEMATICS.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.x,
                    translation.y,
                    rotation,
                    self.robot.poseEstimator.getYaw()
                ))
        else:  # Robot relative
            module_states = const.SWERVE_KINEMATICS.toSwerveModuleStates(
                    ChassisSpeeds(translation.x,
                    translation.y,
                    rotation)
            )
        if self.robot.in_autonomous_mode:
            max_speed = 4.0
        else:
            max_speed = const.SWERVE_MAX_SPEED
        module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
                module_states, max_speed
            )
        self.log_chassis = const.SWERVE_KINEMATICS.toChassisSpeeds(module_states)
        # SmartDashboard.putNumber("translation x", translation.x / 20)
        # SmartDashboard.putNumber("translation y", translation.y / 20)
        # SmartDashboard.putNumber("translation omega", radiansToDegrees(rotation) / 20)

        # SmartDashboard.putNumber("chassis log vx", self.log_chassis.vx / 45)
        # SmartDashboard.putNumber("chassis log vy", self.log_chassis.vy / 45)
        # SmartDashboard.putNumber("chassis log omega dps", self.log_chassis.omega_dps / 20)
        if self.robot.isSimulation():
            curPose = self.robot.poseEstimator.curEstPose
            
            ## Acceleration limits
            final_vel = Pose2d(self.log_chassis.vx, self.log_chassis.vy, degreesToRadians(self.log_chassis.omega_dps))
            max_accel = 3.0


            current_vel = Translation2d(self.two_previous_sim_speeds.vx, self.two_previous_sim_speeds.vy)
            commanded_vel = Translation2d(self.log_chassis.vx, self.log_chassis.vy)

            if abs(commanded_vel.x) < 0.05 and abs(commanded_vel.y) < 0.05:
                final_vel = Pose2d(0, 0, final_vel.rotation())
            elif ((current_vel.distance(commanded_vel) / 0.1) > max_accel):
                self.damping_accel = True
                delta_vel = commanded_vel - current_vel
                vel_rad = Rotation2d(delta_vel.X(), delta_vel.Y()).radians()
                new_vel = Translation2d(current_vel.X() + max_accel * 0.77 * math.cos(vel_rad), current_vel.Y() + max_accel * 0.77 * math.sin(vel_rad))
                final_vel = Pose2d(new_vel, final_vel.rotation())
            else:
                self.damping_accel = False
            self.final_velo = final_vel.translation()

            self.robot.poseEstimator.curEstPose = Pose2d(curPose.X() + final_vel.X() / 30, curPose.Y() + final_vel.Y() / 30, Rotation2d.fromDegrees(curPose.rotation().degrees() + final_vel.rotation().degrees() / 20))
            if self.robot.poseEstimator.poseIsOffField(self.robot.poseEstimator.curEstPose):# or self.in_obstacle(self.robot.poseEstimator.curEstPose.translation()):
                self.robot.poseEstimator.curEstPose = curPose
            self.robot.poseEstimator.set_yaw(self.robot.poseEstimator.curEstPose.rotation().degrees())
            
            self.two_previous_sim_speeds = self.previous_sim_speeds
            self.previous_sim_speeds = ChassisSpeeds(final_vel.X(), final_vel.Y())

        else:
            for idx, module in enumerate(self.robot.poseEstimator.modules):
                SmartDashboard.putNumber("module state " + str(idx + 1), module_states[idx].speed)
                module.set_desired_state(module_states[idx], is_open_loop)


    def drive_with_pid(self, translation: Translation2d, target_angle):
        cur_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(self.log_chassis, self.robot.poseEstimator.curEstPose.rotation())
        
        if self.robot.shoot_intent and Translation2d(cur_speeds.vx, cur_speeds.vy).norm() > 0.2 and self.cur_accel.norm() > 0.28 and (self.robot.poseEstimator.curEstPose.rotation() - Rotation2d.fromDegrees(target_angle)).degrees() >= 30 and self.robot.shooter.shoot_ready and self.robot.shooter.accel_good:
            pid_output = self.angle_pid_far_sotm.calculate(self.robot.poseEstimator.curEstPose.rotation().degrees(), target_angle)
        else:
            pid_output = self.angle_pid.calculate(self.robot.poseEstimator.curEstPose.rotation().degrees(), target_angle)  
        # else:
        #     pid_output = self.angle_pid.calculate(self.robot.poseEstimator.curEstPose.rotation().degrees(), target_angle) 

        

        SmartDashboard.putBoolean("Swerve/With PID", True)
        self.drive(
            translation, pid_output, True, False
        )  # change is_open_loop back to False once done w/ driver tests
        # print(in_motion)

    def drive_robot_relative(
        self, chassis_speeds: ChassisSpeeds, feedfoward=None
    ):  # only use for pathplannerlib
        # PathPlanner returns robot-relative chassis speeds with +ω = CCW.
        # Our kinematics/modules expect the opposite sign, so flip it here
        module_states = const.SWERVE_KINEMATICS.toSwerveModuleStates(chassis_speeds)
        module_states = const.SWERVE_KINEMATICS.desaturateWheelSpeeds(module_states, 4)

        SmartDashboard.putNumber("pathplanner omega", chassis_speeds.omega_dps)
        SmartDashboard.putNumber("Pathplanner curPose yaw", self.robot.poseEstimator.curEstPose.rotation().degrees())

        mag_vel_dummy = Translation2d(chassis_speeds.vx, chassis_speeds.vy).norm()
        dummy_val = self.accel_shoot_limiter.calculate(mag_vel_dummy)

        if self.robot.isSimulation():
            new_chassis_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassis_speeds.vx, chassis_speeds.vy, chassis_speeds.omega, self.robot.poseEstimator.curEstPose.rotation())
            curPose = self.robot.poseEstimator.curEstPose
            self.robot.poseEstimator.curEstPose = Pose2d(
                curPose.X() + new_chassis_speeds.vx / 15, curPose.Y() + new_chassis_speeds.vy / 15, Rotation2d(curPose.rotation().radians() + new_chassis_speeds.omega / 10)
            )
        else:
            for idx, module in enumerate(self.robot.poseEstimator.modules):
                # amps = feedfoward.torqueCurrentsAmps[idx]
                module.set_desired_state(module_states[idx], is_open_loop=False, feed_forward=0.0)
    
    def should_flip_path(self):
        return self.robot.fieldConstants.shouldFlip
    
    def stop_intaking(self):
        self.robot.is_intaking = False
        print("HIHIH\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")

    def go_to_pose_profiled_pid(self, target_pose : Translation2d, feedforward_x=0.0, feedforward_y=0.0, feedfoward_theta=0.0):

        current_pose = self.robot.poseEstimator.curEstPose

        # Calculate the control outputs
        vx = self.x_controller.calculate(current_pose.X(), target_pose.X()) + feedforward_x # meters / 0.05 seconds
        vy = self.y_controller.calculate(current_pose.Y(), target_pose.Y()) + feedforward_y

        # if FieldConstants.shouldFlip:
        #     vx = -vx
        #     vy = -vy
        omega = self.theta_controller.calculate(
            current_pose.rotation().degrees(), target_pose.rotation().degrees()
        ) + feedfoward_theta


        if self.robot.velocity_constrain_pid:
            mag_vel = Translation2d(vx, vy).norm()
            if mag_vel > 1e-3:
                direction = Translation2d(vx, vy) / mag_vel
            else:
                direction = Translation2d(0, 0)

            if mag_vel >= 0.25:
                vx = (vx / mag_vel) * 0.25
                vy = (vy / mag_vel) * 0.25
            new_mag_vel = Translation2d(vx, vy).norm()

            limit_mag = self.accel_shoot_limiter.calculate(new_mag_vel)
            vx = direction.X() * limit_mag
            vy = direction.Y() * limit_mag
        # if self.robot.shoot_intent:
        #     mag_vel = Translation2d(vx, vy).norm()
        #     if mag_vel > 1e-6:
        #         direction = Translation2d(vx, vy) / mag_vel
        #     else:
        #         direction = Translation2d(0, 0) 
        #     limit_mag = self.accel_shoot_limiter.calculate(mag_vel)
        #     vx = direction.X() * limit_mag
        #     vy = direction.Y() * limit_mag

        # Check if the controllers are at their setpoints

        if (
            self.x_controller.atSetpoint()
            and self.y_controller.atSetpoint()
            #and (self.theta_controller.atSetpoint() or self.robot.in_autonomous_mode)
        ):
            # self.robot.running_pid_lineup = False
            # if self.robot.lining_with_outpost:
            #     self.robot.clear_jam = True
            #     self.robot.ignore_shooter_in_jam = True
            #     self.robot.is_intaking = False
            #     self.robot.intake_at_default = False
            # if self.robot.lining_with_trench:
            #     self.robot.at_trench_position = True
            # self.robot.running_pid_lineup = False
            #and (self.theta_controller.atSetpoint() and not self.robot.shoot_intent)
        
            # self.robot.running_pid_lineup = False
            if self.robot.run_p1:
                self.robot.done_p1 = True
            if self.robot.run_p2:
                self.robot.done_p2 = True
            if self.robot.run_p3:
                self.robot.done_p3 = True
            if self.robot.run_p4:
                self.robot.done_p4 = True

            
            
            # self.robot.running_pid_lineup = False


        # Drive the robot using the calculated velocities
        self.drive(Translation2d(vx, vy), omega, True, False)

        # Update SmartDashboard values for debugging
        SmartDashboard.putNumber("t_pose x", target_pose.X())
        SmartDashboard.putNumber("t_pose y", target_pose.Y())
        SmartDashboard.putNumber("vx", vx)
        SmartDashboard.putNumber("vy", vy)
        SmartDashboard.putNumber("omega", omega)

    def stop(self):
        self.drive(Translation2d(0, 0), 0, False, True)

    def get_timestamp(self):
        return wpilib.RobotController.getFPGATime() / 1000000

    def get_pose(self):
        return self.robot.poseEstimator.curEstPose
    
    def get_field_relative_speeds(self):
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            self.get_robot_relative_speeds(),
            self.robot.poseEstimator.curEstPose.rotation()
        )

    def reset_odometry(self, pose):
        self.robot.poseEstimator.odometry.resetPosition(self.robot.poseEstimator.getYaw(), [*self.robot.poseEstimator.get_module_positions()], pose)  # type: ignore
        self.robot.poseEstimator.poseEst.resetPosition(
            self.robot.poseEstimator.getYaw(),
            [*self.robot.poseEstimator.get_module_positions()],
            pose,
        )

    def reset_pid_error(self):
        self.x_controller.reset()
        self.y_controller.reset()
        self.theta_controller.reset()
        
    def get_robot_relative_speeds(self):

        module_states = (
            self.robot.poseEstimator.get_module_states()
        )  # Check this in swervemodule.py, we need to convert kraken speed to m/s
        chassis_speeds = const.SWERVE_KINEMATICS.toChassisSpeeds(module_states)  # type: ignore
        return self.previous_sim_speeds if self.robot.isSimulation() else chassis_speeds

    def shouldFlipPath(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
    
    def get_target_angle(self, tof, target : Translation2d):
        field_relative_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(self.log_chassis, self.robot.poseEstimator.curEstPose.rotation()) #self.get_field_relative_speeds()

        virtual_goal_x = target.x - (tof + 0.5) * (field_relative_speeds.vx)
        virtual_goal_y = target.y - (tof + 0.5) * (field_relative_speeds.vy)

        moving_goal_location = Translation2d(virtual_goal_x, virtual_goal_y)
        robot_to_target = (moving_goal_location - self.robot.poseEstimator.curEstPose.translation())
        self.robot.virtual_target.setPose(Pose2d(virtual_goal_x, virtual_goal_y, 0))

        return Rotation2d.fromDegrees(Rotation2d((-1 * robot_to_target.X()), (-1 * robot_to_target.Y())).degrees() - 90)
    
    def get_hub_angle(self, tof):
        target_goal = self.robot.fieldConstants.flip_Translation2d(self.robot.fieldConstants.Hub.topCenterPoint.toTranslation2d())
        return self.get_target_angle(tof, target_goal)

    def get_hub_distance(self, pos : Translation2d):
        hub_pos = self.robot.fieldConstants.flip_Translation2d(self.robot.fieldConstants.Hub.innerCenterPoint.toTranslation2d())
        return (pos - hub_pos).norm()

    def _get_final_lineup_pose(self, pose : Pose2d):
        if not self.robot.shoot_intent:
            return pose
        else:
            return Pose2d(pose.translation(), self.get_hub_angle())
        
    def create_lookup_table(self):
        # self.dist_lookup_table.add_entry(0.7, 5.404, 79.295, 0.655)
        # self.dist_lookup_table.add_entry(0.774, 5.443, 78.307, 0.665)
        # self.dist_lookup_table.add_entry(0.847, 5.484, 77.357, 0.675)
        # self.dist_lookup_table.add_entry(0.921, 5.526, 76.442, 0.684)
        # self.dist_lookup_table.add_entry(0.995, 5.569, 75.562, 0.693)
        # self.dist_lookup_table.add_entry(1.069, 5.614, 74.716, 0.702)
        # self.dist_lookup_table.add_entry(1.142, 5.66, 73.902, 0.711)
        # self.dist_lookup_table.add_entry(1.216, 5.707, 73.119, 0.72)
        # self.dist_lookup_table.add_entry(1.29, 5.754, 72.366, 0.729)
        # self.dist_lookup_table.add_entry(1.364, 5.803, 71.641, 0.738)
        # self.dist_lookup_table.add_entry(1.437, 5.852, 70.944, 0.746)
        # self.dist_lookup_table.add_entry(1.511, 5.902, 70.273, 0.755)
        # self.dist_lookup_table.add_entry(1.585, 5.953, 69.627, 0.764)
        # self.dist_lookup_table.add_entry(1.659, 6.004, 69.006, 0.772)
        # self.dist_lookup_table.add_entry(1.732, 6.056, 68.407, 0.781)
        # self.dist_lookup_table.add_entry(1.806, 6.108, 67.831, 0.789)
        # self.dist_lookup_table.add_entry(1.88, 6.16, 67.275, 0.797)
        # self.dist_lookup_table.add_entry(1.954, 6.213, 66.74, 0.806)
        # self.dist_lookup_table.add_entry(2.027, 6.266, 66.223, 0.814)
        # self.dist_lookup_table.add_entry(2.101, 6.319, 65.725, 0.822)
        # self.dist_lookup_table.add_entry(2.175, 6.373, 65.245, 0.83)
        # self.dist_lookup_table.add_entry(2.248, 6.426, 64.781, 0.838)
        # self.dist_lookup_table.add_entry(2.322, 6.48, 64.334, 0.846)
        # self.dist_lookup_table.add_entry(2.396, 6.534, 63.902, 0.854)
        # self.dist_lookup_table.add_entry(2.47, 6.588, 63.484, 0.862)
        # self.dist_lookup_table.add_entry(2.543, 6.642, 63.08, 0.87)
        # self.dist_lookup_table.add_entry(2.617, 6.696, 62.69, 0.878)
        # self.dist_lookup_table.add_entry(2.691, 6.75, 62.312, 0.885)
        # self.dist_lookup_table.add_entry(2.765, 6.804, 61.947, 0.893)
        # self.dist_lookup_table.add_entry(2.838, 6.859, 61.593, 0.901)
        # self.dist_lookup_table.add_entry(2.912, 6.913, 61.25, 0.909)
        # self.dist_lookup_table.add_entry(2.986, 6.97, 61.0, 0.918)
        # self.dist_lookup_table.add_entry(3.06, 7.035, 61.0, 0.934)
        # self.dist_lookup_table.add_entry(3.133, 7.101, 61.0, 0.949)
        # self.dist_lookup_table.add_entry(3.207, 7.167, 61.0, 0.965)
        # self.dist_lookup_table.add_entry(3.281, 7.232, 61.0, 0.98)
        # self.dist_lookup_table.add_entry(3.355, 7.298, 61.0, 0.994)
        # self.dist_lookup_table.add_entry(3.428, 7.364, 61.0, 1.009)
        # self.dist_lookup_table.add_entry(3.502, 7.43, 61.0, 1.024)
        # self.dist_lookup_table.add_entry(3.576, 7.496, 61.0, 1.038)
        # self.dist_lookup_table.add_entry(3.649, 7.561, 61.0, 1.052)
        # self.dist_lookup_table.add_entry(3.723, 7.627, 61.0, 1.066)
        # self.dist_lookup_table.add_entry(3.797, 7.692, 61.0, 1.08)
        # self.dist_lookup_table.add_entry(3.871, 7.757, 61.0, 1.094)
        # self.dist_lookup_table.add_entry(3.944, 7.823, 61.0, 1.107)
        # self.dist_lookup_table.add_entry(4.018, 7.888, 61.0, 1.121)
        # self.dist_lookup_table.add_entry(4.092, 7.952, 61.0, 1.134)
        # self.dist_lookup_table.add_entry(4.166, 8.017, 61.0, 1.147)
        # self.dist_lookup_table.add_entry(4.239, 8.082, 61.0, 1.16)
        # self.dist_lookup_table.add_entry(4.313, 8.146, 61.0, 1.173)
        # self.dist_lookup_table.add_entry(4.387, 8.211, 61.0, 1.186)
        # self.dist_lookup_table.add_entry(4.461, 8.275, 61.0, 1.198)
        # self.dist_lookup_table.add_entry(4.534, 8.339, 61.0, 1.211)
        # self.dist_lookup_table.add_entry(4.608, 8.403, 61.0, 1.224)
        # self.dist_lookup_table.add_entry(4.682, 8.466, 61.0, 1.236)
        # self.dist_lookup_table.add_entry(4.756, 8.53, 61.0, 1.248)
        # self.dist_lookup_table.add_entry(4.829, 8.593, 61.0, 1.26)
        # self.dist_lookup_table.add_entry(4.903, 8.656, 61.0, 1.272)
        # self.dist_lookup_table.add_entry(4.977, 8.72, 61.0, 1.284)
        # self.dist_lookup_table.add_entry(5.051, 8.782, 61.0, 1.296)
        # self.dist_lookup_table.add_entry(5.124, 8.845, 61.0, 1.308)
        # self.dist_lookup_table.add_entry(5.198, 8.908, 61.0, 1.32)
        # self.dist_lookup_table.add_entry(5.272, 8.97, 61.0, 1.332)
        # self.dist_lookup_table.add_entry(5.345, 9.033, 61.0, 1.343)
        # self.dist_lookup_table.add_entry(5.419, 9.095, 61.0, 1.355)
        # self.dist_lookup_table.add_entry(5.493, 9.157, 61.0, 1.366)
        # self.dist_lookup_table.add_entry(5.567, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(5.64, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(5.714, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(5.788, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(5.862, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(5.935, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(6.009, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(6.083, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(6.157, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(6.23, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(6.304, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(6.378, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(6.452, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(6.525, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(6.599, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(6.673, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(6.746, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(6.82, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(6.894, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(6.968, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(7.041, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(7.115, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(7.189, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(7.263, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(7.336, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(7.41, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(7.484, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(7.558, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(7.631, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(7.705, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(7.779, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(7.853, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(7.926, 9.2, 61.0, 1.374)
        # self.dist_lookup_table.add_entry(8.0, 9.2, 61.0, 1.374)
        
        #Modified version for -0.5 v_std only
        self.dist_lookup_table.add_entry(0.7, 5.465, 75.048, 0.638)
        self.dist_lookup_table.add_entry(1.084, 5.735, 69.912, 0.689)
        self.dist_lookup_table.add_entry(1.468, 6.029, 65.963, 0.735)
        self.dist_lookup_table.add_entry(1.853, 6.332, 62.87, 0.78)
        self.dist_lookup_table.add_entry(2.237, 6.641, 61.0, 0.835) # calc tof: 1.169
        self.dist_lookup_table.add_entry(2.621, 6.977, 61.0, 0.92)
        self.dist_lookup_table.add_entry(3.005, 7.323, 61.0, 1.0)
        self.dist_lookup_table.add_entry(3.389, 7.672, 61.0, 1.076)
        self.dist_lookup_table.add_entry(3.774, 8.018, 61.0, 1.184) # calc tof: 1.27
        self.dist_lookup_table.add_entry(4.158, 8.36, 61.0, 1.215)
        self.dist_lookup_table.add_entry(4.542, 8.697, 61.0, 1.28)
        self.dist_lookup_table.add_entry(4.926, 9.03, 61.0, 1.343)
        self.dist_lookup_table.add_entry(5.311, 9.2, 61.0, 1.374)
        self.dist_lookup_table.add_entry(5.695, 9.2, 61.0, 1.374)
        self.dist_lookup_table.add_entry(6.079, 9.2, 61.0, 1.374)
        self.dist_lookup_table.add_entry(6.463, 9.2, 61.0, 1.374)
        self.dist_lookup_table.add_entry(6.847, 9.2, 61.0, 1.374)
        self.dist_lookup_table.add_entry(7.232, 9.2, 61.0, 1.374)
        self.dist_lookup_table.add_entry(7.616, 9.2, 61.0, 1.374)
        self.dist_lookup_table.add_entry(8.0, 9.2, 61.0, 1.374)
        
    def create_launch_vel_table(self):
        self.vel_lookup_table.add_entry(6.05, 50)
        self.vel_lookup_table.add_entry(7.3, 60)
        self.vel_lookup_table.add_entry(7.95, 67)
        self.vel_lookup_table.add_entry(8.3, 70)
        self.vel_lookup_table.add_entry(8.9, 80)
        self.vel_lookup_table.add_entry(9.2, 83)
        
    def create_launch_angle_table(self):
        self.angle_lookup_table.add_entry(61, 45)
        self.angle_lookup_table.add_entry(65, 40)
        self.angle_lookup_table.add_entry(71, 30)
        self.angle_lookup_table.add_entry(73, 20)
        self.angle_lookup_table.add_entry(80, 10)
        self.angle_lookup_table.add_entry(85, 0)


    def periodic(self):
        # # 6328 INSPIRED SHOT CALC
        # cur_robot_pose = self.robot.poseEstimator.curEstPose
        # robot_relative_vel = self.get_robot_relative_speeds()
        # est_pose = cur_robot_pose.exp(Twist2d(
        #     robot_relative_vel.vx * 0.03,
        #     robot_relative_vel.vy * 0.03,
        #     robot_relative_vel.omega * 0.03
        # ))

        # if self.robot.shoot_intent:
        #     if self.robot.poseEstimator.cur_pos_in_zone(4.55) or self.robot.in_autonomous_mode:
        #         self.robot.static_target = self.robot.fieldConstants.flip_Translation2d(self.robot.fieldConstants.Hub.topCenterPoint.toTranslation2d())
        #     else:
        #         if not self.robot.fieldConstants.shouldFlip:
        #             if cur_pos.Y() <= self.robot.fieldConstants.fieldWidth / 2: #shoot to right corner blue
        #                 self.robot.static_target = Translation2d(1.694, 1.417)
        #             else: #pass to left corner blue
        #                 self.robot.static_target = Translation2d(1.694, self.robot.fieldConstants.fieldWidth - 1.417)
        #         else:
        #             if cur_pos.Y() <= self.robot.fieldConstants.fieldWidth / 2: # pass to left corner red (red relative)
        #                 self.robot.static_target = Translation2d(1.694, self.robot.fieldConstants.fieldWidth - 1.417)
        #             else: #pass to right corner 
        #                 self.robot.static_target = Translation2d(1.694, 1.417)
            
        #     shooter_pos = est_pose.transformBy(Transform2d())

        start_time = wpilib.RobotController.getFPGATime()


        cur_speeds = self.log_chassis #self.get_robot_relative_speeds()
        phase_delay = 0.04
        phase_delay_twist = Twist2d(cur_speeds.vx * phase_delay + 0.5 * self.cur_accel.X() * (phase_delay ** 2), cur_speeds.vy * phase_delay + 0.5 * self.cur_accel.Y() * (phase_delay ** 2), cur_speeds.omega * phase_delay)
        cur_pos = self.robot.poseEstimator.curEstPose.exp(phase_delay_twist)
        self.robot.distance = self.get_hub_distance(cur_pos.translation())
        if self.robot.shoot_intent:
            cur_rot = cur_pos.rotation().radians()
            shooter_pos = cur_pos.translation() + Translation2d(0, 0.196).rotateBy(Rotation2d(cur_rot))
            if self.robot.poseEstimator.cur_pos_in_zone(4.55) or self.robot.in_autonomous_mode:
                self.robot.static_target = self.robot.fieldConstants.flip_Translation2d(self.robot.fieldConstants.Hub.topCenterPoint.toTranslation2d())
            elif 4.43 < cur_pos.X() < self.robot.fieldConstants.fieldLength - 4.4 or True:
                if not self.robot.fieldConstants.shouldFlip:
                    if cur_pos.Y() <= self.robot.fieldConstants.fieldWidth / 2: #shoot to right corner blue
                        self.robot.static_target = Translation2d(1.694, 1.417)
                    else: #pass to left corner blue
                        self.robot.static_target = Translation2d(1.694, self.robot.fieldConstants.fieldWidth - 1.417)
                else:
                    if cur_pos.Y() >= self.robot.fieldConstants.fieldWidth / 2: # pass to left corner red (red relative)
                        self.robot.static_target = Translation2d(self.robot.fieldConstants.fieldLength - 1.694, self.robot.fieldConstants.fieldWidth - 1.417)
                    else: #pass to right corner 
                        self.robot.static_target = Translation2d(self.robot.fieldConstants.fieldLength - 1.694, 1.417)
            
            dist_from_shooter = shooter_pos.distance(self.robot.static_target)           
            
            temp_time_of_flight = self.dist_lookup_table.interpolate(dist_from_shooter)[2]
            temp_virtual_goal = Translation2d()
            field_relative_speeds = self.get_field_relative_speeds()
            # SmartDashboard.putNumber("Test/ Actual Field Rel X", field_relative_speeds.vx)
            # SmartDashboard.putNumber("Test/ Actual Field Rel Y", field_relative_speeds.vy)
            for _ in range(3):
                if True: # CHANGE TO CASES ON ALLIANCE ZONE AND NEUTRAL ZONE
                    temp_virtual_goal = Translation2d(
                        self.robot.static_target.X() - (temp_time_of_flight + 0.2) * field_relative_speeds.vx, self.robot.static_target.Y() - (temp_time_of_flight + 0.2) * field_relative_speeds.vy 
                    )
                virtual_robot_distance = (shooter_pos - temp_virtual_goal).norm()
                temp_time_of_flight = self.dist_lookup_table.interpolate(virtual_robot_distance)[2]

            SmartDashboard.putNumber("Virtual Goal Dist (shooter)", virtual_robot_distance)
            # self.robot.virtual_target.setPose(Pose2d(temp_virtual_goal, Rotation2d()))
            self.robot.virtual_goal = temp_virtual_goal
            self.robot.time_of_flight = temp_time_of_flight

            vals = self.dist_lookup_table.interpolate(virtual_robot_distance)

            self.robot.fly_speed = self.vel_lookup_table.interpolate(vals[0])
            self.robot.hood_angle = self.angle_lookup_table.interpolate(vals[1])
            if self.robot.fly_speed >= 85:
                self.robot.fly_speed = 85

            if self.robot.hood_angle >= 45:
                self.robot.hood_angle = 45
            elif self.robot.hood_angle <= 0:
                self.robot.hood_angle = 0
        
        
        
        # self.chassis_accel = (
        #     self.get_robot_relative_speeds() - self.previous_chassisspeeds
        # ) / 0.05
        self.previous_chassisspeeds = self.get_robot_relative_speeds()

        if self.robot.in_autonomous_mode:
            # if self.robot.poseEstimator.curEstPose.X() >= 5.172:
            #     # self.robot.is_intaking = True
            #     # self.robot.intake_at_default = False
            #     pass
            # if self.robot.fuel_in_hopper >= 9 and self:
            #     pass

            # if self.robot.running_pid_lineup:
            #     if self.robot.shoot_intent and self.robot.poseEstimator.cur_pos_in_zone(4.5):
            #             rotation = self.robot.drivetrain.get_hub_angle(self.robot.time_of_flight)
            #             lineup  = Pose2d(self.robot.final_lineup_pose.X(), self.robot.final_lineup_pose.Y(), rotation)
            #             # SmartDashboard.putNumber("Shooter/Rotation to Hub", rotation.degrees())
            #     else:
            #         lineup = self.robot.final_lineup_pose
            #     self.go_to_pose_profiled_pid(lineup)
            # elif not self.robot.running_pid_lineup and self.robot.shoot_intent:
            #     rotation = self.get_hub_angle(self.robot.time_of_flight)
            #     self.drive_with_pid(Translation2d(0, 0), rotation.degrees())
            # elif self.robot.should_rotate_trench_auto:
            #     self.drive_with_pid(Translation2d(0, 0), self.robot.auto_rotation_trench)
            # # self.go_to_pose_profiled_pid(self.robot.final_lineup_pose)
            pass

        elapsed_ms = (wpilib.RobotController.getFPGATime() - start_time) / 1000
        SmartDashboard.putNumber("Loop Times/Drivetrain", elapsed_ms)

    def log(self):
        SmartDashboard.putData("PID Controller Reef XY", self.xy_controller)
        SmartDashboard.putData("PID Controller for going to reef, x", self.x_controller)
        SmartDashboard.putData("PID Controller for going to reef, y", self.y_controller)
        SmartDashboard.putData(
            "PID Controller for going to reef, theta", self.theta_controller
        )
        SmartDashboard.putNumber("Distance to Hub", self.get_hub_distance(self.robot.poseEstimator.curEstPose.translation()))

        SmartDashboard.putData("PID Controller (Drivetrain)", self.angle_pid)
        SmartDashboard.putData("PID Controller CLOSE (Drivetrain)", self.angle_pid_close)
        SmartDashboard.putBoolean("Angle at Setpoint", self.angle_pid.atSetpoint())
        SmartDashboard.putNumber("PID Controller Error", self.angle_pid.getError())
        SmartDashboard.putData("PID Controller (XY Inter)", self.xy_inter_controller)