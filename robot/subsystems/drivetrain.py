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
    Twist2d
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

# from shapely import Polygon, Point
# from shapely.affinity import translate, rotate

class Drivetrain(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot

        self.angle_pid = PIDController(0.046, 0.0, 0.001)
        self.angle_pid.enableContinuousInput(0, 360)
        self.angle_pid.setTolerance(0.5)  # Set position tolerance to 0.5 degrees

        self.x_controller = PIDController(1.75, 0, 0.1) #0.01
        self.y_controller = PIDController(1.75, 0, 0.1) #0.01
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
        self.x_controller.setTolerance(0.25, 0.1) #0.025, 0.1
        self.y_controller.setTolerance(0.25, 0.1) #0.025, 0.1
        self.xy_controller.setTolerance(0.0225, 0.15)
        self.theta_controller.enableContinuousInput(0, 360)
        self.theta_controller.setTolerance(2.8, 2.0) #3.0, 0.1
        self.xy_inter_controller.setTolerance(0.15, 2.0)

        self.log_chassis = ChassisSpeeds()

        self.final_velo = Translation2d()
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
        
        if field_relative and not self.robot.isSimulation():
            module_states = const.SWERVE_KINEMATICS.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.x,
                    translation.y,
                    rotation,
                    self.robot.poseEstimator.getYaw(),
                )
            )
        else:  # Robot relative
            module_states = const.SWERVE_KINEMATICS.toSwerveModuleStates(
                ChassisSpeeds(
                    translation.x,
                    translation.y,
                    rotation,
                )
            )
        if self.robot.in_autonomous_mode:
            max_speed = 4.0
        else:
            max_speed = const.SWERVE_MAX_SPEED
        module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
                module_states, max_speed
            )
        self.log_chassis = const.SWERVE_KINEMATICS.toChassisSpeeds(module_states)
        SmartDashboard.putNumber("translation x", translation.x / 20)
        SmartDashboard.putNumber("translation y", translation.y / 20)
        SmartDashboard.putNumber("translation omega", radiansToDegrees(rotation) / 20)

        SmartDashboard.putNumber("chassis log vx", self.log_chassis.vx / 45)
        SmartDashboard.putNumber("chassis log vy", self.log_chassis.vy / 45)
        SmartDashboard.putNumber("chassis log omega dps", self.log_chassis.omega_dps / 20)
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
        pid_output = self.angle_pid.calculate(self.robot.poseEstimator.curEstPose.rotation().degrees(), target_angle)  # type: ignore

        if self.angle_pid.atSetpoint():
            pid_output = 0

        # if not in_motion:
        #     pid_output += math.copysign(0.2, pid_output)
        SmartDashboard.putBoolean("Swerve/With PID", True)
        self.drive(
            translation, pid_output, True, False
        )  # change is_open_loop back to False once done w/ driver tests
        # print(in_motion)

    def drive_robot_relative(
        self, chassis_speeds: ChassisSpeeds, feedfoward : DriveFeedforwards
    ):  # only use for pathplannerlib
        # PathPlanner returns robot-relative chassis speeds with +ω = CCW.
        # Our kinematics/modules expect the opposite sign, so flip it here.
        module_states = const.SWERVE_KINEMATICS.toSwerveModuleStates(chassis_speeds)
        module_states = const.SWERVE_KINEMATICS.desaturateWheelSpeeds(module_states, 4)

        SmartDashboard.putNumber("pathplanner omega", chassis_speeds.omega_dps)
        SmartDashboard.putNumber("pose yaw", self.robot.poseEstimator.curEstPose.rotation().degrees())

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
                amps = feedfoward.torqueCurrentsAmps[idx]
                module.set_desired_state(module_states[idx], is_open_loop=False, feed_forward=amps)
    
    def should_flip_path(self):
        return self.robot.fieldConstants.shouldFlip

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
            # and self.theta_controller.atSetpoint()
        ):
            # self.robot.running_pid_lineup = False
            self.robot.running_pid_lineup = False


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
        field_relative_speeds = self.get_field_relative_speeds()

        virtual_goal_x = target.x - tof * (field_relative_speeds.vx)
        virtual_goal_y = target.y - tof * (field_relative_speeds.vy)

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
        # self.dist_lookup_table.add_entry(0.952, 5.543, 76.074, 0.688)
        # self.dist_lookup_table.add_entry(1.203, 5.698, 73.252, 0.719)
        # self.dist_lookup_table.add_entry(1.455, 5.864, 70.78, 0.749)
        # self.dist_lookup_table.add_entry(1.707, 6.038, 68.611, 0.778)
        # self.dist_lookup_table.add_entry(1.959, 6.217, 66.703, 0.806)
        # self.dist_lookup_table.add_entry(2.21, 6.399, 65.019, 0.834)
        # self.dist_lookup_table.add_entry(2.462, 6.647, 65.0, 0.9)
        # self.dist_lookup_table.add_entry(2.714, 6.902, 65.0, 0.963)
        # self.dist_lookup_table.add_entry(2.966, 7.158, 65.0, 1.022)
        # self.dist_lookup_table.add_entry(3.217, 7.413, 65.0, 1.079)
        # self.dist_lookup_table.add_entry(3.469, 7.665, 65.0, 1.134)
        # self.dist_lookup_table.add_entry(3.721, 7.916, 65.0, 1.186)
        # self.dist_lookup_table.add_entry(3.972, 8.163, 65.0, 1.236)
        # self.dist_lookup_table.add_entry(4.224, 8.409, 65.0, 1.285)
        # self.dist_lookup_table.add_entry(4.476, 8.651, 65.0, 1.333)
        # self.dist_lookup_table.add_entry(4.728, 8.892, 65.0, 1.379)
        # self.dist_lookup_table.add_entry(4.979, 9.13, 65.0, 1.424)
        # self.dist_lookup_table.add_entry(5.231, 9.2, 65.0, 1.437)
        # self.dist_lookup_table.add_entry(5.483, 9.2, 65.0, 1.437)
        # self.dist_lookup_table.add_entry(5.734, 9.2, 65.0, 1.437)
        # self.dist_lookup_table.add_entry(5.986, 9.2, 65.0, 1.437)
        # self.dist_lookup_table.add_entry(6.238, 9.2, 65.0, 1.437)
        # self.dist_lookup_table.add_entry(6.49, 9.2, 65.0, 1.437)
        # self.dist_lookup_table.add_entry(6.741, 9.2, 65.0, 1.437)
        # self.dist_lookup_table.add_entry(6.993, 9.2, 65.0, 1.437)
        # self.dist_lookup_table.add_entry(7.245, 9.2, 65.0, 1.437)
        # self.dist_lookup_table.add_entry(7.497, 9.2, 65.0, 1.437)
        # self.dist_lookup_table.add_entry(7.748, 9.2, 65.0, 1.437)
        # self.dist_lookup_table.add_entry(8.0, 9.2, 65.0, 1.437)

        # Safe Lookup Table
                # Good table
        self.dist_lookup_table.add_entry(0.7, 5.404, 79.295, 0.655)
        self.dist_lookup_table.add_entry(0.952, 5.543, 76.074, 0.688)
        self.dist_lookup_table.add_entry(1.203, 5.698, 73.252, 0.719)
        self.dist_lookup_table.add_entry(1.455, 5.864, 70.78, 0.749)
        self.dist_lookup_table.add_entry(1.707, 6.038, 68.611, 0.778)
        self.dist_lookup_table.add_entry(1.959, 6.217, 66.703, 0.806)
        self.dist_lookup_table.add_entry(2.21, 6.399, 65.019, 0.834)
        self.dist_lookup_table.add_entry(2.462, 6.647, 65.0, 0.9)
        self.dist_lookup_table.add_entry(2.714, 6.902, 65.0, 0.963)

    def create_launch_vel_table(self):
        self.vel_lookup_table.add_entry(5.6, 50)
        self.vel_lookup_table.add_entry(8.2, 70)
        self.vel_lookup_table.add_entry(8.85, 80)
        
    def create_launch_angle_table(self):
        self.angle_lookup_table.add_entry(65, 40)
        self.angle_lookup_table.add_entry(71, 30)
        self.angle_lookup_table.add_entry(73, 20)
        self.angle_lookup_table.add_entry(80, 10)
        self.angle_lookup_table.add_entry(85, 0)

    def periodic(self):
        cur_speeds = self.get_robot_relative_speeds()
        phase_delay_twist = Twist2d(cur_speeds.vx * 0.1, cur_speeds.vy * 0.1, cur_speeds.omega * 0.1)
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
                    if cur_pos.Y() <= self.robot.fieldConstants.fieldWidth / 2: # pass to left corner red (red relative)
                        self.robot.static_target = Translation2d(1.694, self.robot.fieldConstants.fieldWidth - 1.417)
                    else: #pass to right corner 
                        self.robot.static_target = Translation2d(1.694, 1.417)
            
            dist_from_shooter = shooter_pos.distance(self.robot.static_target)           
            
            temp_time_of_flight = self.dist_lookup_table.interpolate(dist_from_shooter)[2]
            temp_virtual_goal = Translation2d()
            field_relative_speeds = self.get_field_relative_speeds()
            SmartDashboard.putNumber("Test/ Field Rel X", field_relative_speeds.vx)
            SmartDashboard.putNumber("Test/ Field Rel Y", field_relative_speeds.vy)
            for _ in range(5):
                if True: # CHANGE TO CASES ON ALLIANCE ZONE AND NEUTRAL ZONE
                    temp_virtual_goal = Translation2d(
                        self.robot.static_target.X() - temp_time_of_flight * field_relative_speeds.vx, self.robot.static_target.Y() - temp_time_of_flight * field_relative_speeds.vy 
                    )
                virtual_robot_distance = (shooter_pos - temp_virtual_goal).norm()
                temp_time_of_flight = self.dist_lookup_table.interpolate(virtual_robot_distance)[2]

            SmartDashboard.putNumber("Virtual Goal Dist (shooter)", virtual_robot_distance)
            self.robot.virtual_target.setPose(Pose2d(temp_virtual_goal, Rotation2d()))
            self.robot.virtual_goal = temp_virtual_goal
            self.robot.time_of_flight = temp_time_of_flight

            vals = self.dist_lookup_table.interpolate(virtual_robot_distance)

            self.robot.fly_speed = self.vel_lookup_table.interpolate(vals[0])
            self.robot.hood_angle = self.angle_lookup_table.interpolate(vals[1])
            if self.robot.fly_speed >= 85:
                self.robot.fly_speed = 85

            if self.robot.hood_angle >= 39:
                self.robot.hood_angle = 39
            elif self.robot.hood_angle <= 0:
                self.robot.hood_angle = 0
        
        
        
        self.chassis_accel = (
            self.get_robot_relative_speeds() - self.previous_chassisspeeds
        ) / 0.05
        self.previous_chassisspeeds = self.get_robot_relative_speeds()

        if self.robot.in_autonomous_mode:
            if self.robot.poseEstimator.curEstPose.X() >= 5.172:
                # self.robot.is_intaking = True
                # self.robot.intake_at_default = False
                pass
            # if self.robot.fuel_in_hopper >= 9 and self:
            #     pass

            if self.robot.running_pid_lineup:
                if self.robot.shoot_intent and self.robot.poseEstimator.cur_pos_in_zone(4.5):
                        rotation = self.robot.drivetrain.get_hub_angle(self.robot.time_of_flight)
                        lineup  = Pose2d(self.robot.final_lineup_pose.X(), self.robot.final_lineup_pose.Y(), rotation)
                        SmartDashboard.putNumber("Shooter/Rotation to Hub", rotation.degrees())
                else:
                    lineup = self.robot.final_lineup_pose
                self.go_to_pose_profiled_pid(lineup)
            elif not self.robot.running_pid_lineup and self.robot.shoot_intent:
                rotation = self.get_hub_angle(self.robot.time_of_flight)
                self.drive_with_pid(Translation2d(0, 0), rotation, )
            # self.go_to_pose_profiled_pid(self.robot.final_lineup_pose)

    def log(self):
        SmartDashboard.putData("PID Controller Reef XY", self.xy_controller)
        SmartDashboard.putData("PID Controller for going to reef, x", self.x_controller)
        SmartDashboard.putData("PID Controller for going to reef, y", self.y_controller)
        SmartDashboard.putData(
            "PID Controller for going to reef, theta", self.theta_controller
        )
        SmartDashboard.putNumber("Distance to Hub", self.get_hub_distance(self.robot.poseEstimator.curEstPose.translation()))

        SmartDashboard.putData("PID Controller (Drivetrain)", self.angle_pid)
        SmartDashboard.putBoolean("Angle at Setpoint", self.angle_pid.atSetpoint())
        SmartDashboard.putNumber("PID Controller Error", self.angle_pid.getError())
        SmartDashboard.putData("PID Controller (XY Inter)", self.xy_inter_controller)