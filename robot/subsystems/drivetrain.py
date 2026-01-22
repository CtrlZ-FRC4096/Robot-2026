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
)
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState
)
from phoenix6 import configs
# from shapely import Polygon, Point
# from shapely.affinity import translate, rotate


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
from pathplannerlib.path import PathPlannerPath, PathConstraints
from wpimath.estimator import SwerveDrive4PoseEstimator
from photoncamera import WrapperedPhotonCamera
from wpimath.units import degreesToRadians, inchesToMeters, radiansToDegrees
from collections import deque

class Drivetrain(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot

        self.angle_pid = PIDController(0.075, 0.0, 0.001)
        self.angle_pid.enableContinuousInput(0, 360)
        self.angle_pid.setTolerance(0.5)  # Set position tolerance to 0.5 degrees

        self.x_controller = PIDController(2.25, 0.01, 0.025) #0.01
        self.y_controller = PIDController(2.25, 0.01, 0.025) #0.01
        self.xy_controller = ProfiledPIDController(2.3, 0.0, 0.025, TrapezoidProfile.Constraints(4.0, 4.0))
        self.theta_controller = PIDController(0.07, 0.01, 0.0015)
        
        self.inter_max_vel = 4.0
        self.inter_max_acc = 4.0
        constraints = TrapezoidProfile.Constraints(self.inter_max_vel, self.inter_max_acc)
        # self.xy_controller = ProfiledPIDController(2.0, 0.01, 0.025)#, constraints, period=0.05)
        self.xy_inter_controller = ProfiledPIDController(1.9, 0.0, 0.0, constraints)


        self.previous_sim_speeds = ChassisSpeeds()
        self.two_previous_sim_speeds = ChassisSpeeds()
        self.damping_accel = False


        ## Need to check these tolerances
        self.x_controller.setTolerance(0.03, 0.1) #0.025, 0.1
        self.y_controller.setTolerance(0.03, 0.1) #0.025, 0.1
        self.xy_controller.setTolerance(0.0225, 0.15)
        self.theta_controller.enableContinuousInput(0, 360)
        self.theta_controller.setTolerance(2.8, 2.0) #3.0, 0.1
        self.xy_inter_controller.setTolerance(0.15, 2.0)

        self.log_chassis = ChassisSpeeds()

        self.final_velo = Translation2d()

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
                    -rotation,
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

            self.robot.poseEstimator.curEstPose = Pose2d(curPose.X() + final_vel.X() / 30, curPose.Y() + final_vel.Y() / 30, Rotation2d.fromDegrees(curPose.rotation().degrees() + final_vel.rotation().degrees() / 27))
            if self.robot.poseEstimator.poseIsOffField(self.robot.poseEstimator.curEstPose):
                self.robot.poseEstimator.curEstPose = curPose
            self.robot.poseEstimator.set_yaw(self.robot.poseEstimator.curEstPose.rotation().degrees() + self.log_chassis.omega_dps / 20)
            
            self.two_previous_sim_speeds = self.previous_sim_speeds
            self.previous_sim_speeds = ChassisSpeeds(final_vel.X(), final_vel.Y())

        else:
            for idx, module in enumerate(self.robot.poseEstimator.modules):
                SmartDashboard.putNumber("module state " + str(idx + 1), module_states[idx].speed)
                module.set_desired_state(module_states[idx], is_open_loop)
    
    def drive_with_pid(self, translation: Translation2d, target_angle):
        pid_output = self.angle_pid.calculate(self.robot.poseEstimator.getYaw().degrees(), target_angle)  # type: ignore

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
        self, chassis_speeds: ChassisSpeeds, feedfoward=None
    ):  # only use for pathplannerlib
        chassis_speeds.omega = -chassis_speeds.omega
        module_states = const.SWERVE_KINEMATICS.toSwerveModuleStates(chassis_speeds)

        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            module_states, const.SWERVE_MAX_SPEED
        )

        for idx, module in enumerate(self.robot.poseEstimator.modules):
            # print(module_states[idx].speed)
            module.set_desired_state(module_states[idx], is_open_loop=False)

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

        # Check if the controllers are at their setpoints
        if (
            self.x_controller.atSetpoint()
            and self.y_controller.atSetpoint()
            and self.theta_controller.atSetpoint()
        ):
            # self.robot.running_pid_lineup = False
            if self.robot.score_intent and self.robot.running_pid_lineup:
                # self.at_scoring_position_drivetrain.appendleft(True)
                self.robot.at_scoring_position = True
            if self.robot.is_intaking and self.robot.running_pid_lineup:
                self.robot.at_intake_position = True


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
        return chassis_speeds

    def shouldFlipPath(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def periodic(self):
        if self.robot.in_autonomous_mode and self.robot.running_pid_lineup:
                self.go_to_pose_profiled_pid(self.robot.final_lineup_pose)

    def log(self):
        SmartDashboard.putData("PID Controller Reef XY", self.xy_controller)
        SmartDashboard.putData("PID Controller for going to reef, x", self.x_controller)
        SmartDashboard.putData("PID Controller for going to reef, y", self.y_controller)
        SmartDashboard.putData(
            "PID Controller for going to reef, theta", self.theta_controller
        )

        SmartDashboard.putData("PID Controller (Drivetrain)", self.angle_pid)
        SmartDashboard.putBoolean("Angle at Setpoint", self.angle_pid.atSetpoint())
        SmartDashboard.putNumber("PID Controller Error", self.angle_pid.getError())
        SmartDashboard.putData("PID Controller (XY Inter)", self.xy_inter_controller)