from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot

from commands2 import Subsystem, SequentialCommandGroup
from wpilibextra.coroutine.subsystem import Subsystem
from phoenix6 import controls, configs, hardware, signals
import wpilib
import wpimath
import wpimath.controller
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from wpimath.trajectory import TrapezoidProfile
from wpilib import Timer
import math
import const
from wpilib import SmartDashboard

from math import sin, pi

class Hopper(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.commanded_speed = 0.0

        # Flywheel motors
        self.indexer_motor = hardware.TalonFX(const.INDEXER_MOTOR_ID, "rio")  
        self.indexer_motor_config = self.robot.get_motor_config(1, 15.0, 0.0, 0.0, 0.55, 0, 0, 26.5)
        self.indexer_motor_config.current_limits.supply_current_limit = 80
        self.indexer_motor_config.torque_current.peak_forward_torque_current = 80
        self.indexer_motor_config.torque_current.peak_reverse_torque_current = -80
        self.indexer_motor.configurator.apply(self.indexer_motor_config)
        self.test_indexer_speed = 80

        # pulsing indexer
        self.hz = 4
        self.amp = 3

        self.time = Timer()


    def get_speed(self):
        if self.robot.isSimulation():
            return self.commanded_speed
        else:
            return self.indexer_motor.get_velocity().value
                    
    def set_speed(self, speed):
        self.commanded_speed = speed
        self.indexer_motor.set_control(controls.VelocityVoltage(speed))

    def stop(self):
        self.commanded_speed = 0.0
        self.indexer_motor.set_control(controls.DutyCycleOut(0.0))

    def periodic(self):
        start_time = wpilib.RobotController.getFPGATime()

        if not self.robot.did_autonomous and self.robot.using_auto:
            if not self.robot.auto_submitted and SmartDashboard.getNumber("Submit Auto? (and FMS Connected)", 0):
                self.robot.auto_submitted = True
                self.robot.fieldConstants.shouldFlip = self.robot.driverstation.getAlliance() == self.robot.driverstation.Alliance.kRed
                if self.robot.fieldConstants.shouldFlip:
                    gyro_offset = 90
                else:
                    gyro_offset = -90
                self.robot.poseEstimator.gyro.set_yaw(gyro_offset)
                self.robot.robot_oriented_angle = gyro_offset
                match self.robot.auto_chooser.getSelected():
                    case 1: # Left Default Trench Bump PP Auto Robust
                        self.robot.poseEstimator.poseEst.resetPose(Pose2d(self.robot.fieldConstants.flip_Translation2d(Translation2d(4.471, 7.587)), Rotation2d.fromDegrees(gyro_offset)))
                        self.robot.poseEstimator.curEstPose = Pose2d(self.robot.fieldConstants.flip_Translation2d(Translation2d(4.471, 7.587)), Rotation2d.fromDegrees(gyro_offset))
                        
                        self.robot.auto = self.robot.autoroutines.left_trench_bump_robust_new()
                    case 2: # Right Default Trench Bump PP Auto Robust
                        self.robot.poseEstimator.poseEst.resetPose(Pose2d(self.robot.fieldConstants.flip_Translation2d(Translation2d(4.47, 0.6)), Rotation2d.fromDegrees(gyro_offset)))
                        self.robot.poseEstimator.curEstPose = Pose2d(self.robot.fieldConstants.flip_Translation2d(Translation2d(4.47, 0.6)), Rotation2d.fromDegrees(gyro_offset))
                        
                        self.robot.auto = self.robot.autoroutines.right_trench_bump_robust_new()
                    case 3: # Rigth Trench Bump Safe / Default 3-BOT BL
                        self.robot.poseEstimator.poseEst.resetPose(Pose2d(self.robot.fieldConstants.flip_Translation2d(Translation2d(3.539, 0.628)), Rotation2d.fromDegrees(gyro_offset)))
                        self.robot.poseEstimator.curEstPose = Pose2d(self.robot.fieldConstants.flip_Translation2d(Translation2d(3.539, 0.628)), Rotation2d.fromDegrees(gyro_offset))

                        self.robot.auto = self.robot.autoroutines.right_trench_bump_safe(5)
                    case 0:
                        self.robot.poseEstimator.poseEst.resetPose(Pose2d(self.robot.fieldConstants.flip_Translation2d(Translation2d(4.471, 4.411)), Rotation2d.fromDegrees(gyro_offset)))
                        self.robot.poseEstimator.curEstPose = Pose2d(self.robot.fieldConstants.flip_Translation2d(Translation2d(4.471, 4.411)), Rotation2d.fromDegrees(gyro_offset))
                        
                        self.robot.auto = SequentialCommandGroup()
                        
        if not self.robot.shoot_intent and self.robot.is_intaking:
            # self.commanded_speed = -0.95
            # self.indexer_motor.set_control(controls.DutyCycleOut(-0.95, enable_foc=False))
            pass
        # elif self.robot.pulse_indexer:
        #     self.set_speed(abs(sin(self.time.get()*pi*self.hz)*self.amp)) # moves fuel towards shooter
        elif (not self.robot.shoot_intent and not self.robot.down_bad) and self.robot.intake_at_default:
            self.commanded_speed = 0
            self.indexer_motor.set_control(controls.DutyCycleOut(0.0))
        elif self.robot.shooter_at_default:
            self.stop()

        # ADD WEIGHT CODE HERE
        # weight_ratio = self.robot.poseEstimator.get_weight_by_accel()
        
        elapsed_ms = (wpilib.RobotController.getFPGATime() - start_time) / 1000
        SmartDashboard.putNumber("Loop Times/Hopper", elapsed_ms)

    def log(self):
        SmartDashboard.putNumber("Hopper/Actual Speed", self.get_speed())
        SmartDashboard.putNumber("Hopper/Commanded Speed", self.commanded_speed)
        SmartDashboard.putNumber("Test/Test indexer speed", self.test_indexer_speed)

