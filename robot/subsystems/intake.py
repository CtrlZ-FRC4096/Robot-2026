from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot

from commands2 import Subsystem
from wpilibextra.coroutine.subsystem import Subsystem
from phoenix6 import controls, configs, hardware, signals
import wpilib
import wpimath
import wpimath.controller
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.trajectory import TrapezoidProfile
from wpilib import Timer
import math
import const
from wpilib import SmartDashboard
from wpimath.units import radiansToDegrees

class Intake(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.request = controls.MotionMagicVoltage(0, enable_foc=True)
        
        self.left_intake_motor = hardware.TalonFX(const.LEFT_INTAKE_MOTOR_ID, "carnivore")
        self.right_intake_motor = hardware.TalonFX(const.RIGHT_INTAKE_MOTOR_ID, "carnivore")
        # self.inside_track_motor = hardware.TalonFX(const.INSIDE_TRACK_MOTOR_ID, "carnivore")
        self.deploy_motor = hardware.TalonFX(const.INTAKE_DEPLOY_MOTOR_ID, "carnivore")

        self.intake_motor_config = self.robot.get_motor_config(0, 5, 0, 0, 0.21, 0, 0, 11)
        self.inside_track_motor_config = self.robot.get_motor_config(0, 1, 0, 0, 0, 0, 0, 0)
        self.deploy_motor_config = self.robot.get_motor_config(0, 1, 0, 0, 0, 0, 0, 0)

        self.left_intake_motor.configurator.apply(self.intake_motor_config)
        self.right_intake_motor.configurator.apply(self.intake_motor_config)
        # self.inside_track_motor.configurator.apply(self.inside_track_motor_config)
        self.deploy_motor.configurator.apply(self.deploy_motor_config)

        self.right_intake_motor.set_control(controls.Follower(const.LEFT_INTAKE_MOTOR_ID, signals.MotorAlignmentValue(1)))
        # self.inside_track_motor.set_control(controls.Follower(const.LEFT_INTAKE_MOTOR_ID, signals.MotorAlignmentValue(0)))

        self.deploy_encoder = wpilib.DutyCycleEncoder(0)
        self.commanded_intake_speed = 0.0
        self.commanded_position = 0.0
        self.test_intake_speed = 50

    def stop(self):
        self.stop_deploy()
        self.stop_intake()

    def stop_intake(self):
        self.commanded_intake_speed = 0.0
        self.left_intake_motor.set_control(controls.VelocityTorqueCurrentFOC(0.0))
    
    def stop_deploy(self):
        self.deploy_motor.set_control(controls.VelocityTorqueCurrentFOC(0.0))

    def set_position(self, position):
        if abs(self.get_position() - position) <= 0.02:
            return
        self.commanded_position = position
        rotations = position # ADD GEAR RATIOS STUFF
        #self.deploy_motor.set_control(controls.PositionTorqueCurrentFOC(rotations)) # USE MOTION MAGIC
        self.deploy_motor.set_control(self.request.with_position(rotations)) # USING MOTION MAGIC
    
    def get_position(self):
        if self.robot.isSimulation():
            return self.commanded_position
        else:
            rotations = self.deploy_motor.get_position().value 
            position = rotations# ADD GEAR RATIOS STUFF
            return position
        
    def can_intake_sim(self):
        return self.robot.is_intaking and self.robot.fuel_in_hopper < 24
    
    def intake_sim_callback(self):
        self.robot.fuel_in_hopper += 1
        
    def set_intake_speed(self, speed):
        self.commanded_intake_speed = speed
        self.left_intake_motor.set_control(controls.VelocityTorqueCurrentFOC(speed))

    def get_intake_speed(self):
        if self.robot.isSimulation():
            return self.commanded_intake_speed
        else:
            return self.left_intake_motor.get_velocity().value
    
    def get_snake_intake_angle(self):
        cur_speeds = self.robot.drivetrain.get_robot_relative_speeds()
        cur_rotation = self.robot.poseEstimator.curEstPose.rotation().degrees()
        angle = math.atan2(cur_speeds.vy, cur_speeds.vx)
        if abs(cur_speeds.vx) <= 0.01 and abs(cur_speeds.vy) <= 0.01:
            return cur_rotation
        return radiansToDegrees(angle)

    def periodic(self):
        if self.robot.is_intaking:
            # if self.robot.fieldConstants.LinesVertical.starting < self.robot.poseEstimator.curEstPose.X() < self.robot.fieldConstants.fieldLength - self.robot.fieldConstants.LinesVertical.starting: # neutral zone
                self.set_intake_speed(self.test_intake_speed) # TUNE
                # self.set_position(45) # TUNE
        elif self.robot.is_climbing:
            self.stop_intake()
            # self.set_position(0.0)
        elif self.robot.mechanisms_at_default:
            self.stop_intake()
            # self.set_position(0.0)
        

    def log(self):
        SmartDashboard.putBoolean("States/Is Intaking", self.robot.is_intaking)
        SmartDashboard.putNumber("Intake/Commanded Intake Speed", self.commanded_intake_speed)
        SmartDashboard.putNumber("Intake/Commanded Intake Position", self.commanded_position)
        SmartDashboard.putNumber("Intake/Actual Intake Position", self.get_position())
        SmartDashboard.putNumber("Intake/Actual Left Intake Speed", self.get_intake_speed())
        SmartDashboard.putNumber("Intake/Actual Right Intake Speed", self.right_intake_motor.get_velocity().value)
        SmartDashboard.putNumber("Intake/Intake Encoder Position", self.deploy_encoder.get())

        SmartDashboard.putNumber("Test/Test intake speed", self.test_intake_speed)
        
