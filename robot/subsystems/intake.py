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
from motor_wrapper import MotorWrapper
import const
from wpilib import SmartDashboard

class Intake(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        
        self.left_intake_motor = MotorWrapper(const.LEFT_INTAKE_MOTOR_ID, "carnivore")
        self.right_intake_motor = MotorWrapper(const.RIGHT_INTAKE_MOTOR_ID, "carnivore")
        self.inside_track_motor = MotorWrapper(const.INSIDE_TRACK_MOTOR_ID, "carnivore")
        self.deploy_motor = MotorWrapper(const.INTAKE_DEPLOY_MOTOR_ID, "carnivore")

        self.right_intake_motor.set_control(controls.Follower(const.LEFT_INTAKE_MOTOR_ID, signals.MotorAlignmentValue(0)))
        self.inside_track_motor.set_control(controls.Follower(const.LEFT_INTAKE_MOTOR_ID, signals.MotorAlignmentValue(0)))

        self.commanded_intake_speed = 0.0
        self.commanded_position = 0.0

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
        self.deploy_motor.set_control(controls.VelocityTorqueCurrentFOC(rotations)) # USE MOTION MAGIC
    
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
    
    def periodic(self):
        if self.robot.is_intaking:
            if self.robot.fieldConstants.LinesVertical.starting < self.robot.poseEstimator.curEstPose.X() < self.robot.fieldConstants.fieldLength - self.robot.fieldConstants.LinesVertical.starting: # neutral zone
                self.set_intake_speed(0) # TUNE
                self.set_position(45) # TUNE
        elif self.robot.is_climbing:
            self.stop_intake()
            self.set_position(0.0)
        elif self.robot.mechanisms_at_default:
            self.stop_intake()
            self.set_position(0.0)
        

    def log(self):
        SmartDashboard.putBoolean("States/Is Intaking", self.robot.is_intaking)
        SmartDashboard.putNumber("Intake/Commanded Intake Speed", self.commanded_intake_speed)
        SmartDashboard.putNumber("Intake/Commanded Intake Position", self.commanded_position)
        SmartDashboard.putNumber("Intake/Actual Intake Position", self.get_position())
        SmartDashboard.putNumber("Intake/Actual Left Intake Speed", self.get_intake_speed())
        SmartDashboard.putNumber("Intake/Actual Right Intake Speed", self.right_intake_motor.get_velocity().value)
        
