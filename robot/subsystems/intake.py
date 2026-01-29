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
        
        self.intake_motor = MotorWrapper(const.INTAKE_MOTOR_ID, "canivore")
        self.deploy_motor = MotorWrapper(const.INTAKE_DEPLOY_MOTOR_ID, "canivore")

        self.commanded_intake_speed = 0.0
        self.commanded_position = 0.0

    def stop_intake(self):
        self.intake_motor.set_control(controls.VelocityTorqueCurrentFOC(0.0))
        self.intake_motor.set_control(controls.StaticBrake())
    
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
        
    def set_intake_speed(self, speed):
        self.commanded_intake_speed = speed
        self.intake_motor.set_control(controls.VelocityTorqueCurrentFOC(speed))

    def get_intake_speed(self):
        if self.robot.isSimulation():
            return self.commanded_intake_speed
        else:
            return self.intake_motor.get_velocity().value
    
    def periodic(self):
        pass

    def log(self):
        SmartDashboard.putNumber("Commanded Intake Speed", self.commanded_intake_speed)
        SmartDashboard.putNumber("Commanded Intake Position", self.commanded_position)
        SmartDashboard.putNumber("Actual Intake Position", self.get_position())
        SmartDashboard.putNumber("Actual Intake Speed", self.get_intake_speed())
        
