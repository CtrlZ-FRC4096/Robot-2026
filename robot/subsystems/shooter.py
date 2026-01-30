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

class Shooter(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.command_speed = 0.0

        # Flywheel motors
        self.top_right_motor = MotorWrapper(const.TOP_RIGHT_MOTOR_ID, "canivore")  # ID?
        self.top_left_motor = MotorWrapper(const.TOP_LEFT_MOTOR_ID, "canivore")
        self.bottom_right_motor = MotorWrapper(const.BOTTOM_RIGHT_MOTOR_ID, "canivore")
        self.bottom_left_motor = MotorWrapper(const.LEFT_RIGHT_MOTOR_ID, "canivore")

        self.motors = [self.top_right_motor, self.top_left_motor, self.bottom_right_motor, self.bottom_left_motor]

    def get_speed(self):
        if self.robot.isSimulation():
            return self.commanded_speed
        else:
            return self.top_right_motor.get_velocity().value, \
                    self.top_left_motor.get_velocity().value, \
                    self.bottom_right_motor.get_velocity().value, \
                    self.bottom_left_motor.get_velocity().value
    
    def set_speed(self, speed):
        self.command_speed = speed

        for motor in self.motors:
            motor.set_control(controls.VelocityTorqueCurrentFOC(speed))

    def stop(self):
        self.command_speed = 0.0
        for motor in self.top_motors + self.bottom_motors:
            motor.set_control(controls.VelocityTorqueCurrentFOC(0.0))

    def log(self):
        tr, tl, br, bl = self.get_speed()
        SmartDashboard.putNumber("Shooter/Top Right Speed", tr)
        SmartDashboard.putNumber("Shooter/Top Left Speed", tl)
        SmartDashboard.putNumber("Shooter/Bottom Right Speed", br)
        SmartDashboard.putNumber("Shooter/Bottom Left Speed", bl)
        SmartDashboard.putNumber("Shooter/Average Speed", sum([tr, tl, br, bl]) / 4)
        SmartDashboard.putNumber("Shooter/Commanded Speed", self.command_speed)
