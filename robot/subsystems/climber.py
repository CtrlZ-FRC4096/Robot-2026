from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot

import wpilib
import wpimath.controller
from commands2 import Subsystem
import math
from wpimath.trajectory import TrapezoidProfile

import const
from wpilib import Timer

from phoenix6 import controls, configs, hardware, signals


# MOST OF THE METHODS IN THIS CLASS WILL NOT WORK RN
class Climber(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.climber_motor = hardware.TalonFX(const.CLIMBER_MOTOR_ID, "rio")
        self.climber_motor_config = self.robot.get_motor_config(0, 0, 0, 0, 0, 0, 0, 0)
        self.climber_motor_config.motion_magic.motion_magic_cruise_velocity = 0
        self.climber_motor_config.motion_magic.motion_magic_acceleration = 0 
        self.climber_motor.configurator.apply(self.climber_motor_config) 

        self.commanded_climber_position = 0.0

    def get_position(self):
        if self.robot.isSimulation():
            return self.commanded_climber_position
        else:
            self.climber_motor.get_position().value # DO GEAR RATIOS AND STUFF

    def set_position(self, position):
        self.commanded_climber_position = position
        rotations = position # DO INVERSE GEAR RATIOS AND stuff
        self.climber_motor.set_control(controls.MotionMagicVoltage(rotations, enable_foc=True))

    def stop(self):
        self.climber_motor.set_control(controls.MotionMagicVoltage(0.0))
        self.climber_motor.set_control(controls.StaticBrake())

    def periodic(self):
        if self.robot.is_climbing:
            pass
        else:
            self.climber_motor.set_control(controls.StaticBrake())  

    def log(self):
        wpilib.SmartDashboard.putNumber("Climber/Actual Position", self.climber_motor.get_position().value)
        wpilib.SmartDashboard.putNumber("Climber/Commanded Position", self.commanded_climber_position)
