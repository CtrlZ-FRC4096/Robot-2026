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
from math import sin, pi

class Hopper(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.commanded_speed = 0.0

        # Flywheel motors
        self.indexer_motor = MotorWrapper(const.INDEXER_MOTOR_ID, "carnivore")

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
        self.indexer_motor.set_control(controls.VelocityTorqueCurrentFOC(speed))

    def stop(self):
        self.commanded_speed = 0.0
        self.indexer_motor.set_control(controls.VelocityTorqueCurrentFOC(0.0))
    
    def periodic(self):
        if self.robot.shoot_fuel:
            self.set_speed(30.0) # TUNE
        elif self.robot.pulse_indexer:
            self.set_speed(abs(sin(self.time.get()*pi*self.hz)*self.amp)) # moves fuel towards shooter
        elif self.robot.is_climbing:
            self.stop()
        elif self.robot.mechanisms_at_default:
            self.stop()

    def log(self):
        SmartDashboard.putNumber("Hopper/Actual Speed", self.get_speed())
        SmartDashboard.putNumber("Hopper/Commanded Speed", self.commanded_speed)