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
import motor_wrapper


class Intake(Subsystem):
    pass
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        
        self.wheel_motor = motor_wrapper.MotorWrapper(21, "canivore")
        