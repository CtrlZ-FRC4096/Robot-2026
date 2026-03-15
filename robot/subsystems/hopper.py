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
from math import sin, pi

class Hopper(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.commanded_speed = 0.0

        # Flywheel motors
        self.indexer_motor = hardware.TalonFX(const.INDEXER_MOTOR_ID, "rio")  
        self.indexer_motor_config = self.robot.get_motor_config(1, 15.0, 0.0, 0.0, 0.55, 0, 0, 26.5)
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
        self.indexer_motor.set_control(controls.VelocityVoltage(speed, enable_foc=False))

    def stop(self):
        self.commanded_speed = 0.0
        self.indexer_motor.set_control(controls.DutyCycleOut(0.0, enable_foc=False))

    def periodic(self):
        if not self.robot.shoot_intent and self.robot.is_intaking:
            # self.commanded_speed = -0.95
            # self.indexer_motor.set_control(controls.DutyCycleOut(-0.95, enable_foc=False))
            pass
        # elif self.robot.pulse_indexer:
        #     self.set_speed(abs(sin(self.time.get()*pi*self.hz)*self.amp)) # moves fuel towards shooter
        elif not self.robot.shoot_intent and self.robot.intake_at_default:
            self.commanded_speed = 0
            self.indexer_motor.set_control(controls.DutyCycleOut(0.0, enable_foc=False))
        elif self.robot.shooter_at_default:
            self.stop()

        # ADD WEIGHT CODE HERE
        weight_ratio = self.robot.poseEstimator.get_weight_by_accel()
        if self.robot.isTeleop():
            self.robot.update_hub_status()

    def log(self):
        SmartDashboard.putNumber("Hopper/Actual Speed", self.get_speed())
        SmartDashboard.putNumber("Hopper/Commanded Speed", self.commanded_speed)
        SmartDashboard.putNumber("Test/Test indexer speed", self.test_indexer_speed)

