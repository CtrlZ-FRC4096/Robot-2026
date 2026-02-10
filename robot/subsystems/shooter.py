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
        self.commanded_fly_speed = 0.0
        self.commanded_hood_position = 0.0
        self.commanded_accelerator_speed = 0.0

        # Flywheel motors
        self.left_fly_motor = MotorWrapper(const.LEFT_FLY_ID, "carnivore")
        self.right_up_fly_motor = MotorWrapper(const.RIGHT_UP_FLY_ID, "carnivore")
        self.right_down_fly_motor = MotorWrapper(const.RIGHT_DOWN_FLY_ID, "carnivore")

        self.accelerator_motor = MotorWrapper(const.SHOOTER_ACCELERATOR_MOTOR_ID, "carnivore")

        self.hood_motor = MotorWrapper(const.SHOOTER_HOOD_MOTOR_ID, "carnivore")

        self.right_up_fly_motor.set_control(controls.Follower(const.LEFT_FLY_ID, signals.MotorAlignmentValue(0)))
        self.right_down_fly_motor.set_control(controls.Follower(const.LEFT_FLY_ID, signals.MotorAlignmentValue(0)))

    def get_fly_speed(self):
        if self.robot.isSimulation():
            return self.commanded_fly_speed
        else:
            return self.left_fly_motor.get_velocity().value
                    
    def set_fly_speed(self, speed):
        self.commanded_fly_speed = speed
        self.left_fly_motor.set_control(controls.VelocityTorqueCurrentFOC(speed))

    def get_accelerator_speed(self):
        if self.robot.isSimulation():
            return self.commanded_accelerator_speed
        else:
            self.accelerator_motor.get_velocity().value

    def set_accelerator_speed(self, speed):
        self.commanded_accelerator_speed = speed
        self.accelerator_motor.set_control(controls.VelocityTorqueCurrentFOC(speed))
    
    def set_hood_position(self, position):
        if abs(self.get_hood_position() - position) <= 0.02:
            return
        self.commanded_hood_position = position
        rotations = position # ADD GEAR RATIOS STUFF
        self.hood_motor.set_control(controls.VelocityTorqueCurrentFOC(rotations)) # USE MOTION MAGIC
    
    def get_hood_position(self):
        if self.robot.isSimulation():
            return self.commanded_hood_position
        else:
            rotations = self.hood_motor.get_position().value 
            position = rotations# ADD GEAR RATIOS STUFF
            return position

    def stop_hood(self):
        self.hood_motor.set_control(controls.VelocityTorqueCurrentFOC(0.0))
    
    def stop_fly(self):
        self.commanded_fly_speed = 0.0
        self.left_fly_motor.set_control(controls.VelocityTorqueCurrentFOC(0.0))

    def stop_accelerator(self):
        self.commanded_accelerator_speed = 0.0
        self.accelerator_motor.set_control(controls.VelocityTorqueCurrentFOC(0.0))

    def stop(self):
        self.stop_hood()
        self.stop_fly()
        self.stop_accelerator()
    
    def periodic(self):
        if self.robot.shoot_fuel:
            self.set_fly_speed(30.0)
            self.set_accelerator_speed(30.0)
            self.set_hood_position(30.0)
        elif self.robot.shoot_intent:
            self.set_fly_speed(30.0)
            self.stop_accelerator()
            if True:
                self.set_hood_position(60.0) #add pose checking
        elif self.robot.is_climbing:
            self.set_hood_position(0.0)
            self.stop_fly()
            self.stop_accelerator()
        elif self.robot.mechanisms_at_default:
            self.set_hood_position(0.0)
            self.stop_fly()
            self.stop_accelerator()
    def log(self):
        SmartDashboard.putNumber("Shooter/Left Fly Speed", self.left_fly_motor.get_velocity().value)
        SmartDashboard.putNumber("Shooter/Right Up Fly Speed", self.right_up_fly_motor.get_velocity().value)
        SmartDashboard.putNumber("Shooter/Right Down Fly Speed", self.right_down_fly_motor.get_velocity().value)
        SmartDashboard.putNumber("Shooter/Commanded Fly Speed", self.commanded_fly_speed)

        SmartDashboard.putNumber("Shooter/Accelerator Speed", self.get_accelerator_speed())
        SmartDashboard.putNumber("Shooter/Commanded Accelerator Speed", self.commanded_accelerator_speed)
        
        SmartDashboard.putNumber("Shooter/Hood Position", self.get_hood_position())
        SmartDashboard.putNumber("Shooter/Commanded Hood Position", self.commanded_hood_position)

        SmartDashboard.putBoolean("States/Shoot Fuel", self.robot.shoot_fuel)
        SmartDashboard.putBoolean("States/Shoot Intent", self.robot.shoot_intent)