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
from wpimath.units import inchesToMeters
from wpilib import Timer
import math
import const
from wpilib import SmartDashboard

class Shooter(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.commanded_fly_speed = 0.0
        self.commanded_hood_position = 0.0
        self.commanded_accelerator_speed = 0.0
        self.request = controls.MotionMagicVoltage(0, enable_foc=True)

        # Flywheel motors
        self.left_fly_motor = hardware.TalonFX(const.LEFT_FLY_ID, "rio")
        self.right_up_fly_motor = hardware.TalonFX(const.RIGHT_UP_FLY_ID, "rio")
        self.right_down_fly_motor = hardware.TalonFX(const.RIGHT_DOWN_FLY_ID, "rio")

        self.accelerator_motor = hardware.TalonFX(const.SHOOTER_ACCELERATOR_MOTOR_ID, "rio")

        self.hood_motor = hardware.TalonFX(const.SHOOTER_HOOD_MOTOR_ID, "rio")

        self.fly_motor_config = self.robot.get_motor_config()
        self.left_fly_motor.configurator.apply(self.fly_motor_config)
        self.right_up_fly_motor.configurator.apply(self.fly_motor_config)
        self.right_down_fly_motor.configurator.apply(self.fly_motor_config)

        self.accelerator_motor_config = self.robot.get_motor_config()
        self.accelerator_motor.configurator.apply(self.accelerator_motor_config)

        self.hood_motor_config = self.robot.get_motor_config()
        self.hood_motor.configurator.apply(self.hood_motor_config)

        self.right_up_fly_motor.set_control(controls.Follower(const.LEFT_FLY_ID, signals.MotorAlignmentValue(0)))
        self.right_down_fly_motor.set_control(controls.Follower(const.LEFT_FLY_ID, signals.MotorAlignmentValue(0)))

    def get_fly_speed(self):
        if self.robot.isSimulation():
            return self.commanded_fly_speed
        else:
            return 0.01 #self.left_fly_motor.get_velocity().value
                    
    def set_fly_speed(self, speed):
        self.commanded_fly_speed = speed
        self.left_fly_motor.set_control(controls.VelocityTorqueCurrentFOC(speed))

    def get_accelerator_speed(self):
        if self.robot.isSimulation():
            return self.commanded_accelerator_speed
        else:
            return 0.01 #self.accelerator_motor.get_velocity().value

    def set_accelerator_speed(self, speed):
        self.commanded_accelerator_speed = speed
        self.accelerator_motor.set_control(controls.VelocityTorqueCurrentFOC(speed))
    
    def set_hood_position(self, position):
        if abs(self.get_hood_position() - position) <= 0.02:
            return
        self.commanded_hood_position = position
        rotations = position # ADD GEAR RATIOS STUFF
        #self.hood_motor.set_control(controls.VelocityTorqueCurrentFOC(rotations)) # USE MOTION MAGIC
        self.hood_motor.set_control(self.request.with_position(rotations)) # USING MOTION MAGIC
    
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
    
    def pose_in_trench(self):
        pose = self.robot.poseEstimator.curEstPose

        min_x_blue = inchesToMeters(156.406)
        max_x_blue = inchesToMeters(205.406)
        min_x_red = inchesToMeters(446.156)
        max_x_red = inchesToMeters(494.844)

        min_y_right = inchesToMeters(0)
        max_y_right = inchesToMeters(51.219)
        min_y_left = inchesToMeters(267.268)
        max_y_left = inchesToMeters(318.111)
        
        if ((min_x_blue <= pose.X() <= max_x_blue and min_y_right <= pose.Y() <= max_y_right) # blue right trench
        or (min_x_blue <= pose.X() <= max_x_blue and min_y_left <= pose.Y() <= max_y_left) # blue left trench
        or (min_x_red <= pose.X() <= max_x_red and min_y_right <= pose.Y() <= max_y_right) # red right trench
        or (min_x_red <= pose.X() <= max_x_red and min_y_left <= pose.Y() <= max_y_left) # red left trench
        ):
            return True
        else:
            return False

    def periodic(self):
        if self.robot.mechanisms_at_default or self.pose_in_trench():
            self.set_hood_position(0.0)
            self.stop_accelerator()
            if self.robot.mechanisms_at_default:
                self.stop_fly()
        elif self.robot.shoot_fuel:
            self.set_fly_speed(30.0)
            self.set_accelerator_speed(30.0)
            self.set_hood_position(30.0)
        elif self.robot.shoot_intent:
            self.set_fly_speed(0.0)
            self.stop_accelerator()
            self.set_hood_position(60.0) #add pose checking
        elif self.robot.is_climbing:
            self.set_hood_position(0.0)
            self.stop_fly()
            self.stop_accelerator()
        
    def log(self):
        # SmartDashboard.putNumber("Shooter/Left Fly Speed", self.left_fly_motor.get_velocity().value)
        # SmartDashboard.putNumber("Shooter/Right Up Fly Speed", self.right_up_fly_motor.get_velocity().value)
        # SmartDashboard.putNumber("Shooter/Right Down Fly Speed", self.right_down_fly_motor.get_velocity().value)
        SmartDashboard.putNumber("Shooter/Commanded Fly Speed", self.commanded_fly_speed)

        # SmartDashboard.putNumber("Shooter/Accelerator Speed", self.get_accelerator_speed())
        SmartDashboard.putNumber("Shooter/Commanded Accelerator Speed", self.commanded_accelerator_speed)
        
        # SmartDashboard.putNumber("Shooter/Hood Position", self.get_hood_position())
        SmartDashboard.putNumber("Shooter/Commanded Hood Position", self.commanded_hood_position)

        SmartDashboard.putBoolean("States/Shoot Fuel", self.robot.shoot_fuel)
        SmartDashboard.putBoolean("States/Shoot Intent", self.robot.shoot_intent)

        SmartDashboard.putBoolean("Shooter/Near Trench", self.pose_in_trench())