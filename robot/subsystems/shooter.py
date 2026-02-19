from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot

from commands2 import Subsystem
from wpilibextra.coroutine.subsystem import Subsystem
from phoenix6 import controls, configs, hardware, signals
import wpilib
import wpimath
import wpimath.controller
from wpimath.geometry import Rotation2d, Translation2d, Translation3d, Rotation3d
from wpimath.trajectory import TrapezoidProfile
from wpimath.units import inchesToMeters, degreesToRadians
from wpilib import Timer
import math
import const
from wpilib import SmartDashboard
import shot_calc
import numpy as np
from lookup_table import LookupTableAll, LookupTableAngle, LookupTableVel

class Shooter(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.commanded_fly_speed = 0.0
        self.commanded_hood_position = 0.0
        self.commanded_accelerator_speed = 0.0
        self.request = controls.MotionMagicVoltage(0, enable_foc=True)

        # Flywheel motors
        self.right_fly_motor = hardware.TalonFX(const.RIGHT_FLY_ID, "rio")
        self.left_up_fly_motor = hardware.TalonFX(const.LEFT_UP_FLY_ID, "rio")
        self.left_down_fly_motor = hardware.TalonFX(const.LEFT_DOWN_FLY_ID, "rio")

        self.accelerator_motor = hardware.TalonFX(const.SHOOTER_ACCELERATOR_MOTOR_ID, "rio")

        self.hood_motor = hardware.TalonFX(const.SHOOTER_HOOD_MOTOR_ID, "rio")

        self.fly_motor_config = self.robot.get_motor_config(0, 10.0, 0, 0, 0.03, 0, 0, 3.5)
        # self.right_fly_motor_config = self.robot.get_motor_config(0, 12.0, 0.1, 0, 0, 0, 0, 0)
        self.right_fly_motor.configurator.apply(self.fly_motor_config)
        self.left_up_fly_motor.configurator.apply(self.fly_motor_config)
        self.left_down_fly_motor.configurator.apply(self.fly_motor_config)

        self.accelerator_motor_config = self.robot.get_motor_config(1, 7.0, 0, 0.025, 0, 0, 0, 9) # retune when we have metal plates
        self.accelerator_motor.configurator.apply(self.accelerator_motor_config)

        self.hood_motor_config = self.robot.get_motor_config(0, 2.0, 0, 0, 0, 0, 0, 0)
        self.hood_motor.configurator.apply(self.hood_motor_config)

        self.right_fly_motor.set_control(controls.Follower(const.LEFT_UP_FLY_ID, True))
        self.left_down_fly_motor.set_control(controls.Follower(const.LEFT_UP_FLY_ID, False))

        self.test_fly_speed = 70
        self.test_accelerator_speed = 80
        self.test_hood_position = 50

        self.shoot_ready = False
        self.accel_good = False

        self.dist_lookup_table = LookupTableAll()


    def get_fly_speed(self):
        if self.robot.isSimulation():
            return self.commanded_fly_speed
        else:
            return self.left_up_fly_motor.get_velocity().value
                    
    def set_fly_speed(self, speed):
        self.commanded_fly_speed = speed
        # if self.get_fly_speed() <= self.commanded_fly_speed * 0.85:
        #     self.right_fly_motor.set_control(controls.DutyCycleOut(0.97))
        # else:
        self.left_up_fly_motor.set_control(controls.VelocityTorqueCurrentFOC(-1 * speed))

    def get_accelerator_speed(self):
        if self.robot.isSimulation():
            return self.commanded_accelerator_speed
        else:
            return self.accelerator_motor.get_velocity().value

    def set_accelerator_speed(self, speed):
        self.commanded_accelerator_speed = speed
        # if self.get_accelerator_speed() <= self.commanded_accelerator_speed * 0.85:
        #     self.accelerator_motor.set_control(controls.VelocityDutyCycle(0.97))
        # else:
        self.accelerator_motor.set_control(controls.VelocityTorqueCurrentFOC(speed))
    
    def set_hood_position(self, position):
        if abs(self.get_hood_position() - position) <= 0.02:
            return
        if self.pose_in_trench():
            self.hood_motor.set_control(self.request.with_position(0))
        else:
            self.commanded_hood_position = position
            rotations = position # ADD GEAR RATIOS STUFF
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
        self.left_up_fly_motor.set_control(controls.VelocityTorqueCurrentFOC(0.0))

    def stop_accelerator(self):
        self.commanded_accelerator_speed = 0.0
        self.accelerator_motor.set_control(controls.VelocityTorqueCurrentFOC(0.0))

    def stop(self):
        self.stop_hood()
        self.stop_fly()
        self.stop_accelerator()
    
    def pose_in_trench(self):
        pose = self.robot.poseEstimator.curEstPose.translation() + Translation2d(0, 0.27).rotateBy(self.robot.poseEstimator.curEstPose.rotation())

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

    def ready_to_shoot(self):
        if abs(self.get_fly_speed()) + 5 >= self.commanded_fly_speed: #and pointed at hub   
            self.shoot_ready = True
            return True
        else:
            self.shoot_ready = False
            return False
    
    def fly_speed_to_launch_vel(self, fly_speed):
        return fly_speed / 4
    
    def create_lookup_table(self):
        min_dist = 0.7
        max_dist = 8
        num_points = 150
        shooter_height = 0.52 + self.robot.poseEstimator.estZ
        hub_pos = np.array([4.625594, 4.034536, 1.83])
        distances = np.linspace(min_dist, max_dist, num_points)
        results = []
        for dist in distances:
            shooter_pos = np.array([
                hub_pos[0] - dist, 
                hub_pos[1], 
                shooter_height])
            shooter_vel = np.array([0.0, 0.0, 0.0])
            optimizer = shot_calc.SleipnirRobustOptimizer(
            shooter_pos=shooter_pos,
            shooter_vel=shooter_vel,
            min_v=5.0,
            max_v=12.0,
            min_angle_deg=60,
            max_angle_deg=87.0
        )
            res = optimizer.optimize()

            if "SUCCESS" in str(res['status']):
                self.dist_lookup_table.add_entry(round(dist, 3), round(res['v'], 3), round(res['angle_deg']), round(res['T'], 4))
        

    def periodic(self):
        if self.robot.mechanisms_at_default:
            self.set_hood_position(0.0)
            self.set_accelerator_speed(0.0)
            self.set_fly_speed(0.0)
        elif self.robot.shoot_intent:
            self.set_fly_speed(40)
            self.set_hood_position(self.test_hood_position)
            if self.robot.shoot_fuel or self.ready_to_shoot() or self.shoot_ready:
                self.set_accelerator_speed(self.test_accelerator_speed)
                if abs(self.get_accelerator_speed()) + 30 >= self.commanded_accelerator_speed or self.accel_good:
                    self.accel_good = True
                    self.robot.hopper.indexer_motor.set_control(controls.DutyCycleOut(0.95))
                    # self.robot.hopper.set_speed(self.robot.hopper.test_indexer_speed) 
                else:
                    self.robot.hopper.set_speed(-20)
        elif self.robot.is_climbing:
            self.set_hood_position(0.0)
            self.stop_fly()
            self.stop_accelerator()

        
    def log(self):
        SmartDashboard.putNumber("Shooter/Right Fly Speed", self.right_fly_motor.get_velocity().value)
        SmartDashboard.putNumber("Shooter/Left Up Fly Speed", self.left_up_fly_motor.get_velocity().value)
        SmartDashboard.putNumber("Shooter/Left Down Fly Speed", self.left_down_fly_motor.get_velocity().value)
        SmartDashboard.putNumber("Shooter/Commanded Fly Speed", self.commanded_fly_speed)

        SmartDashboard.putNumber("Shooter/Accelerator Speed", self.get_accelerator_speed())
        SmartDashboard.putNumber("Shooter/Commanded Accelerator Speed", self.commanded_accelerator_speed)
        
        SmartDashboard.putNumber("Shooter/Hood Position", self.get_hood_position())
        SmartDashboard.putNumber("Shooter/Commanded Hood Position", self.commanded_hood_position)

        SmartDashboard.putBoolean("States/Shoot Fuel", self.robot.shoot_fuel)
        SmartDashboard.putBoolean("States/Shoot Intent", self.robot.shoot_intent)

        SmartDashboard.putBoolean("Shooter/Near Trench", self.pose_in_trench())
        
        SmartDashboard.putNumber("Test/Test fly speed", self.test_fly_speed)
        SmartDashboard.putNumber("Test/Test accelerator speed", self.test_accelerator_speed)
        SmartDashboard.putNumber("Test/Test hood position", self.test_hood_position)
