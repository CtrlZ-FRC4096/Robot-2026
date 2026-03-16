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
from wpimath.units import radiansToDegrees, inchesToMeters
from wpimath.controller import PIDController

class Intake(Subsystem):
    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot
        self.request = controls.MotionMagicVoltage(0, enable_foc=False)

        self.left_intake_motor = hardware.TalonFX(const.LEFT_INTAKE_MOTOR_ID, "rio")
        self.right_intake_motor = hardware.TalonFX(const.RIGHT_INTAKE_MOTOR_ID, "rio")
        self.deploy_motor = hardware.TalonFX(const.INTAKE_DEPLOY_MOTOR_ID, "rio")
        self.deploy_cancoder = hardware.CANcoder(const.INTAKE_DEPLOY_CANCODER_ID, "rio")

        deploy_cancoder_config = configs.CANcoderConfiguration()
        #        self.talonfx.configurator.apply(configs.TalonFXConfiguration())
        deploy_cancoder_config.magnet_sensor.sensor_direction = (
            signals.InvertedValue(0)
        )
        deploy_cancoder_config.magnet_sensor.magnet_offset = -0.27

        self.deploy_cancoder.configurator.apply(
            deploy_cancoder_config  # type: ignore
        )  # Apply settings to angle encoder

        self.intake_motor_config = self.robot.get_motor_config(1, 5, 0, 0, 0.21, 0, 0, 11)
        self.deploy_motor_config = self.robot.get_motor_config(1, 80, 0, 15, 0, 0, 10, 8)
        self.deploy_motor_config.motion_magic.motion_magic_cruise_velocity = 20
        self.deploy_motor_config.motion_magic.motion_magic_acceleration = 40
        self.deploy_motor_config.feedback.feedback_remote_sensor_id = const.INTAKE_DEPLOY_CANCODER_ID
        self.deploy_motor_config.feedback.feedback_sensor_source = signals.FeedbackSensorSourceValue.REMOTE_CANCODER

        self.deploy_motor_config.current_limits.supply_current_limit = 80
        self.deploy_motor_config.torque_current.peak_forward_torque_current = 80
        self.deploy_motor_config.torque_current.peak_reverse_torque_current = -80

        self.left_intake_motor.configurator.apply(self.intake_motor_config)
        self.right_intake_motor.configurator.apply(self.intake_motor_config)
        self.deploy_motor.configurator.apply(self.deploy_motor_config)

        self.right_intake_motor.set_control(controls.Follower(const.LEFT_INTAKE_MOTOR_ID, signals.MotorAlignmentValue(1)))

        # self.deploy_encoder = wpilib.DutyCycleEncoder(6)
        self.commanded_intake_speed = 0.0
        self.commanded_position = 0.0
        self.test_intake_speed = 50

        self.deploy_pid_controller = PIDController(0.01, 0, 0)
        self.intake_reverse_count = 0

        self.tick_count = 0

    def stop(self):
        self.stop_deploy()
        self.stop_intake()

    def stop_intake(self):
        self.commanded_intake_speed = 0.0
        self.left_intake_motor.set_control(controls.DutyCycleOut(0.0, enable_foc=False))

    def stop_deploy(self):
        self.deploy_motor.set_control(controls.DutyCycleOut(0.0, enable_foc=False))

    def set_position(self, position):
        '''
            position is in degrees
        '''
        
        if self.intake_pose_in_trench():
            self.commanded_position = -0.05
            if abs(self.get_position() - 0.05) <= 0.02:
                self.stop_deploy()
            else:
                self.deploy_motor.set_control(controls.MotionMagicTorqueCurrentFOC(-0.05))
        else:
            self.commanded_position = position
            if abs(self.get_position() - self.commanded_position) <= 0.02:
                self.stop_deploy()
            else:
                self.deploy_motor.set_control(controls.MotionMagicTorqueCurrentFOC(position)) # USING MOTION MAGIC

    def get_position(self):
        if self.robot.isSimulation():
            return self.commanded_position
        else:
            rotations = self.deploy_motor.get_position().value
            position = rotations# ADD GEAR RATIOS STUFF
            return position

    def intake_pose_in_trench(self):
        pose = self.robot.poseEstimator.curEstPose.translation() + Translation2d(0, -0.4).rotateBy(self.robot.poseEstimator.curEstPose.rotation())

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

    def can_intake_sim(self):
        return self.robot.is_intaking and self.robot.fuel_in_hopper < 24

    def intake_sim_callback(self):
        self.robot.fuel_in_hopper += 1

    def set_intake_speed(self, speed):
        self.commanded_intake_speed = speed
        self.left_intake_motor.set_control(controls.DutyCycleOut(speed, enable_foc=False))

    def get_intake_speed(self):
        if self.robot.isSimulation():
            return self.commanded_intake_speed
        else:
            return self.left_intake_motor.get_velocity().value

    def get_snake_intake_angle(self):
        cur_speeds = self.robot.drivetrain.get_field_relative_speeds()
        cur_rotation = self.robot.poseEstimator.curEstPose.rotation().degrees()
        # angle = math.atan2(cur_speeds.vx, cur_speeds.vy)
        angle = Translation2d(cur_speeds.vx, cur_speeds.vy).angle().degrees()
        if abs(cur_speeds.vx) <= 0.1 and abs(cur_speeds.vy) <= 0.1 or (abs(angle-cur_rotation) <= 5):
            return cur_rotation
        return angle

    def periodic(self):
        if self.robot.intake_at_default:
            self.stop_intake()
            self.set_position(0.19)
        elif self.robot.clear_jam:
            self.set_intake_speed(-0.8)
            self.set_position(-0.05)
        elif self.robot.is_intaking:
            # if self.robot.fieldConstants.LinesVertical.starting < self.robot.poseEstimator.curEstPose.X() < self.robot.fieldConstants.fieldLength - self.robot.fieldConstants.LinesVertical.starting: # neutral zone
            self.set_intake_speed(0.8) # TUNE
            self.set_position(-0.05) # TUNE
        elif self.robot.pulse_pivot:
            if self.tick_count % 20 < 10:
                # print("switch to out")
                self.set_position(-0.05)
            else:
                # print("switch to in")
                self.set_position(0.19)
            self.set_intake_speed(0.8)
        else:
            self.stop_intake()
            self.set_position(-0.05)

        self.tick_count += 1

    def log(self):
        SmartDashboard.putBoolean("States/Is Intaking", self.robot.is_intaking)
        SmartDashboard.putNumber("Intake/Commanded Intake Speed", self.commanded_intake_speed)
        SmartDashboard.putNumber("Intake/Commanded Intake Position", self.commanded_position)
        SmartDashboard.putNumber("Intake/Actual Intake Position", self.get_position())
        SmartDashboard.putNumber("Intake/Actual Left Intake Speed", self.get_intake_speed())
        SmartDashboard.putNumber("Intake/Actual Right Intake Speed", self.right_intake_motor.get_velocity().value)
        # SmartDashboard.putNumber("Intake/Intake Encoder Position", self.deploy_encoder.get())
        # SmartDashboard.putBoolean("Intake/Intake Encoder Connected", self.deploy_encoder.isConnected())
        SmartDashboard.putData("Intake/Deploy PID Controller", self.deploy_pid_controller)

        SmartDashboard.putNumber("Test/Test intake speed", self.test_intake_speed)
        SmartDashboard.putNumber("Intake/Snake Angle", self.get_snake_intake_angle())
