"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2022
Code for robot "swerve drivetrain prototype"
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

"""
Prepend these to any port IDs.
DIO = Digital I/O
AIN = Analog Input
PWM = Pulse Width Modulation
CAN = Controller Area Network
PCM = Pneumatic Control Module
PDP = Power Distribution Panel
"""

import math

from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics

from pathplannerlib.config import ModuleConfig, DCMotor, RobotConfig

from phoenix6 import signals
import numpy as np

from wpimath.units import volts, newton_meters, amperes

from wpimath.units import degreesToRadians, inchesToMeters



### CONSTANTS ###

# Is running simulator. Value is set in robot.py, robotInit
IS_SIMULATION = False

# Directions, mainly used for swerve module positions on drivetrain
FRONT_LEFT = "front_left"
FRONT_RIGHT = "front_right"
BACK_LEFT = "back_left"
BACK_RIGHT = "back_right"

# Robot drivebase dimensions, in inches and meters
DRIVE_BASE_WIDTH = 29.0
DRIVE_BASE_LENGTH = 29.0
# DRIVE_BASE_RADIUS_METERS = 0.381660882
DRIVETRAIN_TRACKWIDTH_METERS = 0.616
DRIVETRAIN_WHEELBASE_METERS = 0.616

SWERVE_WHEEL_CIRCUMFERENCE = math.pi * (4 * 2.54 / 100)  # C = pi*d
SWERVE_DRIVE_GEAR_RATIO = 6.72  # From belt kit we ordered
SWERVE_ANGLE_GEAR_RATIO = 13.3714  # From Swerve X user guide, for flipped, belt models


SWERVE_MAX_SPEED = 4.75  # 4.75  # meters per second


# Module PID Constants

# Old values from 2022 Swerve X modules
# SWERVE_ANGLE_KP = 0.43
# SWERVE_ANGLE_KI = 0
# SWERVE_ANGLE_KD = 0.004
# SWERVE_ANGLE_KF = 0

# Values from swerve-test bot MK4 modules
# SWERVE_ANGLE_KP = 0.232
# SWERVE_ANGLE_KI = 0.0625
# SWERVE_ANGLE_KD = 0
# SWERVE_ANGLE_KF = 0

SWERVE_ANGLE_KP = 2.4
SWERVE_ANGLE_KI = 0
SWERVE_ANGLE_KD = 0.1
SWERVE_ANGLE_KF = 0

SWERVE_DRIVE_KP = 0.3  # 0.4
SWERVE_DRIVE_KI = 0
SWERVE_DRIVE_KD = 0
SWERVE_DRIVE_KF = 0
SWERVE_DRIVE_KS = 0.1

SWERVE_DRIVE_KV = 1.79 / SWERVE_WHEEL_CIRCUMFERENCE / SWERVE_DRIVE_GEAR_RATIO  # 1 / 5
SWERVE_DRIVE_KA = 0.25 / SWERVE_WHEEL_CIRCUMFERENCE / SWERVE_DRIVE_GEAR_RATIO

# Module Front Left

SWERVE_ANGLE_OFFSET_FRONT_LEFT = Rotation2d.fromDegrees(-153.36914)  # 234.4
SWERVE_DRIVE_MOTOR_ID_FRONT_LEFT = 1
SWERVE_ANGLE_MOTOR_ID_FRONT_LEFT = 2
SWERVE_CANCODER_ID_FRONT_LEFT = 9  # 8
SWERVE_DRIVE_INVERT_FRONT_LEFT = signals.InvertedValue(1)
SWERVE_ANGLE_INVERT_FRONT_LEFT = signals.InvertedValue(1)


# Module Front Right

SWERVE_ANGLE_OFFSET_FRONT_RIGHT = Rotation2d.fromDegrees(107.13867)  # 235.37
SWERVE_DRIVE_MOTOR_ID_FRONT_RIGHT = 3
SWERVE_ANGLE_MOTOR_ID_FRONT_RIGHT = 4
SWERVE_CANCODER_ID_FRONT_RIGHT = 10  # 5
SWERVE_DRIVE_INVERT_FRONT_RIGHT = signals.InvertedValue(0)
SWERVE_ANGLE_INVERT_FRONT_RIGHT = signals.InvertedValue(1)


# Module Back Right

SWERVE_ANGLE_OFFSET_BACK_RIGHT = Rotation2d.fromDegrees(-23.4668)  # 5.80
SWERVE_DRIVE_MOTOR_ID_BACK_RIGHT = 5
SWERVE_ANGLE_MOTOR_ID_BACK_RIGHT = 6
SWERVE_CANCODER_ID_BACK_RIGHT = 11  # 2
SWERVE_DRIVE_INVERT_BACK_RIGHT = signals.InvertedValue(0)
SWERVE_ANGLE_INVERT_BACK_RIGHT = signals.InvertedValue(1)

# Module Back Left
SWERVE_ANGLE_OFFSET_BACK_LEFT = Rotation2d.fromDegrees(149.765625)  # 339.96
SWERVE_DRIVE_MOTOR_ID_BACK_LEFT = 7
SWERVE_ANGLE_MOTOR_ID_BACK_LEFT = 8
SWERVE_CANCODER_ID_BACK_LEFT = 12  # 11
SWERVE_DRIVE_INVERT_BACK_LEFT = signals.InvertedValue(0)
SWERVE_ANGLE_INVERT_BACK_LEFT = signals.InvertedValue(1)

# Other


SWERVE_KINEMATICS = SwerveDrive4Kinematics(
    Translation2d(
        DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0
    ),
    Translation2d(
        DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0
    ),
    Translation2d(
        -DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0
    ),
    Translation2d(
        -DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0
    ),
)

SWERVE_PIGEON_ID = 13  # 12

SWERVE_INVERT_GYRO = False
SWERVE_INVERT_CANCODERS = signals.SensorDirectionValue(False)

# CAN_PDH = 13

LIMELIGHT_HEIGHT_METERS = 0.381
LIMELIGHT_OFFSET_ANGLE_DEG = -25

FIELD_LENGTH_METERS = 17.55
FIELD_WIDTH_METERS = 8.05

# Auto
AUTO_RESOLUTION = 0.02  # path resolution in seconds
MAX_VEL_METERS_AUTO = 6.5  # This is the max velocity you want the robot to drive at, not its true max velocity
MAX_ANG_VEL_RAD_AUTO = MAX_VEL_METERS_AUTO / math.hypot(
    DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0
)  # This is the max velocity you want the robot to rotate at, not its true max rotational velocity
MAX_ACCEL_AUTO = 3  # This is the max rate you want the robot to accelerate at, not its true max acceleration
MAX_ANG_ACCEL_AUTO = (
    6 * math.pi
)  # This is the max rate you want the robot to accelerate at, not its true max acceleration
X_KP = 2  # 8 # 5  # 0.12667925	#0.73225
X_KI = 0.05  # 0.015  # 0.015 #0.0346173		#0.2001
X_KD = 0.2  # 0.01165449	#0.067367

Y_KP = X_KP
Y_KI = X_KI
Y_KD = X_KD
THETA_KP = 0.2  # 1.9 #0.232  # * 2.866 * 5.0
THETA_KI = 0.0  # 0.07  # 0.0625  # * 2.866 * 5.0
THETA_KD = 0.001  # 0.01

FUNNEL_CANRANGE = 14
FUNNEL_INTAKE_MOTOR_CAN_ID = 15

ELEVATOR_MOTOR_1_CAN_ID = 16
ELEVATOR_MOTOR_2_CAN_ID = 17

END_EFFECTOR_MOTOR_CAN_ID = 18
END_EFFECTOR_CANRANGE_ID = 19
END_EFFECTOR_OUTTAKE_MOTOR_CAN_ID = 20

CLIMBER_ARM_MOTOR_CAN_ID = 21
END_EFFECTOR_REEF_ALIGNMENT_CANRANGE = 22

COLLISION_JERK_MAX = 50
SKIDDING_RATIO_MAX = 10

CAM_DICT = {
    "camera_1": (
        np.array([0.042, -0.016, -0.001, 0.0, -0.092, -0.001, 0.002, 0.004], dtype=np.float64),
        np.array([[912.10, 0.0, 652.91],
			[0.0, 911.16, 428.47],
			[0.0, 0.0, 1.0]], dtype=np.float64),
    ),
    "camera_2": (
        np.array([0.041, -0.044, 0.0, -0.001, -0.026, -0.002, 0.002, 0.001], dtype=np.float64),
        np.array([[915.31, 0.0, 631.37],
			[0.0, 915.77, 396.59],
			[0.0, 0.0, 1.0]], dtype=np.float64),
    ),
	"camera_3": (
        np.array([0.05, -0.07, 0.0, -0.001, 0.002, -0.002, 0.005, 0.0], dtype=np.float64),
        np.array([[909.0, 0.0, 678.48],
			[0.0, 908.51, 428.25],
			[0.0, 0.0, 1.0]], dtype=np.float64),
    ),
	"camera_4": (
        np.array([0.048, -0.06, -0.001, -0.001, -0.014, -0.002, 0.002, 0.001], dtype=np.float64),
        np.array([[912.94, 0.0, 629.76],
			[0.0, 913.40, 390.51],
			[0.0, 0.0, 1.0]], dtype=np.float64),
    )
}

ELEVATOR_RAISE_SPEED = 0.5

JENNY = 8675_309999999
