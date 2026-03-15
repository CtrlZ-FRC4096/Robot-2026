# from typing import TYPE_CHECKING

# if TYPE_CHECKING:
#     from robot import Robot

# import wpilib
# import wpimath.controller
# from commands2 import Subsystem
# import math
# from wpimath.trajectory import TrapezoidProfile
# from wpimath.geometry import Pose2d, Rotation2d

# import const
# from wpilib import Timer

# from phoenix6 import controls, configs, hardware, signals


# # MOST OF THE METHODS IN THIS CLASS WILL NOT WORK RN
# class Climber(Subsystem):
#     def __init__(self, robot: "Robot"):
#         super().__init__()
#         self.robot = robot
#         self.climber_motor = hardware.TalonFX(const.CLIMBER_MOTOR_ID, "rio")
#         self.climber_motor_config = self.robot.get_motor_config(0, 10, 0, 0, 0, 0, 0, 0)
#         self.climber_motor_config.motion_magic.motion_magic_cruise_velocity = 100
#         self.climber_motor_config.motion_magic.motion_magic_acceleration = 100
#         self.climber_motor.configurator.apply(self.climber_motor_config) 

#         self.commanded_climber_position = 0.0

#         self.request = controls.MotionMagicTorqueCurrentFOC(0.0)

#         self.test_climber_up_position = 32.5
#         self.test_climber_down_position = 10
#         self.climber_is_up = False

#     def get_position(self):
#         if self.robot.isSimulation():
#             return self.commanded_climber_position
#         else:
#             return self.climber_motor.get_position().value # DO GEAR RATIOS AND STUFF

#     def set_position(self, position):
#         self.commanded_climber_position = position
#         rotations = position # DO INVERSE GEAR RATIOS AND stuff
#         if abs(position - self.get_position()) >= 0.5:
#             self.climber_motor.set_control(self.request.with_position(rotations))
#         else:
#             self.climber_motor.set_control(controls.StaticBrake())

#     def stop(self):
#         # self.climber_motor.set_control(controls.MotionMagicTorqueCurrentFOC(0.0))
#         self.climber_motor.set_control(controls.StaticBrake())

#     def periodic(self):
#         if self.robot.is_climbing:
#             if self.robot.poseEstimator.cur_pos_in_zone() or True:
#                 if not ((abs(self.get_position() - self.test_climber_up_position) <= 1 or self.climber_is_up) and self.robot.at_climbing_position):
#                     self.set_position(self.test_climber_up_position)
#                     # self.robot.final_lineup_pose = Pose2d(1,1, Rotation2d())
#                     # self.robot.running_pid_lineup = True
#                 else:
#                     self.climber_is_up = True
#                     if abs(self.get_position() - self.test_climber_down_position) >= 1:
#                         self.set_position(self.test_climber_down_position)
#                     else:
#                         self.stop()

#             else:
#                 self.set_position(0)
#                 self.stop()
#         else:
#             self.set_position(0)

#     def log(self):
#         wpilib.SmartDashboard.putNumber("Climber/Actual Position", self.climber_motor.get_position().value)
#         wpilib.SmartDashboard.putNumber("Climber/Commanded Position", self.commanded_climber_position)
#         wpilib.SmartDashboard.putBoolean("Climber/Climber is up", self.climber_is_up)
