#! python3
"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2024
Code for robot "swerve drivetrain prototype"
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

DEBUG = True

import logging

# Import our files


from commands2 import (
    Command,
    ParallelCommandGroup,
    ParallelRaceGroup,
    SequentialCommandGroup,
    CommandScheduler,
)
import wpilib
from wpilib import Timer, DataLogManager, DriverStation, Field2d, SmartDashboard
import wpilib.sysid
import wpimath.geometry
import const
import oi
import math
import ntcore
import subsystems.drivetrain
from wpimath.units import inchesToMeters

from wpimath.estimator import SwerveDrive4PoseEstimator


from phoenix6 import controls, signals, configs

# import subsystems.limelight
import subsystems.leds

import subsystems.limelight
import subsystems.poseEstimator
import subsystems.intake
from subsystems import shooter, hopper

from wpilibextra.coroutine.coroutine_robot import CoroutineRobot
from wpilibextra.remote_shell import RemoteShell

from pykit.logger import Logger
from pykit.networktables.nt4Publisher import NT4Publisher
from pykit.wpilog.wpilogwriter import WPILOGWriter

from coroutines import Coroutines

import inspect
import autoroutines

from pathplannerlib.path import PathPlannerPath, Waypoint, IdealStartingState, GoalEndState, PathPoint
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto, NamedCommands, FollowPathCommand, PathConstraints
from pathplannerlib.config import PIDConstants, RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController

from wpimath.geometry import Rotation2d, Pose2d, Translation2d, Pose3d, Rotation3d, Transform3d, Translation3d, Twist2d
from wpimath.units import degreesToRadians, radiansToDegrees

from field_const import FieldConstants

from commands2 import (
    Command,
    ParallelCommandGroup,
    ParallelRaceGroup,
    SequentialCommandGroup,
)
import time
from wpilibextra.coroutine import CoroutineCommand
from wpilib import SmartDashboard



log = logging.getLogger("robot")


class Robot(CoroutineRobot):
    """
    Main robot class.

    This is the central object, holding instances of all the robot subsystem
    and sensor classes.

    It also contains the methods for autonomous and
    teloperated modes, called during mode changes and repeatedly when those
    modes are active.

    The one instance of this class is also passed as an argument to the
    various other classes, so they have full access to all its properties.
    """

    def robot_start(self):
        # Initialize PyKit Logger
        Logger.recordMetadata("Project", "Robot-2026")
        if self.isSimulation():
            Logger.addDataReciever(NT4Publisher(True))
        else:
            Logger.addDataReciever(NT4Publisher(False))
            Logger.addDataReciever(WPILOGWriter())
        Logger.start()
        
        # Networktables
        nt_inst = ntcore.NetworkTableInstance.getDefault()
        nt_inst.startServer()
        self.nt_robot = nt_inst.getTable("SmartDashboard")
        # self.nt_robot.putString('led_mode', 'off')
        
        ### ARE WE USING AN FMS? (EX: IF WE ARE AT COMPETITION) ###
        self.using_FMS = False
        ## ARE WE RUNNING AN AUTO? (DO WE NEED TO WAIT TO SELECT AN AUTO BEFORE INIT)
        self.using_auto = False

        # DRIVERSTATION #
        self.driverstation = wpilib.DriverStation

        if self.using_FMS:
            while not self.driverstation.isFMSAttached():
                # time.sleep(1.0) # Wait until the FMS is connected to the Driver Station
                yield from self.wait(1.0)
                if self.driverstation.isFMSAttached():
                    break
            # time.sleep(1.0) # Give enough time to make sure the FMS has told the Driver Station the Alliance 
        self.fieldConstants = FieldConstants()
        self.fieldConstants.shouldFlip = DriverStation.getAlliance() == DriverStation.Alliance.kRed
        # Match Stuff
        self.match_time = -1

        # Command scheduler
        self.scheduler = CommandScheduler.getInstance()

        self.previously_scored = True
        self.has_coral = True
        
        # subsystems
        self.leds = subsystems.leds.LEDs(self)
        self.poseEstimator = subsystems.poseEstimator.PoseEstimator(self)
        self.drivetrain = subsystems.drivetrain.Drivetrain(self)
        self.intake = subsystems.intake.Intake(self)
        self.shooter = shooter.Shooter(self)
        self.hopper = hopper.Hopper(self)

        self.subsystems = [
            self.drivetrain,
            self.leds,
            self.poseEstimator,
            self.intake,
            self.shooter,
            self.hopper,
        ]

        # If everything in self.subsystems is a Subsystem object, then
        # everything is automatically registered and this isn't needed.
        for subsystem in self.subsystems:
            self.scheduler.registerSubsystem(subsystem)

        # Coroutines
        self.coroutines = Coroutines(self)
        # With V's wrapper for commandify we automatically register all commands

        ### OTHER ###
        
        self.oi = oi.OI(self)

		

        # self.pathplanner_config = RobotConfig.fromGUISettings()

        self.match_time = -1

        ### FIELD LOGGING ###
        self.field = Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)
        ### LOGGING ###
        self.remote_shell = RemoteShell(self)

		# PATH CONSTRAINTS
        # self.path_constraints = PathConstraints(4.0, 4.0, degreesToRadians(540), degreesToRadians(540))
        self.autoroutines = autoroutines.AutoRoutines(self)


        DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog())

        ### STATE MACHINE VARIABLES ###
        self.running_pid_lineup = False
        self.final_lineup_pose = Pose2d()
        self.mechanisms_at_default = True 
        self.trench = True

        self.shoot_fuel = False
        self.shoot_intent = False
        self.spin_up = False
        self.is_climbing = False
        self.is_intaking = False
        self.pulse_indexer = False

        self.snake_intake = False

        self.virtual_target = self.poseEstimator.field.getObject("Virtual Target")

        self.timer = Timer()

        log_refresh_rate = 0.02 if self.isSimulation() else 0.25
        @self.addPeriodic(period=log_refresh_rate, offset=0)
        def _():
            self.log()
            pass

        @self.addPeriodic(period=0.05, offset=-0.01)
        def _leds():
            self.leds.periodicX()
            pass

        self.in_autonomous_mode = False
        self.in_teleop_mode = False

        ## SIMMING STUFF ##
        if self.isSimulation():
            self.max_fuel_in_hopper = 24
            self.x_hopper_max = inchesToMeters(25)
            self.y_hopper_max = inchesToMeters(18)
            self.z_hopper_max = inchesToMeters(15)
            self.fuel_in_hopper = 0
            self.tick_count = 0

        while True:
            yield
            self.scheduler.run()

    def get_motor_config(self, inverted=0, k_p=0.0, k_i=0.0, k_d=0.0, k_v=0.0, k_a=0.0, k_g=0.0, k_s=0.0):
        motor_config = configs.TalonFXConfiguration()
        motor_config.motor_output.inverted = signals.InvertedValue(inverted)
        motor_config.current_limits.stator_current_limit = 100
        motor_config.current_limits.supply_current_limit_enable = True

        motor_config.slot0.k_p = k_p
        motor_config.slot0.k_i = k_i
        motor_config.slot0.k_d = k_d
        motor_config.slot0.k_v = k_v
        motor_config.slot0.k_a = k_a
        motor_config.slot0.k_g = k_g
        motor_config.slot0.k_s = k_s

        motor_config.closed_loop_ramps.torque_closed_loop_ramp_period = 0.02
        motor_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
        motor_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = 0.02
        motor_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 0.02
        motor_config.closed_loop_ramps.voltage_closed_loop_ramp_period = 0.02
        motor_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02

        motor_config.current_limits.supply_current_limit = 80
        motor_config.torque_current.peak_forward_torque_current = 80
        motor_config.torque_current.peak_reverse_torque_current = -80

        return motor_config


    ### DISABLED ###

    def disabled_mode(self):
        if self.using_FMS:
            while not self.driverstation.isFMSAttached():
                yield
            yield from self.wait(2.0)

        self.scheduler.cancelAll()
        # self.drivetrain.gyro_offset = self.drivetrain.gyro.get_roll()

        for subsystem in self.subsystems:
            subsystem.stop()

        while True:  # Needs to continuously call while robot is disabled.
            yield

    ### AUTONOMOUS ###
    def autonomous_mode(self):

        self.has_coral = True # Start with preloaded coral
        self.scheduler.cancelAll()
        self.in_autonomous_mode = True

        # if self.isSimulation():
        #     self.fuel_sim.start()

        # self.scheduler.schedule(self.auto)

    ### TELEOPERATED ###
    def teleop_mode(self):
        self.scheduler.cancelAll()
        self.running_pid_lineup = False
        self.in_autonomous_mode = False
        self.oi.robot_oriented_angle = self.poseEstimator.getYaw().degrees()
        self.in_teleop_mode = True
        if self.isSimulation():
            from fuel_sim import FuelSim
            self.fuel_sim = FuelSim(self, self.intake.can_intake_sim, self.intake.intake_sim_callback)
            self.fuel_sim.start()
        self.timer.start()

        while True:
            yield

    ### WAIT FUNCTION ###
    def wait(self, time):
        timer = Timer()
        timer.start()
        while not timer.hasElapsed(time):
            yield

    def log(self):
        """
        Logs some info to shuffleboard, and standard output
        """
        wpilib.SmartDashboard.putBoolean("Has Coral", self.has_coral)
        SmartDashboard.putNumberArray("Empty Pose", [0,0,0,1,0,0,0])
        wpilib.SmartDashboard.putBoolean("Connected to FMS", self.driverstation.isFMSAttached())
        SmartDashboard.putBoolean("States/Running Pid Lineup", self.running_pid_lineup)
        SmartDashboard.putBoolean("States/Mechanisms at Default", self.mechanisms_at_default)


        if self.isSimulation():
            wpilib.SmartDashboard.putNumberArray("RobotPose", [self.poseEstimator.curEstPose.X(), self.poseEstimator.curEstPose.Y(), self.poseEstimator.curEstPose.rotation().degrees()])
            # SmartDashboard.putNumberArray("ZeroedComponentPoses/Pose0", [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
            # SmartDashboard.putNumberArray("ZeroedComponentPoses/Pose1", [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
            # SmartDashboard.putNumberArray("ZeroedComponentPoses/Pose2", [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
            

            default_shooter_hood = Translation3d(-0.23, 0.15, 0.5)
            cur_hood_pos = self.shooter.get_hood_position()
            final_shooter_hood_quat = Rotation3d(degreesToRadians(cur_hood_pos), 0, 0).getQuaternion()
            final_shooter_hood_trans = default_shooter_hood
            SmartDashboard.putNumberArray("FinalComponentPoses/Pose0", [final_shooter_hood_trans.X(), final_shooter_hood_trans.Y(), final_shooter_hood_trans.Z(), final_shooter_hood_quat.W(), final_shooter_hood_quat.X(), final_shooter_hood_quat.Y(), final_shooter_hood_quat.Z()])
            

            default_inner = Translation3d(0.3, 0.355, 0.2)
            cur_inner_pos = self.intake.get_position()
            final_inner_quat = Rotation3d(0, degreesToRadians(cur_inner_pos), 0).getQuaternion()
            final_inner_trans = default_inner
            SmartDashboard.putNumberArray("FinalComponentPoses/Pose1", [final_inner_trans.X(), final_inner_trans.Y(), final_inner_trans.Z(), final_inner_quat.W(), final_inner_quat.X(), final_inner_quat.Y(), final_inner_quat.Z()])
            
            default_outer = Translation3d(0.2825, 0.32, 0.505)
            arc_vec = Translation2d(0.307975, 0).rotateBy(Rotation2d.fromDegrees(cur_inner_pos))
            inner_outer_transform = Translation3d(arc_vec.Y(),
                                                  0,
                                                  arc_vec.X()) - Translation3d(0, 0, 0.307975) 
            final_outer_quat = Rotation3d(0, degreesToRadians(-cur_inner_pos / 6.43), 0).getQuaternion()
            final_outer_trans = default_outer + inner_outer_transform
            SmartDashboard.putNumberArray("FinalComponentPoses/Pose2", [final_outer_trans.X(), final_outer_trans.Y(), final_outer_trans.Z(), final_outer_quat.W(), final_outer_quat.X(), final_outer_quat.Y(), final_outer_quat.Z()])
            
            default_fuel_pose = Translation3d(-0.26, -0.28, 0.28)
            for fuel_num in range(1,self.max_fuel_in_hopper + 1):
                if fuel_num <= self.fuel_in_hopper:
                    #put the fuel in
                    fuel_diam = self.fieldConstants.fuelDiameter
                    per_x = int(self.x_hopper_max / fuel_diam)
                    per_y = int(self.y_hopper_max / fuel_diam)
                    if per_x < 1: per_x = 1
                    if per_y < 1: per_y = 1

                    idx = fuel_num - 1
                    x_coord = (idx % per_x) * fuel_diam
                    y_coord = ((idx // per_x) % per_y) * fuel_diam
                    z_coord = (idx // (per_x * per_y)) * fuel_diam + 0.5
                    SmartDashboard.putNumberArray(f"Hopper/Sim Fuels/Fuel {fuel_num}", [default_fuel_pose.X() + x_coord, default_fuel_pose.Y() + y_coord, default_fuel_pose.Z() + z_coord, 1.0, 0.0, 0.0, 0.0])
                else:
                    SmartDashboard.putNumberArray(f"Hopper/Sim Fuels/Fuel {fuel_num}", [0, 0, -0.5, 1.0, 0.0, 0.0, 0.0])

            fly_speed = self.shooter.get_fly_speed()
            accel_speed = self.shooter.get_accelerator_speed()
            if fly_speed >= 5 and accel_speed >= 5 and self.fuel_in_hopper > 0:
                #we are shooting every 0.06 seconds
                if self.tick_count % 3 == 0:
                    launch_vel = self.shooter.fly_speed_to_launch_vel(fly_speed)

                    trans = Translation3d(0, 0.27, 0.52) + Translation3d(0, -0.11, 0) + Translation3d(0, 0.11* math.cos(degreesToRadians(cur_hood_pos)), 0.11*math.sin(degreesToRadians(cur_hood_pos)))
                    launch_pos = Translation3d(self.poseEstimator.curEstPose.translation()) + trans.rotateBy(Rotation3d(0, 0, self.poseEstimator.curEstPose.rotation().radians()))
                    self.fuel_sim.launchFuel(launch_vel, cur_hood_pos, 0, launch_pos)
                    self.fuel_in_hopper -= 1

            if self.in_teleop_mode:
                self.fuel_sim.updateSim()
            SmartDashboard.putNumber("Sim/Fuel in Hopper", self.fuel_in_hopper)
            self.tick_count += 1
        for s in self.subsystems:
            s.log()

        self.match_time = self.timer.get()
        wpilib.SmartDashboard.putNumber("Match Time", self.match_time)
        wpilib.SmartDashboard.putNumber(
            "robot oriented angle", self.oi.robot_oriented_angle
        )


### MAIN ###

if __name__ == "__main__":
    # wpilib.run(Robot) go to \Robot-2023\robot folder and run "py -m robotpy run"
    pass
