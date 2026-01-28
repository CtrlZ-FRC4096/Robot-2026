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
from wpilib import Timer, DataLogManager, DriverStation, Field2d
import wpilib.sysid
import wpimath.geometry
import const
import oi
import math
import ntcore
import subsystems.drivetrain
from wpimath.units import inchesToMeters

from wpimath.estimator import SwerveDrive4PoseEstimator


from phoenix6 import controls

# import subsystems.limelight
import subsystems.leds

import subsystems.limelight
import subsystems.poseEstimator

from wpilibextra.coroutine.coroutine_robot import CoroutineRobot
from wpilibextra.remote_shell import RemoteShell
from coroutines import Coroutines

import inspect
import autoroutines

from pathplannerlib.path import PathPlannerPath, Waypoint, IdealStartingState, GoalEndState, PathPoint
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto, NamedCommands, FollowPathCommand, PathConstraints
from pathplannerlib.config import PIDConstants, RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController

from wpimath.geometry import Rotation2d, Pose2d, Translation2d, Pose3d, Rotation3d, Transform3d
from wpimath.units import degreesToRadians

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
from fuel_sim import FuelSim


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

        ## SIMMING STUFF ##
        # const.IS_SIMULATION = self.isSimulation()
        self.fuel_sim = FuelSim()

        # Command scheduler
        self.scheduler = CommandScheduler.getInstance()

        self.previously_scored = True
        self.has_coral = True
        
        # subsystems
        self.leds = subsystems.leds.LEDs(self)
        self.poseEstimator = subsystems.poseEstimator.PoseEstimator(self)
        self.drivetrain = subsystems.drivetrain.Drivetrain(self)

        self.subsystems = [
            self.drivetrain,
            self.leds,
            self.poseEstimator,
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

		### STATE MACHINE ###

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

        while True:
            yield
            self.scheduler.run()



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

        if self.fieldConstants.shouldFlip:
            self.poseEstimator.set_yaw(90)
        else:
            self.poseEstimator.set_yaw(270)

        # self.scheduler.schedule(self.auto)

    ### TELEOPERATED ###
    def teleop_mode(self):
        self.scheduler.cancelAll()
        self.running_pid_lineup = False
        self.in_autonomous_mode = False
        self.oi.robot_oriented_angle = self.poseEstimator.getYaw().degrees()

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
        wpilib.SmartDashboard.putNumberArray("empty pose", [0, 0, 0])
        wpilib.SmartDashboard.putBoolean("Connected to FMS", self.driverstation.isFMSAttached())

        if self.isSimulation():
            wpilib.SmartDashboard.putNumberArray("RobotPose", [self.poseEstimator.curEstPose.X(), self.poseEstimator.curEstPose.Y(), self.poseEstimator.curEstPose.rotation().degrees()])
            self.fuel_sim.update

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
