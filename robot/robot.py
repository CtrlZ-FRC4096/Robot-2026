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
from path import Path, DefaultGlobalConstraints, Waypoint, TranslationTarget, RotationTarget, PathConstraints, RangedConstraint
import wpilib
from wpimath.controller import PIDController
from wpilib import Timer, DataLogManager, DriverStation, Field2d, SmartDashboard
from wpilib.simulation import DriverStationSim
import wpilib.sysid
import wpimath.geometry
import const
import oi
import math
import ntcore
import subsystems.drivetrain
from wpimath.units import inchesToMeters

from wpimath.estimator import SwerveDrive4PoseEstimator
from wpilib.interfaces import GenericHID

from phoenix6 import controls, signals, configs
from bline_json import JsonUtils

# import subsystems.limelight
import subsystems.leds

import subsystems.limelight
import subsystems.poseEstimator
import subsystems.intake
from subsystems import shooter, hopper, climber

from wpilibextra.coroutine.coroutine_robot import CoroutineRobot
from wpilibextra.remote_shell import RemoteShell

from pykit.logger import Logger
from pykit.networktables.nt4Publisher import NT4Publisher
from pykit.wpilog.wpilogwriter import WPILOGWriter

from coroutines import Coroutines

import inspect
import autoroutines

from pathplannerlib.path import PathPlannerPath, Waypoint, IdealStartingState, GoalEndState, PathPoint
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto, NamedCommands, FollowPathCommand#, PathConstraints
from pathplannerlib.config import PIDConstants, RobotConfig, ModuleConfig
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
from pathplannerlib.controller import PathFollowingController, PPHolonomicDriveController
from pathplannerlib.path import DriveFeedforwards

from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from bline_command import BLineCommand, Builder

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
        
        ## ARE WE RUNNING AN AUTO? (DO WE NEED TO WAIT TO SELECT AN AUTO BEFORE INIT)
        self.using_auto = True

        # DRIVERSTATION #
        self.driverstation = wpilib.DriverStation

        self.fieldConstants = FieldConstants()
        # self.fieldConstants.shouldFlip = DriverStation.getAlliance() == DriverStation.Alliance.kRed # false = BLUE, true = RED
        # Match Stuff
        self.match_time = -1

        # Command scheduler
        self.scheduler = CommandScheduler.getInstance()
        
        # subsystems
        # self.leds = subsystems.leds.LEDs(self)
        self.poseEstimator = subsystems.poseEstimator.PoseEstimator(self)
        self.drivetrain = subsystems.drivetrain.Drivetrain(self)
        self.intake = subsystems.intake.Intake(self)
        self.shooter = shooter.Shooter(self)
        self.hopper = hopper.Hopper(self)
        # self.climber = climber.Climber(self)

        self.subsystems = [
            self.drivetrain,
            # self.leds,
            self.poseEstimator,
            self.intake,
            self.shooter,
            self.hopper,
            # self.climber
        ]

        # If everything in self.subsystems is a Subsystem object, then
        # everything is automatically registered and this isn't needed.
        for subsystem in self.subsystems:
            self.scheduler.registerSubsystem(subsystem)

        # Coroutines
        self.coroutines = Coroutines(self)
        # With V's wrapper for commandify we automatically register all commands

        ### OTHER ###
        self.rumble_d1 = False
        self.rumble_d2 = False
        self.oi = oi.OI(self)

		

        # self.pathplanner_config = RobotConfig.fromGUISettings()

        ### FIELD LOGGING ###
        self.field = Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)
        ### LOGGING ###
        self.remote_shell = RemoteShell(self)

		# PATHS
        self.P2_B_L = self.getPathCommand(PathPlannerPath.fromPathFile("P2_B_L"))
        self.P2_B_R = self.getPathCommand(PathPlannerPath.fromPathFile("P2_B_R"))

        self.P1_T_B_R_ROBUST = self.getPathCommand(PathPlannerPath.fromPathFile("P1_T_B_R_Robust"))
        self.P1_T_B_L_ROBUST = self.getPathCommand(PathPlannerPath.fromPathFile("P1_T_B_L_Robust"))

        self.P2_B_R_NEW = self.getPathCommand(PathPlannerPath.fromPathFile("P2_B_R_New")) 
        self.P2_B_L_NEW = self.getPathCommand(PathPlannerPath.fromPathFile("P2_B_L_New"))


        self.bline_translation_controller = PIDController(3, 0, 0)
        self.bline_rotation_controller = PIDController(0, 0, 0)
        self.bline_cross_track_controller = PIDController(0, 0, 0)
        self.bline_builder = Builder(self.drivetrain, 
                                     self.drivetrain.get_pose,
                                     self.drivetrain.get_robot_relative_speeds,
                                     self.drivetrain.drive_robot_relative,
                                     self.drivetrain.get_timestamp,
                                     self.bline_translation_controller,
                                     self.bline_rotation_controller,
                                     self.bline_cross_track_controller,
                                     True,
                                     self.drivetrain.should_flip_path,
                                     self.drivetrain.should_mirror_path)

        # self.bline_path_1 = self.get_bline_path_command(Path([TranslationTarget(Translation2d(6.0, 0.6), 0.5), TranslationTarget(Translation2d(7.0, 4.0), 0.5)]))
        BLineCommand.event_trigger_registry = {
            "stop_intake" : self.drivetrain.stop_intaking
        }
        
        self.P1_T_B_R_ROBUST_BL = self.get_bline_path_command(JsonUtils.load_path("P1_T_B_R_Robust_BL"))
        self.P2_B_R_NEW_BL = self.get_bline_path_command(JsonUtils.load_path("P2_B_R_New_BL"))
        self.SLOW_RIGHT_STEAL_OUT_BL = self.get_bline_path_command(JsonUtils.load_path("SLOW_RIGHT_STEAL_OUT"))
        self.SLOW_RIGHT_STEAL_BL = self.get_bline_path_command(JsonUtils.load_path("SLOW_RIGHT_STEAL"))
        self.SLOW_RIGHT_STEAL_BACK_BL = self.get_bline_path_command(JsonUtils.load_path("SLOW_RIGHT_STEAL_BACK"))
        self.SLOW_RIGHT_STEAL_TRENCH_BL = self.get_bline_path_command(JsonUtils.load_path("SLOW_RIGHT_STEAL_TRENCH"))
        self.SLOW_RIGHT_SAFE_BL = self.get_bline_path_command(JsonUtils.load_path("SLOW_RIGHT_SAFE"))

        self.SLOW_LEFT_STEAL_OUT_BL = self.get_bline_path_command(JsonUtils.load_path("SLOW_LEFT_STEAL_OUT"))
        self.SLOW_LEFT_STEAL_BL = self.get_bline_path_command(JsonUtils.load_path("SLOW_LEFT_STEAL"))
        self.SLOW_LEFT_STEAL_DEPOT_BL = self.get_bline_path_command(JsonUtils.load_path("SLOW_LEFT_STEAL_DEPOT"))
        self.SLOW_LEFT_STEAL_TRENCH_BL = self.get_bline_path_command(JsonUtils.load_path("SLOW_LEFT_STEAL_TRENCH"))

        self.DEFAULT_SLOW_DEPOT_BL = self.get_bline_path_command(JsonUtils.load_path("DEFAULT_SLOW_DEPOT"))

        self.new_path = self.get_bline_path_command(JsonUtils.load_path("new_path"))

        self.mirror_bline_auto = False

        self.autoroutines = autoroutines.AutoRoutines(self)
        self.auto = None

        self.auto_chooser = wpilib.SendableChooser()
        self.auto_chooser.addOption("Left Trench & Bump PP", 1)
        self.auto_chooser.addOption("Right Trench & Bump PP", 2)
        self.auto_chooser.addOption("Right Trench & Bump 3-Bot Safe BL", 3)
        self.auto_chooser.setDefaultOption("Default (no auto)", 0)

        SmartDashboard.putNumber("Submit Auto? (and FMS Connected)", int(0))
        

        DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog())

        ### STATE MACHINE VARIABLES ###
        self.running_pid_lineup = False
        self.final_lineup_pose = Pose2d()
        self.intake_at_default = True
        self.shooter_at_default = True
        self.trench = True
        self.down_bad = False
        self.spin_down = False

        self.static_target = Translation2d()

        self.at_climbing_position = False

        self.shoot_fuel = False
        self.shoot_intent = False
        self.is_intaking = False
        self.pulse_indexer = False
        self.pulse_pivot = False
        self.clear_jam = False
        self.ignore_shooter_in_jam = False

        self.robot_oriented_angle = self.poseEstimator.getYaw().degrees()
        
        # AUTO FLAGS
        self.run_p1 = False
        self.done_p1 = False

        self.run_p2 = False
        self.done_p2 = False

        self.run_p3 = False
        self.done_p3 = False

        self.run_p4 = False
        self.done_p4 = False


        self.snake_intake = False
        self.track_fuel = False

        self.done_rotation_auto = False

        self.did_autonomous = False
        self.auto_submitted = False

        #TESTING
        self.should_hub_track = False
        self.is_hub_active = True
        self.velocity_constrain_pid = False

        # SHOOTING VALUES
        self.time_of_flight = 1
        self.distance = 1
        self.fly_speed = 50
        self.hood_angle = 35
        self.virtual_goal = Translation2d()
        self.fuel_in_hopper = 8

        self.down_bad_fly_speed = 55 # TODO: TUNE
        self.down_bad_hood_angle = 40 # TODO: TUNE

        self.hood_fudge_value = 0

        self.virtual_target = self.poseEstimator.field.getObject("Virtual Target")

        self.timer = Timer()
        self.auto_winner_blue = None


        log_refresh_rate = 0.02 if self.isSimulation() else 0.25
        @self.addPeriodic(period=log_refresh_rate, offset=0)
        def _():
            self.log()
            pass

        # @self.addPeriodic(period=0.05, offset=-0.01)
        # def _leds():
        #     self.leds.periodicX()
        #     pass

        self.in_autonomous_mode = False
        self.in_teleop_mode = False

        # match timer & auto winner logic
        self.match_timer = Timer()
        self.alliance_shift = 0
        self.alliance_shift_time_remaining = 0
        self.auto_win = None  # false = BLUE, true = RED
        # self.auto_win_found = False

        
        if not self.using_auto:
            self.auto = self.autoroutines.right_trench_bump_robust_new()
            self.poseEstimator.poseEst.resetPose(Pose2d(self.fieldConstants.flip_Translation2d(Translation2d(4.47, 0.6)), self.poseEstimator.getYaw())) # for right auto
            # self.poseEstimator.poseEst.resetPose(Pose2d(self.fieldConstants.flip_Translation2d(Translation2d(4.471, 7.587)), self.poseEstimator.getYaw())) # for left auto


        ## SIMMING STUFF ##
        if self.isSimulation():
            self.max_fuel_in_hopper = 24
            self.x_hopper_max = inchesToMeters(25)
            self.y_hopper_max = inchesToMeters(18)
            self.z_hopper_max = inchesToMeters(15)
            
            self.tick_count = 0

            self.fuel_sim = FuelSim(self, self.intake.can_intake_sim, self.intake.intake_sim_callback)
            # self.fuel_sim.stop()
            # self.fuel_sim.clearFuel()
            self.fuel_sim.start()

       
        # test_path = self.flip_path_cmd_across_x(self.getPathCommand(PathPlannerPath.fromPathFile("P2_B_R")))._originalPath
        # test_path_waypoints = test_path.getWaypoints()
        # for idx, waypoint in enumerate(test_path_waypoints):
        #     if idx == 0:
        #         next_control_dist = (waypoint.anchor - waypoint.nextControl).norm()
        #         next_control_heading = Rotation2d((waypoint.nextControl - waypoint.anchor).X(), (waypoint.nextControl - waypoint.anchor).Y()).degrees()
        #         print(f"Start: anchor: {waypoint.anchor}, next_controldist: {next_control_dist}, next_control_head: {next_control_heading}")
        #     elif idx == len(test_path_waypoints) - 1:
        #         prev_control_dist = (waypoint.prevControl - waypoint.anchor).norm()
        #         prev_control_heading = Rotation2d((waypoint.anchor - waypoint.prevControl).X(), (waypoint.anchor - waypoint.prevControl).Y()).degrees()
        #         print(f"End: anchor: {waypoint.anchor}, prev_controldist: {prev_control_dist}, prev_control_head: {prev_control_heading}")
        #     else:
        #         next_control_dist = (waypoint.anchor - waypoint.nextControl).norm()
        #         next_control_heading = Rotation2d((waypoint.nextControl - waypoint.anchor).X(), (waypoint.nextControl - waypoint.anchor).Y()).degrees()
        #         prev_control_dist = (waypoint.prevControl - waypoint.anchor).norm()
        #         prev_control_heading = Rotation2d((waypoint.anchor - waypoint.prevControl).X(), (waypoint.anchor - waypoint.prevControl).Y()).degrees()
        #         print(f"{idx}: anchor: {waypoint.anchor}, heading: {prev_control_heading}, prevdist: {prev_control_dist}, next_controldist: {next_control_dist}")
        # print(test_path.getRotationTargets())

        # chassis = const.SWERVE_KINEMATICS.toChassisSpeeds(SwerveModuleState(-2.847, Rotation2d.fromDegrees(272.373)), SwerveModuleState(-2.668, Rotation2d.fromDegrees(268.330), SwerveModuleState(-2.282, Rotation2d.fromDegrees(268.737)), SwerveModuleState(1.220, Rotation2d.fromDegrees(66.530))))
        # print(chassis)

        while True:
            yield
            self.scheduler.run()
    
    
    def flip_X_coord(self, x):
        return self.fieldConstants.fieldLength - x
    def flip_Y_coord(self, y):
        return self.fieldConstants.fieldWidth - y

    def flip_path_cmd_across_x(self, cmd_path : FollowPathCommand):
        # FLIPPING Y COORDS
        path = cmd_path._originalPath
        waypoints = path.getWaypoints()
        new_waypoints = []
        ideal_start = IdealStartingState(path.getIdealStartingState().velocity, path.getIdealStartingState().rotation.__neg__())
        goal_end = GoalEndState(path.getGoalEndState().velocity, path.getGoalEndState().rotation.__neg__())
        for idx, waypoint in enumerate(waypoints):
            if idx == 0:
                # no prev control
                new_waypoints.append(Waypoint(
                    prevControl=None,
                    anchor=Translation2d(waypoint.anchor.X(), self.flip_Y_coord(waypoint.anchor.Y())),
                    nextControl=Translation2d(waypoint.nextControl.X(), self.flip_Y_coord(waypoint.nextControl.Y()))
                ))
            elif idx == len(waypoints) - 1:
                #no next control
                new_waypoints.append(Waypoint(
                    prevControl=Translation2d(waypoint.prevControl.X(), self.flip_Y_coord(waypoint.prevControl.Y())),
                    anchor=Translation2d(waypoint.anchor.X(), self.flip_Y_coord(waypoint.anchor.Y())),
                    nextControl=None
                ))
            else:
                        # both controls
                new_waypoints.append(Waypoint(
                    prevControl=Translation2d(waypoint.prevControl.X(), self.flip_Y_coord(waypoint.prevControl.Y())),
                        anchor=Translation2d(waypoint.anchor.X(), self.flip_Y_coord(waypoint.anchor.Y())),
                        nextControl=Translation2d(waypoint.nextControl.X(), self.flip_Y_coord(waypoint.nextControl.Y()))
                    ))
        new_path = PathPlannerPath(new_waypoints, path.getGlobalConstraints(), ideal_starting_state=ideal_start, goal_end_state=goal_end)
        return FollowPathCommand(
            new_path,
            self.drivetrain.get_pose, # Robot pose supplier
            self.drivetrain.get_robot_relative_speeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.drivetrain.drive_robot_relative, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
            PPHolonomicDriveController(  # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(
                    0.5, 0, 0
                ),  # Translation PID constants
                PIDConstants(
                    0.5, 0, 0.1
                ),  # Rotation PID constants
            ),
            RobotConfig.fromGUISettings(), # The robot configuration
            self.drivetrain.should_flip_path, # Supplier to control path flipping based on alliance color
            self.drivetrain # Reference to this subsystem to set requirements
        )

    def get_bline_path_command(self, path : Path):
        return self.bline_builder.build(path)

    def getPathCommand(self, path : PathPlannerPath):
        return FollowPathCommand(
            path,
            self.drivetrain.get_pose,
            self.drivetrain.get_robot_relative_speeds,
            self.drivetrain.drive_robot_relative,
            PPHolonomicDriveController(
                PIDConstants(
                    0.95, 0, 0.05 #1 , 0, 0.025
                ),  # Translation PID constants
                PIDConstants(
                    0.75, 0, 0.04 # 0.8, 0, 0.05
                ),  # Rotation PID constants
            ),
            RobotConfig.fromGUISettings(),
            self.drivetrain.should_flip_path,
            self.drivetrain
        ) 


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

        motor_config.current_limits.supply_current_limit = 40
        motor_config.torque_current.peak_forward_torque_current = 40
        motor_config.torque_current.peak_reverse_torque_current = -40

        return motor_config


    ### DISABLED ###

    def disabled_mode(self):

        self.scheduler.cancelAll()
        # self.drivetrain.gyro_offset = self.drivetrain.gyro.get_roll()

        for subsystem in self.subsystems:
            subsystem.stop()

        self.match_timer.stop()

        while True: # Needs to continuously call while robot is disabled.
            yield
    
    ### AUTONOMOUS ###

    def autonomous_mode(self):
        self.scheduler.cancelAll()
        
        self.in_teleop_mode = False
        self.did_autonomous = True
        self.in_autonomous_mode = True
        self.intake_at_default = False
        

        

        # if self.isSimulation():
        #     self.fuel_sim.running = True
        if self.auto is None:
            self.auto = SequentialCommandGroup()

        self.auto = self.autoroutines.test_path()

        self.scheduler.schedule(self.auto)

    def autonomousExit(self):
        for idx in range(len(self.poseEstimator.modules)):
            swerve_drive_motor_config = configs.TalonFXConfiguration()
            # self.drive_motor.configurator.apply(swerve_drive_motor_config)  # type: ignore
            swerve_drive_motor_config.slot0.k_p = 5  # 2.2
            swerve_drive_motor_config.slot0.k_s = 7
            swerve_drive_motor_config.slot0.k_v = 0.5  # 0.24
            ## Feed Forward
            # swerve_drive_motor_config.slot0.k_v = const.SWERVE_DRIVE_KV
            # swerve_drive_motor_config.slot0.k_a = const.SWERVE_DRIVE_KA
            swerve_drive_motor_config.current_limits.supply_current_limit = (
                60  # 80; I am not sure if this is correct
            )

            swerve_drive_motor_config.torque_current.peak_forward_torque_current = (
                60  # Set 60 to save battery; Up this to 80 for more zip
            )
            swerve_drive_motor_config.torque_current.peak_reverse_torque_current = (
                -60
            )
            ##Ramps
            swerve_drive_motor_config.closed_loop_ramps.torque_closed_loop_ramp_period = (
                0.02
            )
            swerve_drive_motor_config.open_loop_ramps.torque_open_loop_ramp_period = 0.02
            swerve_drive_motor_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = (
                0.02
            )
            swerve_drive_motor_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = (
                0.02
            )
            swerve_drive_motor_config.closed_loop_ramps.voltage_closed_loop_ramp_period = (
                0.02
            )
            swerve_drive_motor_config.open_loop_ramps.voltage_open_loop_ramp_period = 0.02

            swerve_drive_motor_config.current_limits.supply_current_limit_enable = True
            swerve_drive_motor_config.motor_output.inverted = (
                self.poseEstimator.modules[idx].drive_invert
            )  # signals.InvertedValue(1)  # This is no longer a boolean; 0 for CCW 1 for CW
            swerve_drive_motor_config.motor_output.neutral_mode = signals.NeutralModeValue(
                1
            )  # set to brake
            swerve_drive_motor_config.current_limits.stator_current_limit = 100

            self.poseEstimator.modules[idx].drive_motor.configurator.apply(swerve_drive_motor_config)

    ### TELEOPERATED ###
    def teleop_mode(self):
        self.scheduler.cancelAll()

        self.shoot_intent = False
        self.spin_down = False
        self.shooter_at_default = True
        self.shoot_fuel = False
        self.pulse_pivot = False
        self.shooter.shoot_ready = False
        self.shooter.accel_good = False

        self.running_pid_lineup = False
        self.in_autonomous_mode = False
        self.robot_oriented_angle = self.poseEstimator.curEstPose.rotation().degrees()
        self.in_teleop_mode = True
        self.timer.start()
        self.match_timer.reset()
        self.match_timer.start()

        while True:
            yield
    
    def hub_active(self):
        # if not self.driverstation.isFMSAttached():
        #     return True
        if self.alliance_shift == 0 or self.alliance_shift == 5:
            return True
        if self.auto_win is None:
            return None
        switch = (self.alliance_shift-1) % 2
        # shift is equal to [did this alliance win auto?]
        return switch == (self.auto_win == self.fieldConstants.shouldFlip)

    def update_hub_status(self):
        self.match_time = self.match_timer.get()
        self.is_hub_active = self.hub_active()

        if self.match_time <= 10:
            self.alliance_shift = 0
            self.alliance_shift_time_remaining = 11-self.match_time
        elif 10 < self.match_time <= 110:
            self.alliance_shift = ((self.match_time-10)//25)+1
            self.alliance_shift_time_remaining = 26-(self.match_time-10)%25
        else:
            self.alliance_shift = 5
            self.alliance_shift_time_remaining = 141-self.match_time

        if (2.5 < self.alliance_shift_time_remaining < 3) and not self.is_hub_active:
            self.rumble_d1 = True
            self.rumble_d2 = True


        if self.auto_win is None and self.match_time >= 4 and self.rumble_d2 == False:
            self.rumble_d2 = True
            

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
        # SmartDashboard.putString("Shooting Values/")
        # SmartDashboard.putNumber("Auto Currently Chosen", self.auto_chooser.getSelected())
        SmartDashboard.putNumberArray("Empty Pose", [0,0,0,1,0,0,0])
        SmartDashboard.putNumber("Battery Voltage", self.driverstation.getBatteryVoltage())
        SmartDashboard.putData("Auto Chooser", self.auto_chooser)
        SmartDashboard.putNumber("Current Auto Chosen", int(self.auto_chooser.getSelected()))
        SmartDashboard.putBoolean("Connected to FMS", self.driverstation.isFMSAttached())
        SmartDashboard.putBoolean("Auto Submitted", self.auto_submitted)
        SmartDashboard.putBoolean("States/Running Pid Lineup", self.running_pid_lineup)
        SmartDashboard.putBoolean("States/Intake at Default", self.intake_at_default)
        SmartDashboard.putBoolean("States/Shooter at Default", self.shooter_at_default)
        SmartDashboard.putBoolean("States/Should Hub Track", self.should_hub_track)
        SmartDashboard.putBoolean("States/Pulse Pivot", self.pulse_pivot)
        SmartDashboard.putBoolean("States/Down Bad", self.down_bad)
        SmartDashboard.putBoolean("States/Clear Jam", self.clear_jam)
        SmartDashboard.putBoolean("States/Spin Down", self.spin_down)

        SmartDashboard.putNumber("Shooting Values/Distance to Hub", self.distance)
        SmartDashboard.putNumber("Shooting Values/Time of Flight", self.time_of_flight)
        SmartDashboard.putNumber("Shooting Values/Hood Angle", self.hood_angle)
        SmartDashboard.putNumber("Shooting Values/Fly Speed", self.fly_speed)
        SmartDashboard.putNumber("Shooting Values/Hood Fudge Value", self.hood_fudge_value)
        SmartDashboard.putNumber("Shooting Values/Down Bad Fly Speed", self.down_bad_fly_speed)
        SmartDashboard.putNumber("Shooting Values/Down Bad Hood Angle", self.down_bad_hood_angle)

        SmartDashboard.putBoolean("Should Flip", self.fieldConstants.shouldFlip)

        wpilib.SmartDashboard.putNumber("Match Time", 140-int(self.match_time))

        if self.isTeleop():
            if self.alliance_shift == 0:
                wpilib.SmartDashboard.putString("Alliance Shift", "TRANSITION")
            elif 1 <= self.alliance_shift <= 4:
                wpilib.SmartDashboard.putString("Alliance Shift", str(int(self.alliance_shift)))
            else:
                wpilib.SmartDashboard.putString("Alliance Shift", "END GAME")
        elif self.isAutonomous():
            wpilib.SmartDashboard.putString("Alliance Shift", "AUTO")
        else:
            wpilib.SmartDashboard.putString("Alliance Shift", "DISABLED")
        wpilib.SmartDashboard.putNumber("Time Remaining", int(self.alliance_shift_time_remaining))
        wpilib.SmartDashboard.putBoolean("Hub active?", self.is_hub_active)
        # SmartDashboard.putString("Auto Win", str(self.auto_win))
        if self.isTeleop():
            self.update_hub_status()
        if self.auto_win is None:
            wpilib.SmartDashboard.putString("Auto Win", "UNKNOWN")
        else:
            wpilib.SmartDashboard.putString("Auto Win", "RED"*self.auto_win+"BLUE"*(not self.auto_win))

        if self.in_teleop_mode and self.auto_win is None and self.match_timer.get() <= 4:
            game_message = self.driverstation.getGameSpecificMessage()
            if game_message != "" and self.auto_win is None:
                self.auto_win = (game_message == "R")
                self.oi.driver2.xbox.setRumble(GenericHID.RumbleType.kLeftRumble, 0)
                self.oi.driver2.xbox.setRumble(GenericHID.RumbleType.kRightRumble, 0)



        if self.isSimulation():
            wpilib.SmartDashboard.putNumberArray("RobotPose", [self.poseEstimator.curEstPose.X(), self.poseEstimator.curEstPose.Y(), self.poseEstimator.curEstPose.rotation().degrees()])
            # SmartDashboard.putNumberArray("ZeroedComponentPoses/Pose0", [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
            # SmartDashboard.putNumberArray("ZeroedComponentPoses/Pose1", [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
            # SmartDashboard.putNumberArray("ZeroedComponentPoses/Pose2", [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])5


            default_shooter_hood = Translation3d(-0.23, 0.15, 0.5)
            cur_hood_pos = self.shooter.get_hood_position()
            final_shooter_hood_quat = Rotation3d(degreesToRadians(cur_hood_pos), 0, 0).getQuaternion()
            final_shooter_hood_trans = default_shooter_hood
            SmartDashboard.putNumberArray("FinalComponentPoses/Pose0", [final_shooter_hood_trans.X(), final_shooter_hood_trans.Y(), final_shooter_hood_trans.Z(), final_shooter_hood_quat.W(), final_shooter_hood_quat.X(), final_shooter_hood_quat.Y(), final_shooter_hood_quat.Z()])
            

            default_inner = Translation3d(0.3, 0.355, 0.2)
            if self.intake.get_position() ==  -0.06:
                cur_inner_pos = 30
            else:
                cur_inner_pos = 0
            # cur_inner_pos = self.intake.get_position()
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
            
            # cur_climber_pos = self.climber.get_position()
            # SmartDashboard.putNumberArray("FinalComponentPoses/Pose3", [0, 0, cur_climber_pos / 5, 0, 0, 0, 0])


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
                if self.tick_count % 2 == 0:
                    vals = self.drivetrain.dist_lookup_table.interpolate((self.virtual_goal - self.poseEstimator.curEstPose.translation()).norm())
                    launch_vel = vals[0]
                    launch_angle = vals[1]

                    trans = Translation3d(0, 0.27, 0.52) + Translation3d(0, -0.11, 0) + Translation3d(0, 0.11* math.cos(degreesToRadians(cur_hood_pos)), 0.11*math.sin(degreesToRadians(cur_hood_pos)))
                    launch_pos = Translation3d(self.poseEstimator.curEstPose.translation()) + trans.rotateBy(Rotation3d(0, 0, self.poseEstimator.curEstPose.rotation().radians()))
                    self.fuel_sim.launchFuel(launch_vel, launch_angle, 0, launch_pos)
                    self.fuel_in_hopper -= 1

            if self.fuel_sim.running:
                self.fuel_sim.updateSim()
            SmartDashboard.putNumber("Sim/Fuel in Hopper", self.fuel_in_hopper)
            self.tick_count += 1
        for s in self.subsystems:
            s.log()
        
        wpilib.SmartDashboard.putNumber(
            "robot oriented angle", self.robot_oriented_angle
        )

        SmartDashboard.putBoolean("Rumble D1", self.rumble_d1)
        SmartDashboard.putBoolean("Rumble D2", self.rumble_d2)

### MAIN ###

if __name__ == "__main__":
    # wpilib.run(Robot) go to \Robot-2023\robot folder and run "py -m robotpy run"
    pass
