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
import subsystems.climber
import subsystems.drivetrain
from wpimath.units import inchesToMeters

from wpimath.estimator import SwerveDrive4PoseEstimator


from phoenix6 import controls

# import subsystems.limelight
import subsystems.elevator
import subsystems.end_effector
import subsystems.funnel_intake
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

from robot_scoring_positions import RobotScoringPositions
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
        # Networktables
        nt_inst = ntcore.NetworkTableInstance.getDefault()
        nt_inst.startServer()
        self.nt_robot = nt_inst.getTable("SmartDashboard")
        # self.nt_robot.putString('led_mode', 'off')
        
        ### ARE WE USING AN FMS? (EX: IF WE ARE AT COMPETITION) ###
        self.using_FMS = False
        ## ARE WE RUNNING AN AUTO? (DO WE NEED TO WAIT TO SELECT AN AUTO BEFORE INIT)
        self.using_auto = False
        ## ARE WE RUNNING 1 DRIVER (TRUE) OR 2 DRIVERS (FALSE) ##
        self.one_driver_ctrl = True

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
        self.sim_coral_scored = []

        self.has_coral = False

        # Command scheduler
        self.scheduler = CommandScheduler.getInstance()

        self.previously_scored = True
        
        # subsystems
        self.leds = subsystems.leds.LEDs(self)
        self.poseEstimator = subsystems.poseEstimator.PoseEstimator(self)
        self.drivetrain = subsystems.drivetrain.Drivetrain(self)
        self.funnel_intake = subsystems.funnel_intake.FunnelIntake(self)
        self.elevator = subsystems.elevator.Elevator(self)
        self.end_effector = subsystems.end_effector.EndEffector(self)
        # self.climber = subsystems.climber.Climber(self)

        self.subsystems = [
            self.drivetrain,
            self.leds,
            self.poseEstimator,
            self.funnel_intake,
            self.elevator,
            self.end_effector,
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
        
        self.oi = oi.OI(self)

		### STATE MACHINE ###
        self.score_state = RobotScoringPositions.L4_Scoring # defaulting to L4
        self.at_scoring_position = False
        self.at_intake_position = False

        self.score_piece = False
        self.mechanisms_at_default = True

        self.pathplanner_config = RobotConfig.fromGUISettings()

        self.match_time = -1

        ### PATHS ###
        self.p_2 = self.followPathCommand("3P_2")
        self.p_3 = self.followPathCommand("3P_3")
        self.p_4 = self.followPathCommand("3P_4")
        self.p_5 = self.followPathCommand("3P_5")
        self.p_6 = self.followPathCommand("3P_6")
        self.p_7 = self.followPathCommand("3P_7")
        self.p_2_f5 = self.followPathCommand("f5_intake")

        self.p_1_2p = self.followPathCommand("2P_1",  PathConstraints(4.0, 4.0, degreesToRadians(540), degreesToRadians(540)))
        self.p_2_2p = self.followPathCommand("2P_2",  PathConstraints(4.0, 4.0, degreesToRadians(540), degreesToRadians(540)))
        self.p_3_2p = self.followPathCommand("2P_3",  PathConstraints(4.0, 4.0, degreesToRadians(540), degreesToRadians(540)))
        self.p_3p_f1_3 = self.followPathCommand("3P_F1_3")
        self.p_3p_f1_4 = self.followPathCommand("3P_F1_4")
        self.p_3p_f1_5 = self.followPathCommand("3P_F1_5")
        self.p_for_2p = [
            self.p_1_2p,
            self.p_2_2p,
            self.p_3_2p,
        ]
        self.p_for_3p_f1 = [
            self.p_2_f5,
            self.p_6,
            self.p_3p_f1_3,
            self.p_3p_f1_4,
            self.p_3p_f1_5,
        ]
        self.p_for_f5 = [
            self.p_2_f5,
            self.p_3,
            self.p_5,
            self.p_7,
        ]
        self.p_for_3p = [
            self.p_2,
            self.p_3,
            self.p_5,
            self.p_6,
        ]

        ### FIELD LOGGING ###
        self.field = Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)
        ### LOGGING ###
        self.remote_shell = RemoteShell(self)

		# PATH CONSTRAINTS
        self.path_constraints = PathConstraints(4.0, 4.0, degreesToRadians(540), degreesToRadians(540))

        self.auto_chooser_has_changed = False
        self.side_chooser_changed = False
        self.autoroutines = autoroutines.AutoRoutines(self)
        self.auto_side_chooser = wpilib.SendableChooser()
        self.auto_side_chooser.addOption("Right Side", False)
        self.auto_side_chooser.addOption("Left Side", True)
        self.auto_side_chooser.setDefaultOption("None (Choose Side)", None)
        self.auto_side_chooser.onChange(self.auto_side_chooser_changed)
        wpilib.SmartDashboard.putData("Auto Side Chooser", self.auto_side_chooser)
        self.auto_chooser = wpilib.SendableChooser()
        self.auto_chooser.setDefaultOption("None (Choose Auto)", None)
        self.auto_chooser.addOption("2 Piece Delayed", 1)
        self.auto_chooser.addOption("3 Piece to Face 5/6", 2)
        self.auto_chooser.addOption("3 Piece to Face 5/1", 3)
        self.auto_chooser.addOption("3 Piece to Face 4/6", 4)
        self.auto_chooser.onChange(self.auto_chooser_changed)
        wpilib.SmartDashboard.putData("Auto Chooser", self.auto_chooser)
        if self.using_auto:
            while True:
                if self.auto_side_chooser.getSelected() != None and self.auto_chooser.getSelected() != None and self.auto_chooser_has_changed and self.side_chooser_changed:
                    break
                yield from self.wait(0.5)

        # NOTE: For TUSH PUSH AUTO, RUN FLIP 3 PIECE TO F5
        if self.auto_chooser.getSelected() == 1:
            self.flip_2_piece_delay_auto(self.auto_side_chooser.getSelected())
            self.auto = self.autoroutines.two_piece_delayed()
        elif self.auto_chooser.getSelected() == 2:
            self.flip_3_piece_to_f5(self.auto_side_chooser.getSelected())
            self.auto = self.autoroutines.three_piece_to_f5()
        elif self.auto_chooser.getSelected() == 3:
            self.flip_3_piece_f1(self.auto_side_chooser.getSelected())
            self.auto = self.autoroutines.three_piece_f1()
        elif self.auto_chooser.getSelected() == 4:
            self.flip_3_piece_auto(self.auto_side_chooser.getSelected())
            self.auto = self.autoroutines.three_piece_auto()


        DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog())

		### STATE MACHINE VARIABLES ###
        self.running_pid_lineup = False
        self.manual_scoring = False
        self.position_on_source = 1
        self.score_intent = False
        self.final_lineup_pose = Pose2d()
        self.right_branch = True
        self.previous_right_branch = True
        self.previous_position_on_source = 1

        self.end_effector_canrange_for_reef_returning_bad_values = False
        self.is_climbing = False
        self.retract_climber = False
        self.descoring_algae = False
        self.strafe_for_L1 = False
        self.raise_elevator_slightly_for_L1 = False
        self.score_with_strafing = False
        self.wheels_at_x = False

        self.is_intaking = False
        self.raise_setpoints = 0.0


        SmartDashboard.putNumber("L2 Height", 23.5)
        SmartDashboard.putNumber("L3 Height", 39.5)
        SmartDashboard.putNumber("L4 Height", 61.5)
        SmartDashboard.putNumber("L2 Out Speed", 35.0)
        SmartDashboard.putNumber("L3 Out Speed", 35.0)
        SmartDashboard.putNumber("L4 Out Speed", 42.0)

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

    def auto_side_chooser_changed(self, _):
        self.side_chooser_changed = True

    def auto_chooser_changed(self, _):
        self.auto_chooser_has_changed = True

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
        new_path = PathPlannerPath(new_waypoints, self.path_constraints, ideal_starting_state=ideal_start, goal_end_state=goal_end)
        return FollowPathCommand(
            new_path,
            self.drivetrain.get_pose, # Robot pose supplier
            self.drivetrain.get_robot_relative_speeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.drivetrain.drive_robot_relative, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
            PPHolonomicDriveController(  # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(
                    const.X_KP, const.X_KI, const.X_KD
                ),  # Translation PID constants
                PIDConstants(
                    const.THETA_KP, const.THETA_KI, const.THETA_KD
                ),  # Rotation PID constants
            ),
            self.pathplanner_config, # The robot configuration
            self.drivetrain.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self.drivetrain # Reference to this subsystem to set requirements
        )
    
    def flip_one_piece_f4(self, left_side : bool):
        if not left_side:
            self.score_1_face = 4
            self.score_1_right_branch = False
        else:
            self.score_1_face = 4
            self.score_1_right_branch = True
        self.poseEstimator.poseEst.resetPose(self.fieldConstants.flip_Pose2d(Pose2d(7.167, self.fieldConstants.fieldWidth / 2, Rotation2d.fromDegrees(270))))
        self.poseEstimator.poseEstSingleTag.resetPose(self.fieldConstants.flip_Pose2d(Pose2d(7.167, self.fieldConstants.fieldWidth / 2, Rotation2d.fromDegrees(270))))

    def flip_2_piece_delay_auto(self, left_side : bool):
        if not left_side:
            self.score_1_face = 4
            self.score_1_right_branch = False
            self.left_source_auto = False
            self.score_2_face = 4
            self.score_2_right_branch = True
            self.wait_score_1_2p = 0.0
            self.auto_position_source = 3
        else:
            self.left_source_auto = True
            self.score_1_face = 4
            self.score_1_right_branch = True
            self.score_2_face = 4
            self.score_2_right_branch = False
            self.wait_score_1_2p = 1.0
            self.auto_position_source = 3
            for idx, command in enumerate(self.p_for_2p):
                self.p_for_2p[idx] = self.flip_path_cmd_across_x(command)
        self.poseEstimator.poseEst.resetPose(self.fieldConstants.flip_Pose2d(Pose2d(7.167, self.fieldConstants.fieldWidth / 2, Rotation2d.fromDegrees(270))))
        self.poseEstimator.poseEstSingleTag.resetPose(self.fieldConstants.flip_Pose2d(Pose2d(7.167, self.fieldConstants.fieldWidth / 2, Rotation2d.fromDegrees(270))))


    def flip_3_piece_auto(self, left_side : bool):
        if not left_side:
            self.score_1_face = 4
            self.score_1_right_branch = False
            self.score_2_face = 6
            self.score_2_right_branch = True
            self.score_3_face = 6
            self.score_3_right_branch = False
            self.score_4_face = 1
            self.score_4_right_branch = True
            self.left_source_auto = False
            self.auto_position_source = 3
        else:
            self.left_source_auto = True
            self.score_1_face = 4
            self.score_1_right_branch = True
            self.score_2_face = 2
            self.score_2_right_branch = False
            self.score_3_face = 2
            self.score_3_right_branch = True
            self.score_4_face = 1
            self.score_4_right_branch = False
            self.auto_position_source = 3
            for idx, command in enumerate(self.p_for_3p):
                self.p_for_3p[idx] = self.flip_path_cmd_across_x(command)
        self.poseEstimator.poseEst.resetPose(self.fieldConstants.flip_Pose2d(Pose2d(7.167, self.fieldConstants.fieldWidth / 2, Rotation2d.fromDegrees(270))))
        self.poseEstimator.poseEstSingleTag.resetPose(self.fieldConstants.flip_Pose2d(Pose2d(7.167, self.fieldConstants.fieldWidth / 2, Rotation2d.fromDegrees(270))))

    def flip_3_piece_f1(self, left_side : bool):
        if not left_side:
            self.left_source_auto = False
            self.score_1_face = 5
            self.score_1_right_branch = False
            self.score_2_face = 1
            self.score_2_right_branch = True
            self.score_3_face = 1
            self.score_3_right_branch = False
            self.score_4_face = 6
            self.score_4_right_branch = False
            self.auto_position_source = 3
            self.poseEstimator.poseEst.resetPose(self.fieldConstants.flip_Pose2d(Pose2d(7.167, 1.372, Rotation2d.fromDegrees(270))))
            self.poseEstimator.poseEstSingleTag.resetPose(self.fieldConstants.flip_Pose2d(Pose2d(7.167, 1.372, Rotation2d.fromDegrees(270))))
        else:
            self.left_source_auto = True
            self.score_1_face = 3
            self.score_1_right_branch = True
            self.score_2_face = 1
            self.score_2_right_branch = False
            self.score_3_face = 1
            self.score_3_right_branch = True
            self.score_4_face = 2
            self.score_4_right_branch = True
            self.auto_position_source = 3
            self.poseEstimator.poseEst.resetPose(self.fieldConstants.flip_Pose2d(Pose2d(7.167, self.fieldConstants.fieldWidth - 1.372, Rotation2d.fromDegrees(270))))
            self.poseEstimator.poseEstSingleTag.resetPose(self.fieldConstants.flip_Pose2d(Pose2d(7.167, self.fieldConstants.fieldWidth - 1.372, Rotation2d.fromDegrees(270))))
            for idx, command in enumerate(self.p_for_3p_f1):
                self.p_for_3p_f1[idx] = self.flip_path_cmd_across_x(command)

    def flip_3_piece_to_f5(self, left_side : bool):
        if not left_side:
            self.score_1_face = 5
            self.score_1_right_branch = False
            self.score_2_face = 6
            self.score_2_right_branch = True
            self.score_3_face = 6
            self.score_3_right_branch = False
            self.score_4_face = 1
            self.score_4_right_branch = True
            self.left_source_auto = False
            self.auto_position_source = 3
            self.poseEstimator.poseEstSingleTag.resetPose(Pose2d(self.fieldConstants.flip_Translation2d(Translation2d(7.13, self.fieldConstants.fieldWidth - 6.772)), self.poseEstimator.getYaw()))
            if self.isSimulation():
                self.poseEstimator.curEstPose = Pose2d(self.fieldConstants.flip_Translation2d(Translation2d(7.13, self.fieldConstants.fieldWidth - 6.772)), self.poseEstimator.getYaw())
        else:
            self.score_1_face = 3
            self.score_1_right_branch = True
            self.score_2_face = 2
            self.score_2_right_branch = False
            self.score_3_face = 2
            self.score_3_right_branch = True
            self.left_source_auto = True
            self.auto_position_source = 3
            self.score_4_face = 1
            self.score_4_right_branch = False
            self.poseEstimator.poseEstSingleTag.resetPose(Pose2d(self.fieldConstants.flip_Translation2d(Translation2d(7.13, 6.772)), self.poseEstimator.getYaw()))
            
            # for idx, command in enumerate(self.p_for_f5):
            #     self.p_for_f5[idx] = self.flip_path_cmd_across_x(command)

    def followPathCommand(self, pathName: str, pathConstraints=None):
        path = PathPlannerPath.fromPathFile(pathName)
        if pathConstraints != None:
            path._globalConstraints = pathConstraints
        return FollowPathCommand(
            path,
            self.drivetrain.get_pose, # Robot pose supplier
            self.drivetrain.get_robot_relative_speeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.drivetrain.drive_robot_relative, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
            PPHolonomicDriveController(  # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(
                    const.X_KP, const.X_KI, const.X_KD
                ),  # Translation PID constants
                PIDConstants(
                    const.THETA_KP, const.THETA_KI, const.THETA_KD
                ),  # Rotation PID constants
            ),
            self.pathplanner_config, # The robot configuration
            self.drivetrain.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self.drivetrain # Reference to this subsystem to set requirements
        )

    ### DISABLED ###

    def disabled_mode(self):
        if self.using_FMS:
            while not self.driverstation.isFMSAttached():
                yield
            yield from self.wait(2.0)
        if self.using_auto:
            while True:
                if self.auto_side_chooser.getSelected() != None and self.auto_chooser.getSelected() != None and self.auto_chooser_has_changed and self.side_chooser_changed:
                    break
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

        self.scheduler.schedule(self.auto)

    ### TELEOPERATED ###
    def teleop_mode(self):
        self.scheduler.cancelAll()
        # self.mechanisms_at_default = True
        self.funnel_intake.is_intaking = False
        self.end_effector.is_intaking = False
        self.running_pid_lineup = False
        self.drivetrain.at_inter_pose = False
        self.score_intent = False
        self.in_autonomous_mode = False
        self.funnel_intake.piece_passing_through = False
        self.oi.robot_oriented_angle = self.poseEstimator.getYaw().degrees()

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
        wpilib.SmartDashboard.putNumber("Score state", self.score_state.number)
        wpilib.SmartDashboard.putBoolean("Score piece", self.score_piece)
        wpilib.SmartDashboard.putBoolean("Mechanisms at default", self.mechanisms_at_default)
        wpilib.SmartDashboard.putBoolean("At scoring position", self.at_scoring_position)
        wpilib.SmartDashboard.putBoolean("at intake position", self.at_intake_position)
        wpilib.SmartDashboard.putBoolean("OI Score Intent", self.score_intent)
        wpilib.SmartDashboard.putBoolean("Has Coral", self.has_coral)
        wpilib.SmartDashboard.putNumberArray("empty pose", [0, 0, 0])
        wpilib.SmartDashboard.putBoolean("Connected to FMS", self.driverstation.isFMSAttached())

        

        RobotScoringPositions.L2_Scoring.elevator_height = SmartDashboard.getNumber("L2 Height", 23.5)
        RobotScoringPositions.L3_Scoring.elevator_height = SmartDashboard.getNumber("L3 Height", 39.5)
        RobotScoringPositions.L4_Scoring.elevator_height = SmartDashboard.getNumber("L4 Height", 61.5)
        RobotScoringPositions.L2_Scoring.end_effector_outtake_speed = SmartDashboard.getNumber("L2 Out Speed", 35.0)
        RobotScoringPositions.L3_Scoring.end_effector_outtake_speed = SmartDashboard.getNumber("L3 Out Speed", 35.0)
        RobotScoringPositions.L4_Scoring.end_effector_outtake_speed = SmartDashboard.getNumber("L4 Out Speed", 42.0)

        if self.isSimulation():
            wpilib.SmartDashboard.putNumberArray("RobotPose", [self.poseEstimator.curEstPose.X(), self.poseEstimator.curEstPose.Y(), self.poseEstimator.curEstPose.rotation().degrees()])
            #elevator stage 3
            wpilib.SmartDashboard.putNumberArray("ZeroedComponentPoses/Pose0", [0.0, 0.0, inchesToMeters(self.elevator.command_height) * 0.87, 0.0, 0.0, 0.0, 0.0])
            #elevator stage 2
            wpilib.SmartDashboard.putNumberArray("ZeroedComponentPoses/Pose1", [0.0, 0.0, inchesToMeters(self.elevator.command_height) * 0.87 / 2, 0.0, 0.0, 0.0, 0.0])
            #end effector
            wpilib.SmartDashboard.putNumberArray("ZeroedComponentPoses/Pose2", [0.0, -1 * inchesToMeters(self.end_effector.command_position) / math.sqrt(2), inchesToMeters(self.elevator.command_height) * 0.87 * 4 / 3 + inchesToMeters(self.end_effector.command_position) / math.sqrt(2), 0.0, 0.0, 0.0, 0.0])
            if self.has_coral:
                #la coral
                coral_pose = [0.0, -1 * inchesToMeters(self.end_effector.command_position) / math.sqrt(2), inchesToMeters(self.elevator.command_height) * 0.87 * 4 / 3 + inchesToMeters(self.end_effector.command_position) / math.sqrt(2), 0.0, 0.0, 0.0, 0.0]
            else:
                coral_pose = []
            wpilib.SmartDashboard.putNumberArray("ZeroedComponentPoses/Pose3", coral_pose)
            # new_pose = Pose3d(old_pose.translation(), old_rotation)
            for face in range(6):
                for level in range(4):
                    if level == 0:
                        amount_in = -0.5
                        pitch_rotate = 15
                        amount_up = 0.0
                    elif level == 1:
                        amount_in = -4
                        pitch_rotate = 0
                        amount_up = 0.0
                    elif level == 2:
                        amount_in = -4
                        pitch_rotate = 0
                        amount_up = 0.0
                    elif level == 3:
                        amount_in = -4
                        pitch_rotate = 25
                        amount_up = 3.25
                    pose_right : Pose3d = self.fieldConstants.Reef.branchPositions[face * 2][level].transformBy(Transform3d(Pose3d(),Pose3d(inchesToMeters(amount_in), 0.0, inchesToMeters(amount_up), Rotation3d(0.0, degreesToRadians(pitch_rotate), 0.0))))
                    quat_right = pose_right.rotation().getQuaternion()
                    pose_left : Pose3d = self.fieldConstants.Reef.branchPositions[face * 2 + 1][level].transformBy(Transform3d(Pose3d(),Pose3d(inchesToMeters(amount_in), 0.0, inchesToMeters(amount_up), Rotation3d(0.0, degreesToRadians(pitch_rotate), 0.0))))
                    quat_left = pose_left.rotation().getQuaternion()
                    if [face + 1, 4 - level, True] not in self.sim_coral_scored:
                        appending_pose_right = []
                    else:
                        appending_pose_right = [pose_right.X(), pose_right.Y(), pose_right.Z(), quat_right.W(), quat_right.X(), quat_right.Y(), quat_right.Z()]
                    if [face + 1, 4 - level, False] not in self.sim_coral_scored:
                        appending_pose_left = []
                    else:
                        appending_pose_left = [pose_left.X(), pose_left.Y(), pose_left.Z(), quat_left.W(), quat_left.X(), quat_left.Y(), quat_left.Z()]
                    wpilib.SmartDashboard.putNumberArray("coral right " + str(face + 1) + str(4 -level), appending_pose_right)
                    wpilib.SmartDashboard.putNumberArray("coral left " + str(face + 1) + str(4 - level), appending_pose_left)
            wpilib.SmartDashboard.putNumber("Sim Pieces Scored", len(self.sim_coral_scored))
            coral_points_scored = 0
            for coral in self.sim_coral_scored:
                coral_points_scored += coral[1] + 1
            SmartDashboard.putNumber("Sim Points Scored", coral_points_scored)
            # wpilib.SmartDashboard.putNumberArray("FinalComponentPoses/Pose3", [0.0,0.0, inchesToMeters(elevator_height) * 1.5, pose3quat.X(), pose3quat.Y(), pose3quat.Z(), pose3quat.W()])
            # wpilib.SmartDashboard.putNumberArray("FinalComponentPoses/Pose4", [0.0, 0.0, inchesToMeters(elevator_height) / 2, 0.0, 0.0, 0.0, 0.0])
            # wpilib.SmartDashboard.putNumberArray("FinalComponentPoses/Pose5", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        for s in self.subsystems:
            s.log()

        self.match_time = self.driverstation.getMatchTime()
        wpilib.SmartDashboard.putNumber("Match Time", self.match_time)
        wpilib.SmartDashboard.putNumber(
            "robot oriented angle", self.oi.robot_oriented_angle
        )


### MAIN ###

if __name__ == "__main__":
    # wpilib.run(Robot) go to \Robot-2023\robot folder and run "py -m robotpy run"
    pass
