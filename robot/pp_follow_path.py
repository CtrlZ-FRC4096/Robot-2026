from math import hypot

from pathplannerlib.controller import *
from pathplannerlib.path import PathPlannerPath, GoalEndState, PathConstraints, IdealStartingState
from pathplannerlib.trajectory import PathPlannerTrajectory
from pathplannerlib.telemetry import PPLibTelemetry
from pathplannerlib.logging import PathPlannerLogging
from pathplannerlib.util import floatLerp, FlippingUtil, DriveFeedforwards
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds
from wpilib import Timer
from commands2 import Command, Subsystem, SequentialCommandGroup, DeferredCommand
import commands2.cmd as cmd
from typing import Callable
from pathplannerlib.config import RobotConfig
from pathplannerlib.pathfinding import Pathfinding
from pathplannerlib.events import EventScheduler
from hal import report, tResourceType


class CustomFollowPathCommand(Command):
    _originalPath: PathPlannerPath
    _poseSupplier: Callable[[], Pose2d]
    _speedsSupplier: Callable[[], ChassisSpeeds]
    _output: Callable[[ChassisSpeeds, DriveFeedforwards], None]
    _controller: PathFollowingController
    _robotConfig: RobotConfig
    _shouldFlipPath: Callable[[], bool]

    _eventScheduler: EventScheduler

    _timer: Timer = Timer()

    _path: PathPlannerPath = None
    _trajectory: PathPlannerTrajectory = None

    currentPathName: str = ''

    def __init__(self, path: PathPlannerPath, pose_supplier: Callable[[], Pose2d],
                 speeds_supplier: Callable[[], ChassisSpeeds],
                 output: Callable[[ChassisSpeeds, DriveFeedforwards], None],
                 controller: PathFollowingController, robot_config: RobotConfig,
                 should_flip_path: Callable[[], bool], *requirements: Subsystem, use_special_init=False):
        """
        Construct a base path following command

        :param path: The path to follow
        :param pose_supplier: Function that supplies the current field-relative pose of the robot
        :param speeds_supplier: Function that supplies the current robot-relative chassis speeds
        :param output: Output function that accepts robot-relative ChassisSpeeds and feedforwards for
            each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
            using a differential drive, they will be in L, R order.
            <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize your
            module states, you will need to reverse the feedforwards for modules that have been flipped
        :param controller: Path following controller that will be used to follow the path
        :param robot_config The robot configuration
        :param should_flip_path: Should the path be flipped to the other side of the field? This will maintain a global blue alliance origin.
        :param requirements: Subsystems required by this command, usually just the drive subsystem
        """
        super().__init__()

        self._originalPath = path
        self._poseSupplier = pose_supplier
        self._speedsSupplier = speeds_supplier
        self._output = output
        self._controller = controller
        self._robotConfig = robot_config
        self._shouldFlipPath = should_flip_path
        self._eventScheduler = EventScheduler()

        self.use_special_init = use_special_init

        self.did_first_init = False

        self.addRequirements(*requirements)

        eventReqs = EventScheduler.getSchedulerRequirements(self._originalPath)
        for req in requirements:
            if req in eventReqs:
                raise RuntimeError(
                    'Events that are triggered during path following cannot require the drive subsystem')
        self.addRequirements(*eventReqs)

        self._path = self._originalPath
        # Ensure the ideal trajectory is generated3
        idealTrajectory = self._path.getIdealTrajectory(self._robotConfig)
        if idealTrajectory is not None:
            self._trajectory = idealTrajectory

    def initialize(self):
        if self.use_special_init:
            if not self.did_first_init:
                self.did_first_init = True

                CustomFollowPathCommand.currentPathName = self._originalPath.name
                if self._shouldFlipPath() and not self._originalPath.preventFlipping:
                    self._path = self._originalPath.flipPath()
                else:
                    self._path = self._originalPath

                currentPose = self._poseSupplier()
                currentSpeeds = self._speedsSupplier()

                self._controller.reset(currentPose, currentSpeeds)

                linearVel = math.hypot(currentSpeeds.vx, currentSpeeds.vy)

                if self._path.getIdealStartingState() is not None:
                    # Check if we match the ideal starting state
                    idealVelocity = abs(linearVel - self._path.getIdealStartingState().velocity) <= 0.25
                    idealRotation = (not self._robotConfig.isHolonomic) or abs(
                        (currentPose.rotation() - self._path.getIdealStartingState().rotation).degrees()) <= 30.0
                    if idealVelocity and idealRotation:
                        # We can use the ideal trajectory
                        self._trajectory = self._path.getIdealTrajectory(self._robotConfig)
                    else:
                        # We need to regenerate
                        self._trajectory = self._path.generateTrajectory(currentSpeeds, currentPose.rotation(),
                                                                        self._robotConfig)
                else:
                    # No ideal starting state, generate the trajectory
                    self._trajectory = self._path.generateTrajectory(currentSpeeds, currentPose.rotation(), self._robotConfig)

                self._eventScheduler.initialize(self._trajectory)

                PathPlannerLogging.logActivePath(self._path)
                PPLibTelemetry.setCurrentPath(self._path)
            else:
                self._timer.reset()
                self._timer.start()
        else:
            CustomFollowPathCommand.currentPathName = self._originalPath.name
            if self._shouldFlipPath() and not self._originalPath.preventFlipping:
                self._path = self._originalPath.flipPath()
            else:
                self._path = self._originalPath

            currentPose = self._poseSupplier()
            currentSpeeds = self._speedsSupplier()

            self._controller.reset(currentPose, currentSpeeds)

            linearVel = math.hypot(currentSpeeds.vx, currentSpeeds.vy)

            if self._path.getIdealStartingState() is not None:
                # Check if we match the ideal starting state
                idealVelocity = abs(linearVel - self._path.getIdealStartingState().velocity) <= 0.25
                idealRotation = (not self._robotConfig.isHolonomic) or abs(
                    (currentPose.rotation() - self._path.getIdealStartingState().rotation).degrees()) <= 30.0
                if idealVelocity and idealRotation:
                    # We can use the ideal trajectory
                    self._trajectory = self._path.getIdealTrajectory(self._robotConfig)
                else:
                    # We need to regenerate
                    self._trajectory = self._path.generateTrajectory(currentSpeeds, currentPose.rotation(),
                                                                    self._robotConfig)
            else:
                # No ideal starting state, generate the trajectory
                self._trajectory = self._path.generateTrajectory(currentSpeeds, currentPose.rotation(), self._robotConfig)

            PathPlannerLogging.logActivePath(self._path)
            PPLibTelemetry.setCurrentPath(self._path)

            self._eventScheduler.initialize(self._trajectory)

            self._timer.reset()
            self._timer.start()

    def execute(self):
        currentTime = self._timer.get()
        targetState = self._trajectory.sample(currentTime)
        if not self._controller.isHolonomic() and self._path.isReversed():
            targetState = targetState.reverse()

        currentPose = self._poseSupplier()
        currentSpeeds = self._speedsSupplier()

        targetSpeeds = self._controller.calculateRobotRelativeSpeeds(currentPose, targetState)

        currentVel = math.hypot(currentSpeeds.vx, currentSpeeds.vy)

        self._output(targetSpeeds, targetState.feedforwards)

        PPLibTelemetry.setCurrentPose(currentPose)
        PathPlannerLogging.logCurrentPose(currentPose)

        PPLibTelemetry.setTargetPose(targetState.pose)
        PathPlannerLogging.logTargetPose(targetState.pose)

        PPLibTelemetry.setVelocities(currentVel, targetState.linearVelocity, currentSpeeds.omega, targetSpeeds.omega)

        self._eventScheduler.execute(currentTime)

    def isFinished(self) -> bool:
        return self._timer.hasElapsed(self._trajectory.getTotalTimeSeconds())

    def end(self, interrupted: bool):
        self._timer.stop()
        CustomFollowPathCommand.currentPathName = ''

        # Only output 0 speeds when ending a path that is supposed to stop, this allows interrupting
        # the command to smoothly transition into some auto-alignment routine
        if not interrupted and self._path.getGoalEndState().velocity < 0.1:
            self._output(ChassisSpeeds(), DriveFeedforwards.zeros(self._robotConfig.numModules))

        PathPlannerLogging.logActivePath(None)

        self._eventScheduler.end()
