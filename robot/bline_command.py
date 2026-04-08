"""
BLineCommand: Command-based path following for RobotPy, inspired by the BLine Java Path class.
This class is a skeleton for a command that follows a Path, with support for flipping, mirroring, constraints, and event triggers.
"""
from commands2 import Command
from typing import Callable, Optional, Union, Dict, Set, List, Tuple
from dataclasses import dataclass, field, replace
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from .path import Path, PathElement, Waypoint, TranslationTarget, RotationTarget, EventTrigger
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds
import math
from wpilibextra.coroutine.subsystem import Subsystem
from wpimath.units import degreesToRadians


# SEGMENT_EPSILON = 1e-6
# T_RATIO_EPSILON = 1e-9

@dataclass(frozen=True)
class TranslationSegmentState:
    start_translation_index : int
    end_translation_index : int
    start_translation : Translation2d
    end_translation : Translation2d
    segment_length : float
    segment_progress : float

    def is_degenerate(self):
        return self.segment_length < 1e-6
    
@dataclass(frozen=True)
class RotationSegmentBounds:
    start_translation_index : int
    end_translation_index : int
    start_translation : Translation2d
    end_translation : Translation2d

@dataclass(frozen=True)
class RotationSelection:
    active_rotation_index : int
    previous_rotation_index : int
    
class Builder:
        def __init__(
            self,
            drive_subsystem,
            pose_supplier: Callable[[], Pose2d],
            robot_relative_speeds_supplier: Callable[[], ChassisSpeeds],
            robot_relative_speeds_consumer: Callable[[ChassisSpeeds], None],
            translation_controller: PIDController,
            rotation_controller: PIDController,
            cross_track_controller: PIDController
        ):
            self.drive_subsystem = drive_subsystem
            self.pose_supplier = pose_supplier
            self.robot_relative_speeds_supplier = robot_relative_speeds_supplier
            self.robot_relative_speeds_consumer = robot_relative_speeds_consumer
            self.translation_controller = translation_controller
            self.rotation_controller = rotation_controller
            self.cross_track_controller = cross_track_controller

            # Default optional settings
            self.should_flip_path_supplier: Callable[[], bool] = lambda: False
            self.should_mirror_path_supplier: Callable[[], bool] = lambda: False
            self.pose_reset_consumer: Callable[[Pose2d], None] = lambda pose: None
            self.use_t_ratio_based_translation_handoffs = False

        def with_should_flip(self, supplier: Callable[[], bool]) -> "BLineCommand.Builder":
            """Configures a custom supplier to determine if the path should be flipped."""
            self.should_flip_path_supplier = supplier
            return self

        def with_default_should_flip(self) -> "BLineCommand.Builder":
            """Automatically flips when on the Red Alliance."""
            # You'll need to implement BLineCommand.should_flip_path logic
            self.should_flip_path_supplier = BLineCommand.should_flip_path 
            return self

        def with_should_mirror(self, supplier: Callable[[], bool]) -> "BLineCommand.Builder":
            """Configures a custom supplier to determine if the path should be mirrored."""
            self.should_mirror_path_supplier = supplier
            return self

        def with_pose_reset(self, consumer: Callable[[Pose2d], None]) -> "BLineCommand.Builder":
            """Consumer to reset odometry at the start of the path."""
            self.pose_reset_consumer = consumer
            return self

        def with_t_ratio_handoffs(self, enabled: bool) -> "BLineCommand.Builder":
            """Enables segment-progress based handoffs instead of distance-based."""
            self.use_t_ratio_based_translation_handoffs = enabled
            return self

        def build(self, path: Path) -> "BLineCommand":
            """Returns a fully configured BLineCommand."""
            if not all([self.translation_controller, self.rotation_controller, self.cross_track_controller]):
                raise ValueError("All PID controllers must be provided to the Builder.")

            return BLineCommand(
                path=path,
                pose_supplier=self.pose_supplier,
                robot_relative_speeds_supplier=self.robot_relative_speeds_supplier,
                robot_relative_speeds_consumer=self.robot_relative_speeds_consumer,
                translation_controller=self.translation_controller,
                rotation_controller=self.rotation_controller,
                cross_track_controller=self.cross_track_controller,
                should_flip_path_supplier=self.should_flip_path_supplier,
                should_mirror_path_supplier=self.should_mirror_path_supplier,
                pose_reset_consumer=self.pose_reset_consumer,
                # Ensure your BLineCommand.__init__ accepts this new flag:
                # use_t_ratio=self.use_t_ratio_based_translation_handoffs 
            )

class BLineCommand(Command):

    NO_ACTIVE_ROTATION_INDEX = -1
    event_trigger_registry : Dict[str, Callable[[], None]] = {}

    def __init__(self,
                 path: Path,
                 driveSubsystem : Subsystem,
                 pose_supplier: Callable[[], Pose2d],
                 robot_relative_speeds_supplier: Callable[[], ChassisSpeeds],
                 robot_relative_speeds_consumer: Callable[[ChassisSpeeds], None],
                 timestamp_supplier : Callable[[], float],
                 translation_controller: PIDController,
                 rotation_controller: PIDController,
                 cross_track_controller: PIDController,
                 should_flip_path_supplier: Callable[[], bool],
                 should_mirror_path_supplier: Callable[[], bool],
                 pose_reset_consumer: Optional[Callable[[Pose2d], None]] = None):
        super().__init__()

        if translation_controller is None or rotation_controller is None or cross_track_controller is None:
            raise RuntimeError("give me controllers")
        self.path = path.copy()
        self.pose_supplier = pose_supplier
        self.robot_relative_speeds_supplier = robot_relative_speeds_supplier
        self.robot_relative_speeds_consumer = robot_relative_speeds_consumer
        self.translation_controller = translation_controller
        self.rotation_controller = rotation_controller
        self.cross_track_controller = cross_track_controller
        self.should_flip_path_supplier = should_flip_path_supplier
        self.should_mirror_path_supplier = should_mirror_path_supplier
        self.pose_reset_consumer = pose_reset_consumer
        self.timestamp_supplier = timestamp_supplier

        self.rotation_element_index = self.NO_ACTIVE_ROTATION_INDEX
        self.translation_element_index = 0
        self.event_trigger_element_index = 0
        self.last_speeds = ChassisSpeeds()
        self.last_timestamp = 0.0
        self.log_counter = 0
        self.path_init_start_pose = Pose2d()
        self.previous_rotation_element_target_rad = 0.0
        self.previous_rotation_element_index = 0
        self.current_rotation_target_rad = Rotation2d()
        self.current_rotation_target_init_rad = 0.0
        self.path_elements_with_constraints = []
        self.fired_event_trigger_indices: Set[int] = set()
        self.robot_translations = []
        self.fired_event_trigger_count = 0
        self.finished = False

        self.addRequirements(driveSubsystem)

    def configure_controllers(self):
        self.translation_controller.setTolerance(self.path.get_end_translation_tolerance_m())
        self.rotation_controller.setTolerance(degreesToRadians(self.path.get_end_rotation_tolerance_deg()))
        self.cross_track_controller.setTolerance(self.path.get_end_translation_tolerance_m())
        self.rotation_controller.enableContinuousInput(-math.pi, math.pi)

    def initialize(self):
        if self.translation_controller is None or self.rotation_controller is None:
            raise RuntimeError("give me translation and rotation controllers")
        
        if not self.path.is_valid():
            return
        
        if self.should_flip_path_supplier():
            self.path.flip()

        if self.should_mirror_path_supplier():
            self.path.mirror()
        
        self.path_elements_with_constraints = self.path.get_path_elements_with_constraints_no_waypoints()
        if len(self.path_elements_with_constraints) == 0:
            raise RuntimeError("must be real path")
        
        start_pose = self.path.get_start_pose(self.pose_supplier().rotation())

        self.rotation_element_index = self.NO_ACTIVE_ROTATION_INDEX
        self.translation_element_index = 0
        self.event_trigger_element_index = 0
        self.fired_event_trigger_indices.clear()
        self.fired_event_trigger_count = 0
        self.last_timestamp = self.timestamp_supplier()
        self.path_init_start_pose = self.pose_supplier()
        self.last_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(self.robot_relative_speeds_supplier(), self.path_init_start_pose.rotation())

        self.previous_rotation_element_target_rad = self.path_init_start_pose.rotation().radians()
        self.previous_rotation_element_index = -1
        self.current_rotation_target_init_rad = self.path_init_start_pose.rotation().radians()
        
        self.rotation_controller.reset()
        self.translation_controller.reset()
        self.configure_controllers()

        path_translations = []
        self.robot_translations.clear()
        self.log_counter = 0
        for i in range(len(self.path_elements_with_constraints)):
            if isinstance(self.path_elements_with_constraints[i][0], TranslationTarget):
                path_translations.append(self.path_elements_with_constraints[i][0].translation)

    def stop_commanded_motion(self):
        zero_speeds = ChassisSpeeds()
        self.robot_relative_speeds_consumer(zero_speeds)
        self.last_speeds = zero_speeds

    def is_translation_target_at(self, index : int):
        return index >= 0 and index < len(self.path_elements_with_constraints) and isinstance(self.path_elements_with_constraints[index][0], TranslationTarget)
    
    def is_rotation_target_at(self, index : int):
        return index >= 0 and index < len(self.path_elements_with_constraints) and isinstance(self.path_elements_with_constraints[index][0], RotationTarget)

        

    def execute(self):
        if not self.path.is_valid():
            self.stop_commanded_motion()         
            return
        
        now = self.timestamp_supplier()
        dt = now - self.last_timestamp
        self.last_timestamp = now

        cur_pose = self.pose_supplier()

        if self.translation_element_index >= len(self.path_elements_with_constraints):
            self.stop_commanded_motion()
            return
        
        if not self.is_translation_target_at(self.translation_element_index):
            self.stop_commanded_motion()
            return
        
        previous_translation_index = self.translation_element_index
        

    def isFinished(self):
        return self.finished

    def end(self, interrupted):
        # Stop drivetrain if needed
        pass

    def flip_path(self):
        # Implement flipping logic (alliance side)
        # This is a stub; actual implementation depends on your field symmetry
        pass

    def mirror_path(self):
        # Implement mirroring logic (vertical centerline)
        # This is a stub; actual implementation depends on your field symmetry
        pass

    def get_start_pose(self) -> Pose2d:
        # Return the starting pose of the path
        # This should match the Java getStartPose logic
        # For now, just return the first translation target with rotation
        for elem in self.path.get_path_elements():
            if isinstance(elem, Waypoint):
                return Pose2d(elem.translation_target.translation, elem.rotation_target.rotation)
            if isinstance(elem, TranslationTarget):
                return Pose2d(elem.translation, Rotation2d())
        raise RuntimeError("Path has no valid start pose")

    def get_initial_module_direction(self) -> Rotation2d:
        # Return the initial module direction (for swerve)
        # This should match the Java getInitialModuleDirection logic
        # For now, just return the rotation of the first element
        pose = self.get_start_pose()
        return pose.rotation()

    # Add more methods as needed for constraint resolution, event triggers, etc.
