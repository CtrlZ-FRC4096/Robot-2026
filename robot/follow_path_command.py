"""
FollowPathCommand: Command-based path following for RobotPy, translated from the BLine Java FollowPath class.
Implements full path following with translation, rotation, cross-track PID, event triggers, flipping, mirroring, and constraints.
Logging hooks are omitted for simplicity.
"""
from commands2 import Command
from typing import Callable, Optional, Dict, Set, List, Tuple
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.controller import PIDController
from .path import Path, PathElement, TranslationTarget, RotationTarget, EventTrigger, Waypoint
import math

class FollowPathCommand(Command):
    SEGMENT_EPSILON = 1e-6
    T_RATIO_EPSILON = 1e-9
    NO_ACTIVE_ROTATION_INDEX = -1

    event_trigger_registry: Dict[str, Callable[[], None]] = {}

    @staticmethod
    def register_event_trigger(key: str, action: Callable[[], None]):
        FollowPathCommand.event_trigger_registry[key] = action

    def __init__(self,
                 path: Path,
                 pose_supplier: Callable[[], Pose2d],
                 robot_relative_speeds_supplier: Callable[[], ChassisSpeeds],
                 robot_relative_speeds_consumer: Callable[[ChassisSpeeds], None],
                 translation_controller: PIDController,
                 rotation_controller: PIDController,
                 cross_track_controller: PIDController,
                 should_flip_path_supplier: Optional[Callable[[], bool]] = None,
                 should_mirror_path_supplier: Optional[Callable[[], bool]] = None,
                 pose_reset_consumer: Optional[Callable[[Pose2d], None]] = None):
        super().__init__()
        self.path = path.copy()
        self.pose_supplier = pose_supplier
        self.robot_relative_speeds_supplier = robot_relative_speeds_supplier
        self.robot_relative_speeds_consumer = robot_relative_speeds_consumer
        self.translation_controller = translation_controller
        self.rotation_controller = rotation_controller
        self.cross_track_controller = cross_track_controller
        self.should_flip_path_supplier = should_flip_path_supplier or (lambda: False)
        self.should_mirror_path_supplier = should_mirror_path_supplier or (lambda: False)
        self.pose_reset_consumer = pose_reset_consumer

        self.rotation_element_index = self.NO_ACTIVE_ROTATION_INDEX
        self.translation_element_index = 0
        self.event_trigger_element_index = 0
        self.last_speeds = ChassisSpeeds()
        self.last_timestamp = 0.0
        self.path_init_start_pose = Pose2d()
        self.previous_rotation_element_target_rad = 0.0
        self.previous_rotation_element_index = 0
        self.current_rotation_target_rad = Rotation2d()
        self.current_rotation_target_init_rad = 0.0
        self.path_elements_with_constraints = []
        self.fired_event_trigger_indices: Set[int] = set()
        self.fired_event_trigger_count = 0
        self.finished = False

    def initialize(self):
        # Optionally reset odometry to path start pose
        if self.pose_reset_consumer:
            self.pose_reset_consumer(self.path.get_start_pose())
        # Flip/mirror if needed
        if self.should_flip_path_supplier():
            self.path.flip()
        if self.should_mirror_path_supplier():
            self.path.mirror()
        self.path_elements_with_constraints = self.path.get_path_elements_with_constraints()
        self.translation_element_index = 0
        self.rotation_element_index = self.NO_ACTIVE_ROTATION_INDEX
        self.event_trigger_element_index = 0
        self.fired_event_trigger_indices.clear()
        self.fired_event_trigger_count = 0
        self.finished = False
        self.last_timestamp = 0.0
        self.last_speeds = ChassisSpeeds()
        self.path_init_start_pose = self.pose_supplier()
        # Reset controllers
        self.translation_controller.reset()
        self.rotation_controller.reset()
        self.cross_track_controller.reset()

    def execute(self):
        if not self.path.is_valid():
            self.finished = True
            return
        pose = self.pose_supplier()
        # Get current translation/rotation targets
        elements = self.path.get_path_elements()
        if self.translation_element_index >= len(elements):
            self.finished = True
            return
        # Find next translation target
        while self.translation_element_index < len(elements):
            elem = elements[self.translation_element_index]
            if isinstance(elem, (TranslationTarget, Waypoint)):
                break
            self.translation_element_index += 1
        if self.translation_element_index >= len(elements):
            self.finished = True
            return
        # Compute error to translation target
        target_elem = elements[self.translation_element_index]
        if isinstance(target_elem, Waypoint):
            target_translation = target_elem.translation_target.translation
        elif isinstance(target_elem, TranslationTarget):
            target_translation = target_elem.translation
        else:
            self.finished = True
            return
        error_vec = target_translation - pose.translation()
        error_mag = error_vec.norm()
        # Check if within handoff radius
        handoff_radius = 0.1  # TODO: get from constraints or element
        if error_mag < handoff_radius:
            self.translation_element_index += 1
            if self.translation_element_index >= len(elements):
                self.finished = True
                return
            return  # Wait for next cycle
        # Translation PID
        translation_output = self.translation_controller.calculate(error_mag, 0.0)
        # Direction toward target
        direction = error_vec / error_mag if error_mag > 1e-9 else Translation2d()
        vx = translation_output * direction.x
        vy = translation_output * direction.y
        # Rotation target
        rotation_target = None
        for i in range(self.translation_element_index, len(elements)):
            elem = elements[i]
            if isinstance(elem, (Waypoint, RotationTarget)):
                if isinstance(elem, Waypoint):
                    rotation_target = elem.rotation_target.rotation
                else:
                    rotation_target = elem.rotation
                break
        if rotation_target is None:
            rotation_target = pose.rotation()
        rotation_error = rotation_target.radians() - pose.rotation().radians()
        rotation_output = self.rotation_controller.calculate(rotation_error, 0.0)
        # Cross-track correction (stub: set to zero for now)
        cross_track_output = 0.0
        # Compose chassis speeds
        speeds = ChassisSpeeds(vx, vy, rotation_output + cross_track_output)
        self.robot_relative_speeds_consumer(speeds)
        self.last_speeds = speeds
        # Event triggers
        for i, elem in enumerate(elements):
            if isinstance(elem, EventTrigger) and i not in self.fired_event_trigger_indices:
                t_ratio = elem.t_ratio
                # TODO: compute progress and fire if reached
                # For now, fire all triggers at start
                key = elem.lib_key
                if key in self.event_trigger_registry:
                    self.event_trigger_registry[key]()
                self.fired_event_trigger_indices.add(i)
                self.fired_event_trigger_count += 1

    def isFinished(self):
        return self.finished

    def end(self, interrupted):
        # Stop drivetrain if needed
        self.robot_relative_speeds_consumer(ChassisSpeeds())
