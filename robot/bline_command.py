"""
BLineCommand: Command-based path following for RobotPy, inspired by the BLine Java Path class.
This class is a skeleton for a command that follows a Path, with support for flipping, mirroring, constraints, and event triggers.
"""
from commands2 import Command
from typing import Callable, Optional, Union, Dict, Set, List, Tuple
from dataclasses import dataclass, field, replace
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from .path import Path, PathElement, Waypoint, TranslationTarget, RotationTarget, EventTrigger, TranslationTargetConstraint, RotationTargetConstraint
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
            cross_track_controller: PIDController,
            use_t_ratio_based_translation : bool
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
            self.use_t_ratio_based_translation_handoffs = use_t_ratio_based_translation

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
                 use_t_ratio_based_translation_handoff : bool,
                 pose_reset_consumer: Optional[Callable[[Pose2d], None]] = None,
                 ):
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

        self.use_t_ratio_based_translation_handoff = use_t_ratio_based_translation_handoff

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

    def advance_translation_targets(self, cur_pose : Pose2d):
        while True:
            if (self.translation_element_index >= len(self.path_elements_with_constraints) or not isinstance(self.path_elements_with_constraints[self.translation_element_index][0], TranslationTarget)):
                return
            next_translation_index = self.find_next_translation_target_index(self.translation_element_index + 1)
            if next_translation_index < 0:
                return
            
            current_translation_target : TranslationTarget = self.path_elements_with_constraints[self.translation_element_index][0]
            if current_translation_target.intermediate_handoff_radius_meters is not None:
                handoff_radius = current_translation_target.intermediate_handoff_radius_meters
            else:
                handoff_radius = self.path.get_default_global_constraints().getIntermediateHandoffRadius()

            current_segment = self.get_current_translation_segment_state(cur_pose)
            if not self.should_handoff_translation_target(cur_pose, current_translation_target, current_segment, handoff_radius):
                return

            self.translation_element_index = next_translation_index


    def should_handoff_translation_target(self, cur_pose : Pose2d, current_translation_target : TranslationTarget, current_segment : TranslationSegmentState, handoff_radius : float):
        distance_to_target = cur_pose.translation().distance(current_translation_target.translation)
        if current_segment.is_degenerate():
            return True
        
        if not self.use_t_ratio_based_translation_handoff:
            return distance_to_target <= handoff_radius
        
        handoff_threshold = 1.0 - (handoff_radius / current_segment.segment_length)
        handoff_threshold = max(0.0, min(1.0, handoff_threshold))
        return current_segment.segment_progress >= handoff_threshold or distance_to_target <= handoff_radius

    def find_next_translation_target_index(self, start_index : int):
        for i in range(max(start_index, 0), len(self.path_elements_with_constraints)):
            if isinstance(self.path_elements_with_constraints[i][0], TranslationTarget):
                return i
            
        return -1
    
    def find_previous_translation_target_index(self, start_index : int):
        for i in reversed(range(0, min(start_index, len(self.path_elements_with_constraints) - 1))):
            if isinstance(self.path_elements_with_constraints[i][0], TranslationTarget):
                return i
            return -1
    
    def get_translation_at_index(self, translation_index : int):
        if translation_index >= 0 and translation_index < len(self.path_elements_with_constraints) and isinstance(self.path_elements_with_constraints[translation_index][0], TranslationTarget):
            return self.path_elements_with_constraints[translation_index][0].translation
        
        return self.path_init_start_pose.translation()


    def calculate_segment_projection_t(self, segment_start : Translation2d, segment_end : Translation2d, point : Translation2d):
        dx = segment_end.X() - segment_start.X()
        dy = segment_end.Y() - segment_start.Y()
        segment_length_squared = dx * dx + dy * dy
        if segment_length_squared < 1e-6:
            return 0.0
        
        dxPoint = point.X() - segment_start.X()
        dyPoint = point.Y() - segment_start.Y()
        t = (dxPoint * dx + dyPoint * dy) / segment_length_squared
        return max(0.0, min(1.0, t))

    def get_current_translation_segment_state(self, cur_pose : Pose2d):
        if self.translation_element_index < 0 or self.translation_element_index >= len(self.path_elements_with_constraints) or not isinstance(self.path_elements_with_constraints[self.translation_element_index][0], TranslationTarget):
            current_translation = cur_pose.translation()
            return TranslationSegmentState(
                -1,
                self.translation_element_index,
                current_translation,
                current_translation,
                0.0,
                1.0
            )
        
        start_translation_index = self.find_previous_translation_target_index(self.translation_element_index - 1)
        start_translation = self.get_translation_at_index(start_translation_index) if start_translation_index >= 0 else self.path_init_start_pose.translation()
        end_translation = self.get_translation_at_index(self.translation_element_index)
        segment_length = start_translation.distance(end_translation)
        segment_progress = 1.0 if segment_length < 1e-6 else self.calculate_segment_projection_t(start_translation, end_translation, cur_pose.translation())

        return TranslationSegmentState(
            start_translation_index,
            self.translation_element_index,
            start_translation,
            end_translation,
            segment_length,
            segment_progress
        )

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
        self.advance_translation_targets(cur_pose)
        translation_handoff_occured = self.translation_element_index != previous_translation_index
        if translation_handoff_occured:
            pass # logging
        if self.translation_element_index >= len(self.path_elements_with_constraints) or not self.is_translation_target_at(self.translation_element_index):
            self.stop_commanded_motion()
            return
        
        current_segment = self.get_current_translation_segment_state(cur_pose)


        last_rotation_element_index = self.rotation_element_index
        rotation_selection = self.selectRotationTarget(current_segment)
        self.rotation_element_index = rotation_selection.active_rotation_index if rotation_selection.active_rotation_index >= 0 else self.NO_ACTIVE_ROTATION_INDEX

        if (rotation_selection.previous_rotation_index >= 0 and
            rotation_selection.previous_rotation_index != self.previous_rotation_element_index and
            isinstance(self.path_elements_with_constraints[rotation_selection.previous_rotation_index][0], RotationTarget)):
            self.previous_rotation_element_target_rad = self.path_elements_with_constraints[rotation_selection.previous_rotation_index][0].rotation.radians()
            self.previous_rotation_element_index = rotation_selection.previous_rotation_index
            self.current_rotation_target_init_rad = cur_pose.rotation().radians()

        if last_rotation_element_index != self.rotation_element_index:
            pass #log

        self.process_event_triggers()

        target_translation = self.path_elements_with_constraints[self.translation_element_index][0].translation if self.is_translation_target_at(self.translation_element_index) else cur_pose.translation()
        remaining_distance = self.calculate_remaining_path_distance()
        angle_to_target = math.atan2(
            target_translation.Y() - cur_pose.Y(),
            target_translation.X() - cur_pose.X()
        )
        if not isinstance(self.path_elements_with_constraints[self.translation_element_index][1], TranslationTargetConstraint):
            self.stop_commanded_motion()
            return
        translation_constraint : TranslationTargetConstraint = self.path_elements_with_constraints[self.translation_element_index][1]

        translation_controller_output = max(-translation_constraint.max_velocity_mps, min(translation_constraint.max_velocity_mps, -self.translation_controller.calculate(remaining_distance, 0)))

        cached_remaining_distance = remaining_distance
        vx = translation_controller_output * math.cos(angle_to_target)
        vy = translation_controller_output * math.sin(angle_to_target)


        cross_track_error = self.caclulate_cross_track_error()

    def caclulate_cross_track_error(self):
        target_translation = self.path_elements_with_constraints[self.translation_element_index][0].translation
        previous_translation = self.get_current_translation_segment_start()

        cur_pose = self.pose_supplier()
        robot_position = cur_pose.translation()

        closest_point = self.calculate_projected_point_on_segment(previous_translation, target_translation, robot_position)

        path_vector_x = target_translation.X() - previous_translation.X()
        path_vcetor_y = target

    def calculate_projected_point_on_segment(self, segment_start : Translation2d, segment_end : Translation2d, point : Translation2d):
        t = self.calculate_segment_projection_t(segment_start, segment_end, point)
        dx = segment_end.X() - segment_start.X()
        dy = segment_end.Y() - segment_start.Y()
        return Translation2d(
            segment_start.X() + t * dx,
            segment_start.Y() + t * dy
        )

    def get_current_translation_segment_start(self):
        previous_translation_index = self.find_previous_translation_target_index(self.translation_element_index - 1)
        return self.get_translation_at_index(previous_translation_index) if previous_translation_index >= 0 else self.path_init_start_pose.translation()



    def calculate_remaining_path_distance(self):
        previous_translation = self.pose_supplier().translation()
        remaining_distance = 0
        for i in range(self.translation_element_index, len(self.path_elements_with_constraints)):
            if isinstance(self.path_elements_with_constraints[i][0], TranslationTarget):
                remaining_distance += previous_translation.distance(self.path_elements_with_constraints[i][0].translation)
                previous_translation = self.path_elements_with_constraints[i][0].translation

        return remaining_distance

    def process_event_triggers(self, cur_pose : Pose2d):
        while self.event_trigger_element_index < len(self.path_elements_with_constraints):
            element = self.path_elements_with_constraints[self.event_trigger_element_index][0]
            if not isinstance(element, EventTrigger):
                self.event_trigger_element_index += 1
                continue

            if self.event_trigger_element_index in self.fired_event_trigger_indices:
                self.event_trigger_element_index += 1
                continue

            if not self.is_event_trigger_t_ratio_reached():
                break

            trigger = element
            action = self.event_trigger_registry.get(trigger.lib_key)
            if action is not None:
                action.run() # look at this
            
            self.fired_event_trigger_indices.add(self.event_trigger_element_index)
            self.fired_event_trigger_count += 1
            self.event_trigger_element_index += 1
                
    def is_event_trigger_t_ratio_reached(self, event_index : int, cur_pose : Pose2d):
        if event_index >= len(self.path_elements_with_constraints) or not isinstance(self.path_elements_with_constraints[event_index][0], EventTrigger):
            return False
        
        if self.is_event_trigger_next_segment(event_index):
            return False
        
        if self.is_event_trigger_previous_segment(event_index):
            return True
        
        translation_A = None
        translation_B = None

        for i in reversed(range(event_index)):
            if isinstance(self.path_elements_with_constraints[i][0], TranslationTarget):
                translation_A = self.path_elements_with_constraints[i][0].translation
                break

        for i in range(event_index + 1, len(self.path_elements_with_constraints)):
            if isinstance(self.path_elements_with_constraints[i][0], TranslationTarget):
                translationB = self.path_elements_with_constraints[i][0].translation
                break
        
        if translation_A is None or translation_B is None:
            return False
        
        segment_length = translation_A.distance(translation_B)
        if segment_length < 1e-6:
            return True
        
        segment_progress = self.calculate_segment_projection_t(translation_A, translation_B, cur_pose.translation())
        target_t_ratio = self.path_elements_with_constraints[event_index][0].t_ratio
        return segment_progress >= target_t_ratio

    def is_event_trigger_next_segment(self, event_index : int):
        return event_index > self.translation_element_index
    
    def is_event_trigger_previous_segment(self, event_index : int):
        if event_index > self.translation_element_index:
            return False
        for i in range(event_index, self.translation_element_index):
            if isinstance(self.path_elements_with_constraints[i][0], TranslationTarget):
                return True
            
        return False

    
    def selectRotationTarget(self, current_segment : TranslationSegmentState):
        previous_rotation_index = -1
        active_rotation_index = -1
        max_t_ratio_on_current_segment = self.get_max_t_ratio_on_segment(current_segment.end_translation_index)

        for i in range(len(self.path_elements_with_constraints)):
            if not isinstance(self.path_elements_with_constraints[i][0], RotationTarget):
                continue

            bounds = self.get_rotation_segment_bounds(i)
            if bounds is None:
                previous_rotation_index = i
                continue

            if bounds.end_translation_index < current_segment.end_translation_index:
                previous_rotation_index = i
                continue

            if bounds.end_translation_index > current_segment.end_translation_index:
                active_rotation_index = i
                break

            rotation_target = self.path_elements_with_constraints[i][0]
            target_t_ratio = self.clamp_t_ratio(rotation_target.t_ratio)
            if current_segment.is_degenerate():
                if target_t_ratio + 1e-9 < max_t_ratio_on_current_segment:
                    previous_rotation_index = i
                    continue
                active_rotation_index = i
                break

            if target_t_ratio <= current_segment.segment_progress + 1e-9:
                previous_rotation_index = i
                continue

            active_rotation_index = i
            break

        return RotationSelection(active_rotation_index, previous_rotation_index)


    def get_max_t_ratio_on_segment(self, segment_end_translation_index : int):
        max_t_ratio = -1000000000000000
        for i in range(self.path_elements_with_constraints):
            if not isinstance(self.path_elements_with_constraints[i][0], RotationTarget):
                continue

            bounds = self.get_rotation_segment_bounds(i)
            if bounds is None or bounds.end_translation_index != segment_end_translation_index:
                continue

            clamped_t_ratio = self.clamp_t_ratio(self.path_elements_with_constraints[i][0].t_ratio)
            max_t_ratio = max(max_t_ratio, clamped_t_ratio)

        return max_t_ratio
    
    def clamp_t_ratio(self, t_ratio : float):
        return max(0.0, min(1.0, t_ratio))

    def get_rotation_segment_bounds(self, rotation_index : int):
        if rotation_index < 0 or rotation_index >= len(self.path_elements_with_constraints) or not isinstance(self.path_elements_with_constraints[rotation_index][0], RotationTarget):
            return
        
        start_translation_index = self.find_previous_translation_target_index(rotation_index - 1)
        end_translation_index = self.find_next_translation_target_index(rotation_index + 1)

        if end_translation_index < 0:
            return
        
        start_translation = self.get_translation_at_index(start_translation_index) if start_translation_index >= 0 else self.path_init_start_pose.translation()
        end_translation = self.get_translation_at_index(end_translation_index)
        return RotationSegmentBounds(
            start_translation_index,
            end_translation_index,
            start_translation,
            end_translation
        )


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
