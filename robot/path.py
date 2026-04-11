"""
Path representation for robot trajectories in Python (RobotPy/WPILib style).
This is a translation of the BLine Java Path class, adapted for Python and RobotPy conventions.
"""
from typing import List, Optional, Callable, Tuple, Union
from dataclasses import dataclass, field, replace
from math import isclose
import typing

from wpimath.geometry import Pose2d, Rotation2d, Translation2d

import math
from enum import Enum
from typing import List
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

class FieldSymmetry(Enum):
    """Enum representing the different types of field symmetry"""
    ROTATIONAL = 0  # Red side is blue side rotated 180 degrees
    MIRRORED = 1    # Field is mirrored vertically over the center

class FlippingUtil:
    # Default field sizes (2024/2025 standard sizes)
    field_size_x = 16.541
    field_size_y = 8.069
    symmetry_type = FieldSymmetry.ROTATIONAL

    @staticmethod
    def flip_field_position(pos: Translation2d) -> Translation2d:
        """Flip a field position to the other side of the field."""
        if FlippingUtil.symmetry_type == FieldSymmetry.MIRRORED:
            return Translation2d(FlippingUtil.field_size_x - pos.X(), pos.Y())
        else:
            return Translation2d(FlippingUtil.field_size_x - pos.X(), FlippingUtil.field_size_y - pos.Y())

    @staticmethod
    def flip_field_rotation(rotation: Rotation2d) -> Rotation2d:
        """Flip a field rotation to the other side of the field."""
        if FlippingUtil.symmetry_type == FieldSymmetry.MIRRORED:
            # pi - rotation
            return Rotation2d(math.pi) - rotation
        else:
            # rotation - pi
            return rotation - Rotation2d(math.pi)

    @staticmethod
    def flip_field_pose(pose: Pose2d) -> Pose2d:
        """Flip a field pose to the other side of the field."""
        return Pose2d(
            FlippingUtil.flip_field_position(pose.translation()),
            FlippingUtil.flip_field_rotation(pose.rotation())
        )

    @staticmethod
    def mirror_field_position(pos: Translation2d) -> Translation2d:
        """Mirror a position across the field width centerline (y -> fieldSizeY - y)."""
        return Translation2d(pos.X(), FlippingUtil.field_size_y - pos.Y())

    @staticmethod
    def mirror_field_rotation(rotation: Rotation2d) -> Rotation2d:
        """Mirror a rotation across the field width centerline (theta -> -theta)."""
        return Rotation2d(-rotation.radians())

    @staticmethod
    def mirror_field_pose(pose: Pose2d) -> Pose2d:
        """Mirror a pose across the field width centerline."""
        return Pose2d(
            FlippingUtil.mirror_field_position(pose.translation()),
            FlippingUtil.mirror_field_rotation(pose.rotation())
        )

    @staticmethod
    def flip_field_speeds(field_speeds: ChassisSpeeds) -> ChassisSpeeds:
        """Flip field relative speeds for the other side of the field."""
        if FlippingUtil.symmetry_type == FieldSymmetry.MIRRORED:
            return ChassisSpeeds(
                -field_speeds.vx,
                field_speeds.vy,
                -field_speeds.omega
            )
        else:
            return ChassisSpeeds(
                -field_speeds.vx,
                -field_speeds.vy,
                field_speeds.omega
            )

    @staticmethod
    def flip_feedforwards(feedforwards: List[float]) -> List[float]:
        """Swaps feedforwards (e.g., swapping Left and Right motor values)."""
        if FlippingUtil.symmetry_type == FieldSymmetry.MIRRORED:
            if len(feedforwards) == 4:
                return [feedforwards[1], feedforwards[0], feedforwards[3], feedforwards[2]]
            elif len(feedforwards) == 2:
                return [feedforwards[1], feedforwards[0]]
        return feedforwards

    @staticmethod
    def flip_feedforward_xs(feedforward_xs: List[float]) -> List[float]:
        return FlippingUtil.flip_feedforwards(feedforward_xs)

    @staticmethod
    def flip_feedforward_ys(feedforward_ys: List[float]) -> List[float]:
        flipped = FlippingUtil.flip_feedforwards(feedforward_ys)
        if FlippingUtil.symmetry_type == FieldSymmetry.MIRRORED:
            # Invert Y directions
            return [-val for val in flipped]
        return flipped


# --- Path Element Types ---

class PathElement:
    def copy(self):
        raise NotImplementedError()

@dataclass(frozen=True)
class TranslationTarget(PathElement):
    translation: Translation2d
    intermediate_handoff_radius_meters: Optional[float] = None

    def copy(self):
        return replace(self)

    @classmethod
    def from_xy(cls, x: float, y: float, handoff_radius: Optional[float] = None):
        return cls(Translation2d(x, y), handoff_radius)

@dataclass(frozen=True)
class RotationTarget(PathElement):
    rotation: Rotation2d
    t_ratio: float = 1.0
    profiled_rotation: bool = True

    def copy(self):
        return replace(self)

@dataclass(frozen=True)
class Waypoint(PathElement):
    translation_target: TranslationTarget
    rotation_target: RotationTarget

    def copy(self):
        return Waypoint(self.translation_target.copy(), self.rotation_target.copy())

    @classmethod
    def from_pose(cls, pose: Pose2d, handoff_radius: Optional[float] = None, profiled_rotation: bool = True):
        return cls(
            TranslationTarget(pose.translation(), handoff_radius),
            RotationTarget(pose.rotation(), 1.0, profiled_rotation)
        )
    @classmethod
    def from_xyrot(cls, x: float, y: float, rot: Rotation2d, handoff_radius: Optional[float] = None, profiled_rotation: bool = True):
        return cls(
            TranslationTarget(Translation2d(x, y), handoff_radius),
            RotationTarget(rot, 1.0, profiled_rotation)
        )

@dataclass(frozen=True)
class EventTrigger(PathElement):
    t_ratio: float
    lib_key: str
    def copy(self):
        return replace(self)

# --- Constraint Types ---

@dataclass(frozen=True)
class RangedConstraint:
    value: float
    start_ordinal: int
    end_ordinal: int

@dataclass(frozen=True)
class WaypointConstraint:
    max_velocity_mps: float
    max_acceleration_mps2: float
    max_velocity_dps: float
    max_acceleration_dps2: float

@dataclass(frozen=True)
class TranslationTargetConstraint:
    max_velocity_mps: float
    max_acceleration_mps2: float

@dataclass(frozen=True)
class RotationTargetConstraint:
    max_velocity_dps: float
    max_acceleration_dps2: float

class DefaultGlobalConstraints:
    def __init__(self, 
                 max_velocity_mps : float, 
                 max_acceleration_ms2 : float, 
                 max_velocity_dps : float, 
                 max_acceleration_ds2 : float, 
                 end_translation_tolerance : float, 
                 end_rotation_tolerance : float, 
                 intermediate_handoff_radius : float):
        self.max_velocity = max_velocity_mps
        self.max_acceleration = max_acceleration_ms2
        self.max_velocity_omega = max_velocity_dps
        self.max_acceleration_omega = max_acceleration_ds2
        self.end_translation_tolerance = end_translation_tolerance
        self.end_rotation_tolerance = end_rotation_tolerance
        self.intermediate_handoff_radius = intermediate_handoff_radius

    def copy(self):
        c = DefaultGlobalConstraints(self.max_velocity, self.max_acceleration, self.max_velocity_omega, self.max_acceleration_omega, self.end_translation_tolerance, self.end_rotation_tolerance, self.intermediate_handoff_radius)
        return c
    
    def getMaxVelocityMps(self):
        return self.max_velocity
    
    def getMaxAccelMs2(self):
        return self.max_acceleration
    
    def getMaxVelocityDps(self):
        return self.max_velocity_omega
    
    def getMaxAccelerationDps2(self):
        return self.max_acceleration_omega
    
    def getEndTranslationTolerance(self):
        return self.end_translation_tolerance

    def getEndRotationTolerance(self):
        return self.end_rotation_tolerance
    
    def getIntermediateHandoffRadius(self):
        return self.intermediate_handoff_radius


class PathConstraints:
    def __init__(self, max_velocity_mps: Optional[List[RangedConstraint]] = None,
                 max_acceleration_mps2: Optional[List[RangedConstraint]] = None, 
                 max_velocity_dps: Optional[List[RangedConstraint]] = None,
                 max_acceleration_dps2: Optional[List[RangedConstraint]] = None,
                 end_translation_tolerance_m: Optional[float] = None,
                 end_rotation_tolerance_deg: Optional[float] = None):
        self.max_velocity_mps = max_velocity_mps
        self.max_acceleration_mps2 = max_acceleration_mps2
        self.max_velocity_dps = max_velocity_dps
        self.max_acceleration_dps2 = max_acceleration_dps2
        self.end_translation_tolerance = end_translation_tolerance_m
        self.end_rotation_tolerance = end_rotation_tolerance_deg

    def setMaxVelocityMps(self, constraints: List[RangedConstraint]):
        self.max_velocity_mps = list(constraints) if constraints else []
        return self

    def setMaxAccelerationMps2(self, constraints: List[RangedConstraint]):
        self.max_acceleration_mps2 = list(constraints) if constraints else []
        return self

    def setMaxVelocityDps(self, constraints: List[RangedConstraint]):
        self.max_velocity_dps = list(constraints) if constraints else []
        return self

    def setMaxAccelerationDps2(self, constraints: List[RangedConstraint]):
        self.max_acceleration_dps2 = list(constraints) if constraints else []
        return self
    
    def setEndTranslationTolerance(self, value):
        self.end_translation_tolerance = value
        return self
    
    def setEndRotationToleranceDeg(self, value):
        self.end_rotation_tolerance = value
        return self
    
    def getMaxVelocityMps(self):
        return self.max_velocity_mps
    
    def getMaxAccelerationMps2(self):
        return self.max_acceleration_mps2

    def getMaxVelocityDps(self):
        return self.max_velocity_dps

    def getMaxAccelerationDps2(self):
        return self.max_acceleration_dps2
    
    def getEndTranslationTolerance(self):
        return self.end_translation_tolerance
    
    def getEndRotationToleranceDeg(self):
        return self.end_rotation_tolerance
    
    def copy(self):
        c = PathConstraints()
        # Pass the lists directly, do not use * unpacking
        if self.max_velocity_mps is not None:
            c.setMaxVelocityMps(self.max_velocity_mps)
        if self.max_acceleration_mps2 is not None:
            c.setMaxAccelerationMps2(self.max_acceleration_mps2)
        if self.max_velocity_dps is not None:
            c.setMaxVelocityDps(self.max_velocity_dps)
        if self.max_acceleration_dps2 is not None:
            c.setMaxAccelerationDps2(self.max_acceleration_dps2)
            
        c.setEndTranslationTolerance(self.end_translation_tolerance)
        c.setEndRotationToleranceDeg(self.end_rotation_tolerance)
        return c
    

# --- Path Class ---

class Path:
    def __init__(self,
                 path_elements: List[PathElement],
                 path_constraints=None,
                #  default_global_constraints: Optional[DefaultGlobalConstraints] = None,
                 flipped = False,
                 is_valid = True):
        if path_elements is None:
            raise NameError("deez nutz")
        
        if path_constraints is None:
            path_constraints = PathConstraints()

        # if default_global_constraints is None:
        #     raise NameError("deez nuts")
        
        self.flipped = flipped
        self.isValid = is_valid

        self.path_elements = path_elements
        self.path_constraints = path_constraints
        self.default_global_constraints = DefaultGlobalConstraints(4.0, 4.5, 540.0, 720.0, 0.05, 4.0, 0.2)

        self._validate_path_endpoints()

        

    def _validate_path_endpoints(self):
        if len(self.path_elements) == 0:
            self.isValid = False
            return
        if len(self.path_elements) == 1:
            if isinstance(self.path_elements[0], RotationTarget):
                self.isValid = False
            return
        
        first = self.path_elements[0]
        last = self.path_elements[-1]
        first_valid = isinstance(first, (Waypoint, TranslationTarget))
        last_valid = isinstance(last, (Waypoint, TranslationTarget))
        if not (first_valid and last_valid):
            self.isValid = False

    def is_valid(self):
        return self.isValid

    def get_default_global_constraints(self) -> DefaultGlobalConstraints:
        return self.default_global_constraints.copy()
    
    def set_default_global_constraints(self, default_global_constraints : DefaultGlobalConstraints):
        if default_global_constraints is None:
            raise TypeError("deez nutsd")
        self.default_global_constraints = default_global_constraints.copy()

    def set_path_constraints(self, path_constraints: PathConstraints):
        if path_constraints is None:
            raise ValueError("path_constraints cannot be None")
        self.path_constraints = path_constraints.copy()

    def get_path_constraints(self) -> PathConstraints:
        return self.path_constraints.copy()

    def add_path_element(self, path_element: PathElement):
        self.path_elements.append(path_element)
        return self

    def get_element(self, index: int) -> PathElement:
        if index >= 0 and index < len(self.path_elements):
            return self.path_elements[index]
        raise ValueError("out of range")

    def set_element(self, index: int, element: PathElement):
        if index >= 0 and index < len(self.path_elements):
            self.path_elements[index] = element
            return
        raise ValueError("out of range")

    def remove_element(self, index: int) -> PathElement:
        if index >= 0 and index < len(self.path_elements):
            return self.path_elements.pop(index)
        raise ValueError("out of bounds")

    def reorder_elements(self, new_order: List[int]):
        if len(new_order) != len(self.path_elements):
            raise ValueError("New order must match elements length")
        self.path_elements = [self.path_elements[i] for i in new_order]
        return self

    def get_path_elements(self) -> List[PathElement]:
        return list(self.path_elements)

    def set_path_elements(self, path_elements: List[PathElement]):
        if path_elements is None:
            raise ValueError("path_elements cannot be None")
        self.path_elements = list(path_elements)

    def get_path_elements_with_constraints(self) -> List[Tuple[PathElement, any]]:
        """
        Pairs each PathElement with its resolved constraints based on ordinals 
        and global defaults.
        """
        if not self.is_valid():
            return []

        elements_with_constraints = []
        translation_ordinal = 0
        rotation_ordinal = 0

        # Helper function to find a value in a list of RangedConstraints
        def get_value(constraints_list: Optional[List[RangedConstraint]], ordinal: int) -> float:
            if constraints_list:
                for c in constraints_list:
                    if c.start_ordinal <= ordinal <= c.end_ordinal:
                        return c.value
            return -1.0

        for element in self.path_elements:
            if isinstance(element, Waypoint):
                # 1. Resolve values from specific path constraints
                v = get_value(self.path_constraints.max_velocity_mps, translation_ordinal)
                a = get_value(self.path_constraints.max_acceleration_mps2, translation_ordinal)
                rv = get_value(self.path_constraints.max_velocity_dps, rotation_ordinal)
                ra = get_value(self.path_constraints.max_acceleration_dps2, rotation_ordinal)

                # 2. Fallback to Global Defaults if -1
                v = v if v != -1 else self.default_global_constraints.getMaxVelocityMps()
                a = a if a != -1 else self.default_global_constraints.getMaxAccelMs2()
                rv = rv if rv != -1 else self.default_global_constraints.getMaxVelocityDps()
                ra = ra if ra != -1 else self.default_global_constraints.getMaxAccelerationDps2()

                elements_with_constraints.append((
                    element, 
                    WaypointConstraint(v, a, rv, ra)
                ))
                translation_ordinal += 1
                rotation_ordinal += 1

            elif isinstance(element, TranslationTarget):
                v = get_value(self.path_constraints.max_velocity_mps, translation_ordinal)
                a = get_value(self.path_constraints.max_acceleration_mps2, translation_ordinal)

                v = v if v != -1 else self.default_global_constraints.getMaxVelocityMps()
                a = a if a != -1 else self.default_global_constraints.getMaxAccelMs2()

                elements_with_constraints.append((
                    element, 
                    TranslationTargetConstraint(v, a)
                ))
                translation_ordinal += 1

            elif isinstance(element, RotationTarget):
                rv = get_value(self.path_constraints.max_velocity_dps, rotation_ordinal)
                ra = get_value(self.path_constraints.max_acceleration_dps2, rotation_ordinal)

                rv = rv if rv != -1 else self.default_global_constraints.getMaxVelocityDps()
                ra = ra if ra != -1 else self.default_global_constraints.getMaxAccelerationDps2()

                elements_with_constraints.append((
                    element, 
                    RotationTargetConstraint(rv, ra)
                ))
                rotation_ordinal += 1

            elif isinstance(element, EventTrigger):
                # Event triggers don't have motion constraints
                elements_with_constraints.append((element, None))

        return elements_with_constraints
    
    def get_path_elements_with_constraints_no_waypoints(self):
        if not self.is_valid():
            return []
        
        elements_with_constraints = self.get_path_elements_with_constraints()
        out = []
        
        for i, (element, constraint) in enumerate(elements_with_constraints):
            if isinstance(element, Waypoint):

                translation_target = element.translation_target
                rotation_target = element.rotation_target

                translation_target_constraint = TranslationTargetConstraint(
                    constraint.max_velocity_mps, 
                    constraint.max_acceleration_mps2
                )
                rotation_target_constraint = RotationTargetConstraint(
                    constraint.max_velocity_dps, 
                    constraint.max_acceleration_dps2
                )
                
                if i == 0:
                    initial_rotation = RotationTarget(
                        rotation_target.rotation,
                        0.0,
                        rotation_target.profiled_rotation
                    )
                    
                    out.append((translation_target, translation_target_constraint))
                    out.append((initial_rotation, rotation_target_constraint))
                else:
                    standard_rotation = RotationTarget(
                        rotation_target.rotation,
                        1.0,
                        rotation_target.profiled_rotation
                    )
                    out.append((standard_rotation, rotation_target_constraint))
                    out.append((translation_target, translation_target_constraint))
            else:
                out.append((element, constraint))
                
        return out
    
    def flip(self):
        if not self.is_valid() or self.flipped:
            return

        # Save previous symmetry and force rotational for the flipping calculation
        previous_symmetry_type = FlippingUtil.symmetry_type
        FlippingUtil.symmetry_type = FieldSymmetry.ROTATIONAL

        try:
            new_elements = []
            for element in self.path_elements:
                if isinstance(element, TranslationTarget):
                    new_elements.append(TranslationTarget(
                        FlippingUtil.flip_field_position(element.translation),
                        element.intermediate_handoff_radius_meters
                    ))
                
                elif isinstance(element, RotationTarget):
                    new_elements.append(RotationTarget(
                        FlippingUtil.flip_field_rotation(element.rotation),
                        element.t_ratio,
                        element.profiled_rotation
                    ))
                
                elif isinstance(element, Waypoint):
                    # Flip both sub-targets of the Waypoint
                    flipped_translation = TranslationTarget(
                        FlippingUtil.flip_field_position(element.translation_target.translation),
                        element.translation_target.intermediate_handoff_radius_meters
                    )
                    flipped_rotation = RotationTarget(
                        FlippingUtil.flip_field_rotation(element.rotation_target.rotation),
                        element.rotation_target.t_ratio,
                        element.rotation_target.profiled_rotation
                    )
                    new_elements.append(Waypoint(flipped_translation, flipped_rotation))
                
                elif isinstance(element, EventTrigger):
                    # Event triggers are ratio-based, so their position doesn't 
                    # change, but we create a new instance to keep it immutable
                    new_elements.append(EventTrigger(
                        element.t_ratio,
                        element.lib_key
                    ))
                else:
                    # Fallback for unknown element types
                    new_elements.append(element)
            
            self.path_elements = new_elements
            self.flipped = True
        finally:
            FlippingUtil.symmetry_type = previous_symmetry_type

    def mirror(self) -> None:
        """
        Mirrors this path vertically across the field centerline.
        
        This mirrors across the field width (horizontal centerline), where
        y -> field_size_y - y and x is unchanged, via FlippingUtil.
        """
        if not self.is_valid():
            return

        mirrored_elements = []
        for element in self.path_elements:
            if isinstance(element, TranslationTarget):
                mirrored_elements.append(TranslationTarget(
                    FlippingUtil.mirror_field_position(element.translation),
                    element.intermediate_handoff_radius_meters
                ))
                
            elif isinstance(element, RotationTarget):
                mirrored_elements.append(RotationTarget(
                    FlippingUtil.mirror_field_rotation(element.rotation),
                    element.t_ratio,
                    element.profiled_rotation
                ))
                
            elif isinstance(element, Waypoint):
                mirrored_translation = TranslationTarget(
                    FlippingUtil.mirror_field_position(element.translation_target.translation),
                    element.translation_target.intermediate_handoff_radius_meters
                )
                mirrored_rotation = RotationTarget(
                    FlippingUtil.mirror_field_rotation(element.rotation_target.rotation),
                    element.rotation_target.t_ratio,
                    element.rotation_target.profiled_rotation
                )
                mirrored_elements.append(Waypoint(mirrored_translation, mirrored_rotation))
                
            elif isinstance(element, EventTrigger):
                # Event triggers just hold ratios and keys, no field coordinates
                mirrored_elements.append(EventTrigger(
                    element.t_ratio,
                    element.lib_key
                ))
            else:
                # Fallback for unknown elements that implement copy()
                mirrored_elements.append(element.copy() if hasattr(element, 'copy') else element)
                
        self.path_elements = mirrored_elements

    def undo_flip(self):
        if not self.is_valid() or not self.flipped:
            return
        self.flipped = False
        self.flip()
        self.flipped = False

    def get_start_pose(self, fallback_rotation: Optional[Rotation2d] = None) -> Pose2d:
        """
        Gets the starting pose for this path.
        
        The translation comes from the first translation target. The rotation comes
        from the first rotation target, or falls back to the provided rotation 
        (or a default rotation of 0) if none exists.
        
        Args:
            fallback_rotation: The rotation to use if no rotation target is found.
                               Defaults to Rotation2d() (0 degrees).
        
        Returns:
            The starting pose (Pose2d).
            
        Raises:
            RuntimeError: If the path is invalid, empty, or has no translation targets.
        """
        # Handle the default 0-degree rotation if no fallback is provided
        if fallback_rotation is None:
            fallback_rotation = Rotation2d()

        if not self.is_valid():
            raise RuntimeError("Path invalid - cannot compute start pose")

        # elements is a list of tuples: (PathElement, PathElementConstraint)
        elements = self.get_path_elements_with_constraints_no_waypoints()
        
        if not elements:
            raise RuntimeError("Path must contain at least one element")

        # 1. Find the first TranslationTarget
        reset_translation = None
        for element, constraint in elements:
            if isinstance(element, TranslationTarget):
                reset_translation = element.translation
                break
                
        if reset_translation is None:
            raise RuntimeError("Path must contain at least one translation target")

        # 2. Find the first RotationTarget
        reset_rotation = fallback_rotation
        for element, constraint in elements:
            if isinstance(element, RotationTarget):
                reset_rotation = element.rotation
                break

        return Pose2d(reset_translation, reset_rotation)

    def get_initial_module_direction(self, pose_supplier: Optional[Callable[[], Pose2d]] = None) -> Rotation2d:
        """
        Gets the initial direction the robot's modules should face when starting this path.
        
        It is highly recommended to pre-orient swerve modules toward this direction 
        before the start of an autonomous routine to prevent micro-deviations.
        
        Args:
            pose_supplier: A callable that returns the robot's current Pose2d. 
                          If None, it defaults to using self.get_start_pose().
                          
        Returns:
            The initial module direction relative to the robot's rotation.
        """
        # If no supplier is provided, default to the path's own starting pose
        if pose_supplier is None:
            robot_pose = self.get_start_pose()
        else:
            robot_pose = pose_supplier()
        
        if not self.is_valid():
            return Rotation2d(0)

        # Extract translation targets
        elements = self.get_path_elements_with_constraints_no_waypoints()
        translation_targets = []
        for element, _ in elements:
            if isinstance(element, TranslationTarget):
                translation_targets.append(element)


        if not translation_targets:
            return Rotation2d(0)
        

        robot_trans = robot_pose.translation()
        
        # Logic to find the best target to point at
        selected_target = translation_targets[-1] # Default to last target
        
        if len(translation_targets) > 1:
            for target in translation_targets:
                # Use handoff radius if it exists, otherwise use global default
                handoff_radius = target.intermediate_handoff_radius_meters
                if handoff_radius is None:
                    handoff_radius = self.default_global_constraints.getIntermediateHandoffRadius()
                
                distance_to_target = robot_trans.distance(target.translation)
                
                # If we are outside this target's "zone", point at it
                if distance_to_target > handoff_radius:
                    selected_target = target
                    break
        else:
            selected_target = translation_targets[0]

        # Calculate vector from robot to target
        delta_x = selected_target.translation.X() - robot_trans.X()
        delta_y = selected_target.translation.Y() - robot_trans.Y()
        
        # Calculate angle of travel and subtract robot's rotation to get module-relative angle
        angle_to_target = Rotation2d(delta_x, delta_y)
        return angle_to_target - robot_pose.rotation()

    def get_end_translation_tolerance_m(self) -> float:
        if self.path_constraints.end_translation_tolerance is not None:
            return self.path_constraints.end_translation_tolerance
        else:
            return self.default_global_constraints.end_translation_tolerance

    def get_end_rotation_tolerance_deg(self) -> float:
        if self.path_constraints.end_rotation_tolerance is not None:
            return self.path_constraints.end_rotation_tolerance
        else:
            return self.default_global_constraints.end_rotation_tolerance

    def copy(self):
        deep_copied_elements = []
        for element in self.path_elements:
            deep_copied_elements.append(element.copy())

        copied_path = Path(deep_copied_elements, self.path_constraints.copy(), self.default_global_constraints.copy())
        copied_path.flipped = self.flipped
        copied_path.isValid = self.isValid
        return copied_path