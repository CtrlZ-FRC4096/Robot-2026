"""
Path representation for robot trajectories in Python (RobotPy/WPILib style).
This is a translation of the BLine Java Path class, adapted for Python and RobotPy conventions.
"""
from typing import List, Optional, Callable, Tuple, Union
from dataclasses import dataclass, field, replace
from math import isclose

from wpimath.geometry import Pose2d, Rotation2d, Translation2d

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

@dataclass
class DefaultGlobalConstraints:
    max_velocity_mps: float
    max_acceleration_mps2: float
    max_velocity_dps: float
    max_acceleration_dps2: float
    end_translation_tolerance_m: float
    end_rotation_tolerance_deg: float
    intermediate_handoff_radius_m: float

    def copy(self):
        return replace(self)

@dataclass
class PathConstraints:
    max_velocity_mps: Optional[List[RangedConstraint]] = None
    max_acceleration_mps2: Optional[List[RangedConstraint]] = None
    max_velocity_dps: Optional[List[RangedConstraint]] = None
    max_acceleration_dps2: Optional[List[RangedConstraint]] = None
    end_translation_tolerance_m: Optional[float] = None
    end_rotation_tolerance_deg: Optional[float] = None

    def copy(self):
        return replace(self,
            max_velocity_mps=list(self.max_velocity_mps) if self.max_velocity_mps else None,
            max_acceleration_mps2=list(self.max_acceleration_mps2) if self.max_acceleration_mps2 else None,
            max_velocity_dps=list(self.max_velocity_dps) if self.max_velocity_dps else None,
            max_acceleration_dps2=list(self.max_acceleration_dps2) if self.max_acceleration_dps2 else None
        )

# --- Path Class ---

class Path:
    default_global_constraints: Optional[DefaultGlobalConstraints] = None

    def __init__(self,
                 path_elements: List[PathElement],
                 constraints: Optional[PathConstraints] = None,
                 default_global_constraints: Optional[DefaultGlobalConstraints] = None):
        if path_elements is None:
            raise ValueError("path_elements cannot be None")
        self.path_elements = list(path_elements)
        self.path_constraints = constraints.copy() if constraints else PathConstraints()
        if default_global_constraints:
            Path.default_global_constraints = default_global_constraints.copy()
        if Path.default_global_constraints is None:
            raise RuntimeError("Default global constraints must be set before creating a Path")
        self.flipped = False
        self.is_valid = True
        self._validate_path_endpoints()

    def _validate_path_endpoints(self):
        if not self.path_elements:
            self.is_valid = False
            return
        if len(self.path_elements) == 1:
            if isinstance(self.path_elements[0], RotationTarget):
                self.is_valid = False
            return
        first = self.path_elements[0]
        last = self.path_elements[-1]
        first_valid = isinstance(first, (Waypoint, TranslationTarget))
        last_valid = isinstance(last, (Waypoint, TranslationTarget))
        if not (first_valid and last_valid):
            self.is_valid = False

    def is_valid_path(self) -> bool:
        return self.is_valid

    def get_default_global_constraints(self) -> DefaultGlobalConstraints:
        return Path.default_global_constraints.copy()

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
        return self.path_elements[index]

    def set_element(self, index: int, element: PathElement):
        self.path_elements[index] = element

    def remove_element(self, index: int) -> PathElement:
        return self.path_elements.pop(index)

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

    def get_end_translation_tolerance_m(self) -> float:
        return self.path_constraints.end_translation_tolerance_m \
            if self.path_constraints.end_translation_tolerance_m is not None \
            else Path.default_global_constraints.end_translation_tolerance_m

    def get_end_rotation_tolerance_deg(self) -> float:
        return self.path_constraints.end_rotation_tolerance_deg \
            if self.path_constraints.end_rotation_tolerance_deg is not None \
            else Path.default_global_constraints.end_rotation_tolerance_deg

    def copy(self):
        return Path(
            [e.copy() for e in self.path_elements],
            self.path_constraints.copy(),
            Path.default_global_constraints.copy()
        )

    # Additional methods for constraints, flipping, mirroring, etc. can be added as needed.

# Example usage:
# Path.default_global_constraints = DefaultGlobalConstraints(
#     max_velocity_mps=3.0,
#     max_acceleration_mps2=2.0,
#     max_velocity_dps=180.0,
#     max_acceleration_dps2=90.0,
#     end_translation_tolerance_m=0.05,
#     end_rotation_tolerance_deg=2.0,
#     intermediate_handoff_radius_m=0.2
# )
# path = Path([
#     Waypoint.from_xyrot(1, 1, Rotation2d.fromDegrees(0)),
#     TranslationTarget.from_xy(2, 2),
#     RotationTarget(Rotation2d.fromDegrees(90), 0.5),
#     Waypoint.from_xyrot(3, 1, Rotation2d.fromDegrees(180))
# ])
