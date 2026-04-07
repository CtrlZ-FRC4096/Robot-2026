from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from dataclasses import dataclass, replace
from typing import Optional, Union


class PathElement:
    def copy(self):
        return replace(self)

class PathElementConstraint:
    pass

@dataclass(frozen=True)
class WaypointConstraint(PathElementConstraint):
     max_vel_m_s : float
     max_accel_m_s2 : float
     max_vel_deg_s : float
     max_accel_deg_s2 : float

@dataclass(frozen=True)
class TranslationTargetConstraint(PathElementConstraint):
    max_vel_m_s : float
    max_accel_m_s2 : float

@dataclass(frozen=True)
class RotationTargetConstraint(PathElementConstraint):
    max_vel_deg_s : float
    max_accel_deg_s2 : float

@dataclass(frozen=True)
class TranslationTarget(PathElement):
    translation : Translation2d
    handoff_radius : Optional[float] = None

    @classmethod
    def from_xy(cls, x: float, y : float, handoff : Optional[float] = None)
        return cls(Translation2d(x, y), handoff)
    
@dataclass(frozen=True)
class RotationTarget(PathElement):
    rotation : Rotation2d
    t_ratio : float = 1.0
    profiled : bool = True

@dataclass(frozen=True)
class Waypoint(PathElement):
    translation_target : TranslationTarget
    rotation_target : RotationTarget

    def __init__(self, translation : Union[Translation2d, Pose2d], rotation : Optional[Rotation2d] = None, handoff : Optional[float] = None, profiled : bool = True):
        if isinstance(translation, Pose2d):
            t_val = translation.translation()
            r_val = translation.rotation() if rotation is None else rotation
        else:
            t_val = translation
            r_val = rotation if rotation is not None else Rotation2d()

        object.__setattr__(self, 'translation_target', TranslationTarget(t_val, handoff))
        object.__setattr__(self, 'rotation_target', RotationTarget(r_val, 1.0, profiled))

class Path:
    def __init__(self, *elements : PathElement):
        self.elements = list(elements)
        self._is_flipped = False
    
    @classmethod
    def from_json(cls, name : str):
        # add logic to get from json here
        return cls()
    
    
