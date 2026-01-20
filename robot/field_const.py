from typing import TYPE_CHECKING

import math
from enum import Enum

from wpimath.geometry import (
    Rotation2d,
    Rotation3d,
    Translation2d,
    Translation3d,
    Pose2d,
    Pose3d,
    Transform2d,
)
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.units import inchesToMeters, degreesToRadians

from wpilib import DriverStation
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
import json


class FieldConstants:
    """
    These are field constants and positions from the blue alliance side.
    (0,0): When standing at the blue driver stations, (0,0) is to the right and back (is the extension of Driver Station wall and processor wall)
    Y-axis: across the width of field
    X-axis: down the length
    """

    fieldLength = inchesToMeters(651.22)
    fieldWidth = inchesToMeters(317.69)
    startingLineX = inchesToMeters(156.61)  # Measured from the inside of starting line
    algaeDiameter = inchesToMeters(16)
    #algae is not a thing in this game
    shouldFlip = DriverStation.getAlliance() == DriverStation.Alliance.kRed
    reef_tags = {6, 7, 8, 9, 10, 11} if shouldFlip else {17, 18, 19, 20, 21, 22}
    face_to_tag = (
        {1: 7, 2: 6, 3: 11, 4: 10, 5: 9, 6: 8}
        if shouldFlip
        else {1: 18, 2: 19, 3: 20, 4: 21, 5: 22, 6: 17}
    )
    tag_to_face = (
        {7: 1, 6: 2, 11: 3, 10: 4, 9: 5, 8: 6}
        if shouldFlip
        else {18: 1, 19: 2, 20: 3, 21: 4, 22: 5, 17: 6}
    )

    @staticmethod
    def flip_X_coord(x):
        return FieldConstants.fieldLength - x if FieldConstants.shouldFlip else x

    @staticmethod
    def flip_Y_coord(y):
        return FieldConstants.fieldWidth - y if FieldConstants.shouldFlip else y

    @staticmethod
    def flip_Translation2d(translation):
        return (
            Translation2d(
                FieldConstants.flip_X_coord(translation.X()),
                FieldConstants.flip_Y_coord(translation.Y()),
            )
            if FieldConstants.shouldFlip
            else translation
        )

    @staticmethod
    def flip_Rotation2d(rotation):
        return (
            rotation.rotateBy(Rotation2d.fromDegrees(180))
            if FieldConstants.shouldFlip
            else rotation
        )

    @staticmethod
    def flip_Pose2d(pose):
        return (
            Pose2d(
                FieldConstants.flip_Translation2d(pose.translation()),
                FieldConstants.flip_Rotation2d(pose.rotation()),
            )
            if FieldConstants.shouldFlip
            else pose
        )