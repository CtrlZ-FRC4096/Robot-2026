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
    def __init__(self):
        # AprilTag related constants
        self.tag_map = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

        
        self.fieldLength = self.tag_map.getFieldLength()
        self.fieldWidth = self.tag_map.getFieldWidth()
        self.fuelDiameter = 0.15 # meters
        self.shouldFlip = True

        
        self.aprilTagCount = len(self.tag_map.getTags())
        self.aprilTagWidth = inchesToMeters(6.5)

    # Will need to update tags. Reference 2024 code

    def flip_X_coord(self, x):
        return self.fieldLength - x if self.shouldFlip else x

    def flip_Y_coord(self, y):
        return self.fieldWidth - y if self.shouldFlip else y

    def flip_Translation2d(self, translation):
        return (
            Translation2d(
                self.flip_X_coord(translation.X()),
                self.flip_Y_coord(translation.Y()),
            )
            if self.shouldFlip
            else translation
        )

    def flip_Rotation2d(self, rotation):
        return (
            rotation.rotateBy(Rotation2d.fromDegrees(180))
            if self.shouldFlip
            else rotation
        )

    def flip_Pose2d(self, pose):
        return (
            Pose2d(
                self.flip_Translation2d(pose.translation()),
                self.flip_Rotation2d(pose.rotation()),
            )
            if self.shouldFlip
            else pose
        )
    
    @property
    def Hub(self):
        class _Hub:
            width = inchesToMeters(47.0)
            height = inchesToMeters(72.0)
            innerWidth = inchesToMeters(41.7)
            innerHeight = inchesToMeters(56.5)
            topCenterPoint = Translation3d(self.tag_map.getTagPose(26).X() + width / 2.0,
                                           self.fieldWidth / 2.0,
                                           height)
            innerCenterPoint = Translation3d(self.tag_map.getTagPose(26).X() + width / 2.0,
                                             self.fieldWidth / 2.0,
                                             innerHeight)
            nearLeftCorner = Translation2d(topCenterPoint.X() - width / 2.0, self.fieldWidth / 2.0 + width / 2.0)
            nearRightCorner = Translation2d(topCenterPoint.X() - width / 2.0, self.fieldWidth / 2.0 - width / 2.0)
            farLeftCorner = Translation2d(topCenterPoint.X() + width / 2.0, self.fieldWidth / 2.0 + width / 2.0)
            farRightCorner = Translation2d(topCenterPoint.X() + width / 2.0, self.fieldWidth / 2.0 - width / 2.0)
            oppTopCenterPoint = Translation3d(self.tag_map.getTagPose(4).X() + width / 2.0,
                                              self.fieldWidth / 2.0,
                                              height)
            oppNearLeftCorner = Translation2d(oppTopCenterPoint.X() - width / 2.0, self.fieldWidth / 2.0 + width / 2.0)
            oppNearRightCorner = Translation2d(oppTopCenterPoint.X() - width / 2.0, self.fieldWidth / 2.0 - width / 2.0)
            oppFarLeftCorner = Translation2d(oppTopCenterPoint.X() + width / 2.0, self.fieldWidth / 2.0 + width / 2.0)
            oppFarRightCorner = Translation2d(oppTopCenterPoint.X() + width / 2.0, self.fieldWidth / 2.0 - width / 2.0)

            nearFace = self.tag_map.getTagPose(26).toPose2d()
            farFace = self.tag_map.getTagPose(20).toPose2d()
            rightFace = self.tag_map.getTagPose(18).toPose2d()
            leftFace = self.tag_map.getTagPose(21).toPose2d()
        return _Hub
    
    @property
    def Tower(self):
        class _Tower:
            # Dimensions
            width = inchesToMeters(49.25)
            depth = inchesToMeters(45.0)
            height = inchesToMeters(78.25)
            innerOpeningWidth = inchesToMeters(32.250)
            frontFaceX = inchesToMeters(43.51)
            uprightHeight = inchesToMeters(72.1)
            # Rung heights
            lowRungHeight = inchesToMeters(27.0)
            midRungHeight = inchesToMeters(45.0)
            highRungHeight = inchesToMeters(63.0)
            # Reference points - alliance
            centerPoint = Translation2d(
                frontFaceX,
                self.tag_map().getTagPose(31).get().Y()
            )
            leftUpright = Translation2d(
                frontFaceX,
                self.tag_map().getTagPose(31).get().Y()
                + innerOpeningWidth / 2
                + inchesToMeters(0.75)
            )
            rightUpright = Translation2d(
                frontFaceX,
                self.tag_map().getTagPose(31).get().Y()
                - innerOpeningWidth / 2
                - inchesToMeters(0.75)
            )
            # Reference points - opponent
            oppCenterPoint = Translation2d(
                self.fieldLength - frontFaceX,
                self.tag_map().getTagPose(15).get().Y()
            )
            oppLeftUpright = Translation2d(
                self.fieldLength - frontFaceX,
                self.tag_map().getTagPose(15).get().Y()
                + innerOpeningWidth / 2
                + inchesToMeters(0.75)
            )
            oppRightUpright = Translation2d(
                self.fieldLength - frontFaceX,
                self.tag_map().getTagPose(15).get().Y()
                - innerOpeningWidth / 2
                - inchesToMeters(0.75)
            )
        return _Tower
    
    def Output(self):
        class _Output:
            # Dimensions
            width = inchesToMeters(31.8)
            openingDistanceFromFloor = inchesToMeters(28.1)
            height = inchesToMeters(7.0)
            # Reference points 
            center = Translation2d(self.tag_map.getTagPose(29).X())

        return _Output
    @property
    def LeftBump(self):
        class _LeftBump:
            width = inchesToMeters(73.0)
            height = inchesToMeters(6.513)
            depth = inchesToMeters(44.4)
            nearLeftCorner = Translation2d(self.LinesVertical)
        return _LeftBump

    @property
    def RightBump(self):
        class _RightBump:
            width = inchesToMeters(73.0)
            height = inchesToMeters(6.513)
            depth = inchesToMeters(44.4)

            nearLeftCorner = Translation2d(self.LinesVertical.hubCenter + width / 2, inchesToMeters(255))
            nearRightCorner = self.Hub.nearLeftCorner
            farLeftCorner = Translation2d(self.LinesVertical.hubCenter - width / 2, inchesToMeters(255))
            farRightCorner = self.Hub.farLeftCorner

            oppNearLeftCorner = Translation2d(self.LinesVertical.hubCenter + width /2, inchesToMeters(255))
            oppNearRightCorner = self.Hub.oppNearLeftCorner
            oppFarLeftCorner = Translation2d(self.LinesVertical.hubCenter - width / 2, inchesToMeters(255))
            oppFarRightCorner = self.Hub.oppFarLeftCorner

        return _RightBump

    @property
    def LinesVertical(self):
        class _LinesVertical:
            center = self.fieldLength / 2.0
            starting = self.tag_map.getTagPose(26).X()
            allianceZone = starting
            hubCenter = self.Hub.width / 2.0 + starting # Note to self: Define Hub Class later on
            neutralZoneNear = center - inchesToMeters(120)
            neutralZoneFar = center + inchesToMeters(120)
            oppHubCenter = self.tag_map.getTagPose(4).X() + self.Hub.width / 2.0
            oppAllianceZone = self.tag_map.getTagPose(10).X()
        return _LinesVertical
    
    @property
    def LinesHorizontal(self):
        class _LinesHorizontal:
            center = self.fieldWidth / 2.0
            rightBumpStart = self.Hub.nearRightCorner.Y()
            rightBumpEnd = rightBumpStart - RightBump.width
            rightTrenchOpenStart = rightBumpEnd - inchesToMeters(12.0)
            rightTrenchOpenEnd = 0.0

            leftBumpEnd = Hub.nearLeftCorner.Y()
            leftBumpStart = leftBumpEnd + LeftBump.width
            leftTrenchOpenEnd = leftBumpStart + inchesToMeters(12.0)
            leftTrenchOpenStart = self.fieldWidth
        return _LinesHorizontal
    


fieldConstants = FieldConstants()
# print(fieldConstants.LinesVertical.center)  