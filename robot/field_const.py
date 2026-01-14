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

    fieldLength = inchesToMeters(690.876)
    fieldWidth = inchesToMeters(317)
    startingLineX = inchesToMeters(299.438)  # Measured from the inside of starting line
    algaeDiameter = inchesToMeters(16)
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

    class Processor:
        centerFace = Pose2d(inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90))

    class Barge:
        farCage = Translation2d(
            inchesToMeters(345.428), inchesToMeters(286.779)
        )  # cage closest to the middle
        middleCage = Translation2d(inchesToMeters(345.428), inchesToMeters(242.855))
        closeCage = Translation2d(
            inchesToMeters(345.428), inchesToMeters(199.947)
        )  # cage closest to outside wall

        # from floor to bottom of cage
        deepHeight = inchesToMeters(3.125)
        shallowHeight = inchesToMeters(30.125)

    class CoralStation:
        leftCenterFace = Pose2d(
            inchesToMeters(33.526),
            inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011),
        )
        rightCenterFace = Pose2d(
            inchesToMeters(33.526),
            inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90),
        )


    class ReefHeight(Enum):
        L4 = (inchesToMeters(67), -90)
        L3 = (inchesToMeters(47.625), -35)
        L2 = (inchesToMeters(31.875), -35)
        L1 = (inchesToMeters(18), 0)

        def __init__(self, height, pitch):
            self.height = height
            self.pitch = pitch

    class Reef:
        center = Translation2d(inchesToMeters(176.746), inchesToMeters(158.501))
        tag_map = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)
        faceToZoneLine = inchesToMeters(
            12
        )  # Side of the reef to the inside of the reef zone line
        centerFaces = [
            tag_map.getTagPose(18).toPose2d(),
            tag_map.getTagPose(19).toPose2d(),
            tag_map.getTagPose(20).toPose2d(),
            tag_map.getTagPose(21).toPose2d(),
            tag_map.getTagPose(22).toPose2d(),
            tag_map.getTagPose(17).toPose2d(),
        ]  # Starting facing the driver station in clockwise order
        branchPositions = []

        for face in range(6):
            # Right and left determined from standing outside of the reef looking at the face (not from looking from the inside of reef).
            fillRight = []
            fillLeft = []
            for level in [
                (inchesToMeters(67), -90),
                (inchesToMeters(47.625), -35),
                (inchesToMeters(31.875), -35),
                (inchesToMeters(18), 0),
            ]:
                poseDirection = Pose2d(
                    center, Rotation2d.fromDegrees(180 - (60 * face))
                )
                adjustX = inchesToMeters(30.738)
                adjustY = inchesToMeters(6.469)

                fillRight.append(
                    Pose3d(
                        Translation3d(
                            poseDirection.transformBy(
                                Transform2d(adjustX, adjustY, Rotation2d())
                            ).X(),
                            poseDirection.transformBy(
                                Transform2d(adjustX, adjustY, Rotation2d())
                            ).Y(),
                            level[0],
                        ),
                        Rotation3d(
                            0,
                            degreesToRadians(level[1]),
                            poseDirection.rotation().radians(),
                        ),
                    )
                )
                fillLeft.append(
                    Pose3d(
                        Translation3d(
                            poseDirection.transformBy(
                                Transform2d(adjustX, -adjustY, Rotation2d())
                            ).X(),
                            poseDirection.transformBy(
                                Transform2d(adjustX, -adjustY, Rotation2d())
                            ).Y(),
                            level[0],
                        ),
                        Rotation3d(
                            0,
                            degreesToRadians(level[1]),
                            poseDirection.rotation().radians(),
                        ),
                    )
                )

            branchPositions.append(fillRight)
            branchPositions.append(fillLeft)

    class ReefCalibratedToField:
        """
        JSON in this format:
        {
            "red": {
                "left": {1: (), 2: (), 3: (), 4: (), 5: (), 6: ()},
                "right": {1: (), 2: (), 3: (), 4: (), 5: (), 6: ()}
            },
            "blue": {
                "left": {1: (), 2: (), 3: (), 4: (), 5: (), 6: ()},
                "right": {1: (), 2: (), 3: (), 4: (), 5: (), 6: ()}
            }
        }
        """
        # calibrated_data = {
            # "red": {
            #     "left": {1: Pose2d(14.379, 3.961, Rotation2d.fromDegrees(-89.65)), 2: Pose2d(13.662, 2.858, Rotation2d.fromDegrees(-149.45)), 3: Pose2d(12.349, 2.918, Rotation2d.fromDegrees(149.93)), 4: Pose2d(11.743, 4.087, Rotation2d.fromDegrees(89.742)), 5: Pose2d(12.462, 5.194, Rotation2d.fromDegrees(30.198)), 6: Pose2d(13.762, 5.140, Rotation2d.fromDegrees(-29.828))},
            #     "right": {1: Pose2d(14.367, 4.265, Rotation2d.fromDegrees(-88.777)), 2: Pose2d(13.919, 3.015, Rotation2d.fromDegrees(-149.969)), 3: Pose2d(12.596, 2.789, Rotation2d.fromDegrees(150.881)), 4: Pose2d(11.767, 3.798, Rotation2d.fromDegrees(91.195)), 5: Pose2d(12.210, 5.046, Rotation2d.fromDegrees(30.331)), 6: Pose2d(13.520, 5.26, Rotation2d.fromDegrees(-29.844))}
            # },
        #     "blue": {
        #         "left": {1: Pose2d(3.178, 4.081, Rotation2d.fromDegrees(90.039)), 2: Pose2d(3.898, 5.194, Rotation2d.fromDegrees(30.198)), 3: Pose2d(5.206, 5.124, Rotation2d.fromDegrees(-30.489)), 4: Pose2d(5.799, 3.974, Rotation2d.fromDegrees(-89.280)), 5: Pose2d(5.089, 2.858, Rotation2d.fromDegrees(-150.211)), 6: Pose2d(3.765, 2.936, Rotation2d.fromDegrees(148.370))},
        #         "right": {1: Pose2d(3.189, 3.797, Rotation2d.fromDegrees(89.594)), 2: Pose2d(3.645, 5.037, Rotation2d.fromDegrees(30.737)), 3: Pose2d(4.938, 5.275, Rotation2d.fromDegrees(-29.443)), 4: Pose2d(5.790, 4.269, Rotation2d.fromDegrees(-88.984)), 5: Pose2d(5.341, 3.018, Rotation2d.fromDegrees(-149.494)), 6: Pose2d(4.050, 2.775, Rotation2d.fromDegrees(149.601))}
        #     }
        # }
        calibrated_data = {
            "red": {
                "left": {1: Pose2d(14.370, 4.001, Rotation2d.fromDegrees(-89.74)), 2: Pose2d(13.662, 2.858, Rotation2d.fromDegrees(-149.45)), 3: Pose2d(12.349, 2.918, Rotation2d.fromDegrees(149.93)), 4: Pose2d(11.743, 4.087, Rotation2d.fromDegrees(89.742)), 5: Pose2d(12.462, 5.194, Rotation2d.fromDegrees(30.198)), 6: Pose2d(13.762, 5.140, Rotation2d.fromDegrees(-29.828))},
                "right": {1: Pose2d(14.389, 4.311, Rotation2d.fromDegrees(-89.81)), 2: Pose2d(13.919, 3.015, Rotation2d.fromDegrees(-149.969)), 3: Pose2d(12.596, 2.789, Rotation2d.fromDegrees(150.881)), 4: Pose2d(11.767, 3.798, Rotation2d.fromDegrees(91.195)),  5: Pose2d(12.210 + (inchesToMeters(1) * math.cos(degreesToRadians(30.331))), 5.046 + (inchesToMeters(1) * math.sin(degreesToRadians(30.331))), Rotation2d.fromDegrees(30.331)), 6: Pose2d(13.520, 5.26, Rotation2d.fromDegrees(-29.844))}
            },
            "blue": {
                "left": {1: Pose2d(3.175, 4.079, Rotation2d.fromDegrees(90.02)), 2: Pose2d(3.886 + (inchesToMeters(0.5) * math.cos(degreesToRadians(28.965))), 5.197 + (inchesToMeters(0.5) * math.sin(degreesToRadians(28.965))), Rotation2d.fromDegrees(28.965)), 3: Pose2d(5.203, 5.129, Rotation2d.fromDegrees(-29.93)), 4: Pose2d(5.801, 3.977, Rotation2d.fromDegrees(-89.766)), 5: Pose2d(5.098, 2.863, Rotation2d.fromDegrees(-149.723)), 6: Pose2d(3.777, 2.922, Rotation2d.fromDegrees(149.839))},
                "right": {1: Pose2d(3.180, 3.788 - inchesToMeters(0.5), Rotation2d.fromDegrees(90.64)), 2: Pose2d(3.642, 5.044, Rotation2d.fromDegrees(29.089)), 3: Pose2d(4.936, 5.273, Rotation2d.fromDegrees(-29.622)), 4: Pose2d(5.794, 4.246, Rotation2d.fromDegrees(-89.71)), 5: Pose2d(5.348, 3.020, Rotation2d.fromDegrees(-149.589)), 6: Pose2d(4.055, 2.775, Rotation2d.fromDegrees(150.920))}
            }
        }

    class StagingPositions:
        """Positions of the starting algae and coral on top of each other"""

        # standing at driver station facing away
        leftIceCream = Translation2d(inchesToMeters(48), inchesToMeters(230.5))
        middleIceCream = Translation2d(inchesToMeters(48), inchesToMeters(158.5))
        rightIceCream = Translation2d(inchesToMeters(48), inchesToMeters(86.5))


# TEST PRINTING FIELD CONST VALUES


## A bunch of constants to test

# print(FieldConstants.Barge.farCage)
# print(FieldConstants.ReefHeight.L4.height)
# print(FieldConstants.Reef.centerFaces[0])
# print(FieldConstants.Reef.branchPositions)
# print(range(len(FieldConstants.Reef.branchPositions)))
# print(len(FieldConstants.Reef.branchPositions))
# print(FieldConstants.Reef.branchPositions)
# print(FieldConstants.Reef.branchPositions)
# print(FieldConstants.Reef.centerFaces)

# #array of elements that are lists of pose3d's for one sector (twelfth (face + right or left branch)) goes top branch to bottom
# # right, left, right, left, ...
# print(FieldConstants.Reef.branchPositions)
# for idx in range(len(FieldConstants.Reef.branchPositions)):
#     for level in range(4):
#         reefHeightLevels = {
#              FieldConstants.ReefHeight.L4 : "4",
#              FieldConstants.ReefHeight.L3 : "3",
#              FieldConstants.ReefHeight.L2 : "2",
#              FieldConstants.ReefHeight.L1 : "1"
#         }
#         for reef_height, lvl in reefHeightLevels.items():
#              if math.isclose(FieldConstants.Reef.branchPositions[idx][level].Z(), reef_height.height, abs_tol=1e-6):
#                 branch_level = lvl

#         print("Face", ((idx // 2) + 1),
#             ", right-branch" if idx % 2 else ", left-branch",
#             ", L" + branch_level,
#             "Pitch:", FieldConstants.Reef.branchPositions[idx][level].rotation().Y(),
#             "\n Pose3d: \n", FieldConstants.Reef.branchPositions[idx][level],
#             end="\n\n"
#             )
