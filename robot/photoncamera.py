import wpilib
from wpimath.units import feetToMeters, inchesToMeters
from photonlibpy.photonCamera import (
    PhotonCamera,
    setVersionCheckEnabled,
)  # VisionLEDMode
from photonlibpy import photonPoseEstimator
from wpimath.geometry import (
    Pose2d,
    Pose3d,
    Translation2d,
    Translation3d,
    Rotation2d,
    Rotation3d,
    Transform3d,
    Transform2d,
)

import const
from wpilib import DriverStation, SmartDashboard, Timer, Field2d

from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from field_const import FieldConstants

import numpy as np
import math

# Distance threshold in meters - tags farther than this are ignored
DISTANCE_THRESHOLD = 4.0
# Ambiguity threshold - tags with higher ambiguity are ignored
AMBIGUITY_THRESHOLD = 0.3


## Code from 1736
# Describes one on-field pose estimate from the a camera at a specific time.
class CameraPoseObservation:
    def __init__(self, time, estFieldPose, trustworthiness=1.0):
        self.time = time
        self.estFieldPose = estFieldPose
        self.trustworthiness = trustworthiness  # TODO - not used yet


# Wrappers photonvision to:
# 1 - resolve issues with target ambiguity (two possible poses for each observation)
# 2 - Convert pose estimates to the field
# 3 - Handle recording latency of when the image was actually seen
class WrapperedPhotonCameraTag:
    def __init__(self, camName, robotToCam):
        self.cam = PhotonCamera(camName)

        self.camName = camName
        self.timeoutSec = 1.0
        self.poseEstimates : list[Pose2d] = []
        self.zEstimates = []
        self.robotToCam: Transform3d = robotToCam
        self.counter = 0
        self.tag_map = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

    @staticmethod
    def tgt_corner_to_list(target):
        return [target.x, target.y]

    def update(
        self,
        prevEstPoseSingleTag: Pose2d,
    ):
        self.poseEstimates = []
        self.zEstimates = []
        self.tagPositions = []
        self.tagAmbiguity = []
        self.poseSingleTag = []
        self.singleTagIDs = []
        self.tagDistances = []

        # Get all unread results and take only the LATEST one
        # This discards older queued results to minimize latency
        all_results = self.cam.getAllUnreadResults()
        
        # Only process if there are results available
        if len(all_results) == 0:
            return
        
        # Take the latest (most recent) result, discard older ones
        res = all_results[-1]
        
        self.obsTime = res.getTimestampSeconds()

        # Process each target from the latest result, filtering by distance and ambiguity
        for target in res.getTargets():
            tgtID = target.getFiducialId()
            tagFieldPose = self.tag_map.getTagPose(tgtID)

            target_x_angle = math.radians(target.getYaw())
            target_y_angle = -1 * math.radians(target.getPitch())

            distance_3d = target.getBestCameraToTarget().translation().norm()
 
            # Filter: skip tags that are too far or too ambiguous
            if distance_3d > DISTANCE_THRESHOLD:
                continue
            
            ambiguity = target.getPoseAmbiguity()
            if ambiguity > AMBIGUITY_THRESHOLD:
                continue

            distance_2d_to_tag = distance_3d * math.cos(
                (-1 * self.robotToCam.rotation().Y()) - target_y_angle
            )

            # Calculate the rotation of the camera to the tag
            cam_to_tag_rotation = Rotation2d(
                prevEstPoseSingleTag.rotation().radians()
                + self.robotToCam.rotation().Z()
                - target_x_angle
            )
            
            # Calculate the translation of the camera to the tag
            field_to_camera_translation = (
                Pose2d(
                    tagFieldPose.toPose2d().translation(),
                    Rotation2d(cam_to_tag_rotation.radians() + math.pi),
                )
                .transformBy(
                    Transform2d(
                        Translation2d(distance_2d_to_tag, 0.0), Rotation2d()
                    )
                )
                .translation()
            )

            # Calculate the pose of the robot
            robot_pose = Pose2d(
                field_to_camera_translation,
                Rotation2d(
                    prevEstPoseSingleTag.rotation().radians()
                    + self.robotToCam.rotation().Z()
                ),
            ).transformBy(
                Transform2d(
                    Pose2d(
                        self.robotToCam.X(),
                        self.robotToCam.Y(),
                        self.robotToCam.rotation().Z(),
                    ),
                    Pose2d(),
                )
            )
            robot_pose = Pose2d(
                robot_pose.translation(), prevEstPoseSingleTag.rotation()
            )
            zEst = tagFieldPose.Z() - self.robotToCam.Z() - math.sin(self.robotToCam.rotation().Y() +  target_y_angle) * distance_3d
            self.zEstimates.append(zEst)
            self.poseSingleTag.append([robot_pose, tgtID, ambiguity, zEst])
            self.singleTagIDs.append(tgtID)
            self.tagDistances.append(distance_3d)

    def getObsTime(self):
        return self.obsTime
    
    def getZEstimates(self):
        return self.zEstimates

    def getTagIds(self):
        return self.tag_ids

    def getPoseEstimates(self):
        return self.poseEstimates

    def getTagPositions(self):
        return self.tagPositions

    def getTagAmbiguity(self):
        return self.tagAmbiguity

    def getPoseSingleTag(self):
        return self.poseSingleTag

    def getSingleTagIDs(self):
        return self.singleTagIDs
    
    def getTagDistances(self):
        return self.tagDistances

    def _toFieldPose(self, tgtPose: Pose3d, camToTarget: Transform3d):
        camPose = tgtPose.transformBy(camToTarget.inverse())
        return camPose.transformBy(self.robotToCam.inverse()).toPose2d()

    # Returns true of a pose is on the field, false if it's outside of the field perimieter
    @staticmethod
    def _poseIsOnField(self, pose: Pose2d):
        trans = pose.translation()
        x = trans.X()
        y = trans.Y()
        inY = -0.5 < y < FieldConstants.fieldWidth + 0.5
        inX = -0.5 < x < FieldConstants.fieldLength + 0.5
        return inX and inY