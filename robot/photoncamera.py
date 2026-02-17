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
# import cv2

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
        # setVersionCheckEnabled(False)

        self.cam = PhotonCamera(camName)
        # TODO is this really the name of the camera or is this just as a reminder? Camera1,2,3,or 4??
        self.cameraDistortVector = const.CAM_DICT[camName][0]
        self.cameraIntrinsMatrix = const.CAM_DICT[camName][1]

        self.camName = camName
        self.timeoutSec = 1.0
        self.poseEstimates = []
        self.robotToCam: Transform3d = robotToCam
        self.counter = 0
        self.tag_map = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

    @staticmethod
    def tgt_corner_to_list(target):
        return [target.x, target.y]

    def update(
        self,
        prevEstPoseSingleTag: Pose2d,
        gyro_rotation : Rotation3d
    ):
        # self.counter += 1
        self.poseEstimates = []
        self.tagPositions = []
        self.tagAmbiguity = []
        self.poseSingleTag = []
        self.singleTagIDs = []
        self.tagDistances = []
        # if (self.counter % 20 == 0):
        #     if not self.cam.isConnected():
        #         # Faulted - no estimates, just return.
        #         print("Camera not connected")
        #        pass
        #        return

        # Grab whatever the camera last reported for observations in a camera frame
        # Note: Results simply report "I processed a frame". There may be 0 or more targets seen in a frame
        res = self.cam.getLatestResult()

        # MiniHack - results also have a more accurate "getTimestamp()", but this is
        # broken in photonvision 2.4.2. Hack with the non-broken latency calcualtion
        # latency = res.getLatencyMillis()
        # obsTime = wpilib.Timer.getFPGATimestamp() - latency

        self.obsTime = res.getTimestampSeconds()

                ## Single Tag Code
        # Process each target.
        # Each target has multiple solutions for where you could have been at on the field
        # when you observed it
        # (https://docs.wpilib.org/en/stable/docs/software/vision-processing/
        # apriltag/apriltag-intro.html#d-to-3d-ambiguity)
        # We want to select the best possible pose per target
        # We should also filter out targets that are too far away, and poses which
        # don't make sense.

        for target in res.getTargets():

            # Transform both poses to on-field poses
            tgtID = target.getFiducialId()

            tagFieldPose = self.tag_map.getTagPose(tgtID)

            # tgt_to_camera = target.getBestCameraToTarget().inverse()
            # camera_to_robot = self.robotToCam.inverse()
            # robot_pose = tagFieldPose.transformBy(tgt_to_camera).transformBy(camera_to_robot).transformBy(Transform3d(Translation3d(), Rotation3d(gyro_rotation.X(), gyro_rotation.Y(), 0)))

            target_x_angle = math.radians(target.getYaw())
            target_y_angle = -1 * math.radians(target.getPitch())

            distance_3d = target.getBestCameraToTarget().translation().norm()
 
            distance_2d_to_tag = distance_3d * math.cos(
                (-1 * self.robotToCam.rotation().Y()) - target_y_angle
            )  # cosine is even so we don't need to negate both

            # Calculate the rotation of the camera to the tag. The rotation of the robot + rotation of the camera - the angle of the target
            cam_to_tag_rotation = Rotation2d(
                prevEstPoseSingleTag.rotation().radians()
                + self.robotToCam.rotation().Z()
                - target_x_angle
            )
            # Calculate the translation of the camera to the tag. We take the position of the tag, transform by the distance to the tag, in the direction of the camera
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

            # Calculate the pose of the robot. We take the position of the camera to the tag and transform it by the robot to camera transform
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

            self.poseSingleTag.append(robot_pose)
            self.singleTagIDs.append(tgtID)



    def getObsTime(self):
        return self.obsTime

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

class WrapperedPhotonCameraFuel:
    def __init__(self, camName, robotToCam):
        # setVersionCheckEnabled(False)

        self.cam = PhotonCamera(camName)
        # TODO is this really the name of the camera or is this just as a reminder? Camera1,2,3,or 4??
        self.cameraDistortVector = const.CAM_DICT[camName][0]
        self.cameraIntrinsMatrix = const.CAM_DICT[camName][1]

        self.camName = camName
        self.poseEstimates = []
        self.robotToCam: Transform3d = robotToCam
    
    def update(self, curPose : Pose2d):
        res = self.cam.getLatestResult()
        for target in res.getTargets():
            pass