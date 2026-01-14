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
import cv2


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
class WrapperedPhotonCamera:
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
        self.tag_map = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)

        self.reef_tags_to_use = [6,7,8,9,10,11,17,18,19,20,21,22]

    @staticmethod
    def tgt_corner_to_list(target):
        return [target.x, target.y]

    def update(
        self,
        prevEstPose: Pose2d,
        prevEstPoseSingleTag: Pose2d,
        allianceColor: str,
        yaw: Rotation2d,
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

        ## MultiTag code
        photon_pose_estimator = photonPoseEstimator.PhotonPoseEstimator(
            self.tag_map,
            photonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.cam,
            self.robotToCam,
        )
        vision_est = photon_pose_estimator.update(res)

        if vision_est is not None:

            robot_pose = vision_est.estimatedPose.toPose2d()

            # if ((robot_pose.x > -0.5) and (robot_pose.x < const.FIELD_LENGTH_METERS + 0.5) and (robot_pose.y > -0.5) and (robot_pose.y < const.FIELD_WIDTH_METERS + 0.5)): # Check if the robot is on the field
            self.poseEstimates.append(CameraPoseObservation(self.obsTime, robot_pose))
            for target in res.getTargets():
                tgtID = target.getFiducialId()

                tagFieldPose = self.tag_map.getTagPose(tgtID)
                self.tagAmbiguity.append(target.getPoseAmbiguity())
                self.tagPositions.append(tagFieldPose)
                self.tagDistances.append(target.getBestCameraToTarget().translation().norm())

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
            if tgtID in self.reef_tags_to_use:  # Only use reef IDs, everything else is not great

                tagFieldPose = self.tag_map.getTagPose(tgtID)

                # corners = np.ndarray(
                #     [[[corner.x, corner.y] for corner in target.getDetectedCorners()]]
                # )
                corners = np.array(
                    [
                        WrapperedPhotonCamera.tgt_corner_to_list(corner)
                        for corner in target.getDetectedCorners()
                    ]
                ).astype(np.float32)

                # SmartDashboard.putNumber(f"corners for tag x: {self.camName}", corners[0][0])
                # SmartDashboard.putNumber(f"corners for tag y: {self.camName}", corners[0][1])

                # Return list of n corners, for fiducials this is counter clockwise starting from the top left corner of the tag.
                corners_undistorted = cv2.undistortPoints(  # Unsure if these corners have already been undistorted
                    corners,
                    # self.cam.getCameraMatrix(),
                    # self.cam.getDistortionCoefficients(),
                    self.cameraIntrinsMatrix,
                    self.cameraDistortVector,
                    None,
                    self.cameraIntrinsMatrix,
                )  # Return list of n corners, for fiducials this is counter clockwise starting from the top left corner of the tag.
                corners = np.zeros((4, 2))
                for index, corner in enumerate(
                    corners_undistorted
                ):  # calculate the angle of each corner relative to the camera center in the x and y directions (radians)
                    vec = np.linalg.inv(self.cameraIntrinsMatrix).dot(
                        np.array([corner[0][0], corner[0][1], 1]).T
                    )
                    corners[index][0] = math.atan(vec[0])
                    corners[index][1] = math.atan(vec[1])

                # Calculate the center of the target in x and y angles (radians)
                target_x_angle = np.mean(corners[:, 0])
                target_y_angle = np.mean(corners[:, 1])

                # SmartDashboard.putNumber(f"tgt x {self.camName}", target_x_angle)
                # SmartDashboard.putNumber(f"tgt y {self.camName}", target_y_angle)

                # print("targ x: ", target_x_angle)
                # print("target y: ", target_y_angle)

                # z_dist = tag_map.getTagPose(tgtID).Z() - inchesToMeters(4.87)

                distance_3d = target.getBestCameraToTarget().translation().norm()
                # SmartDashboard.putNumber(f"3d distance for {self.camName}", distance_3d)

                distance_2d_to_tag = distance_3d * math.cos(
                    (-1 * self.robotToCam.rotation().Y()) - target_y_angle
                )  # cosine is even so we don't need to negate both
                # SmartDashboard.putNumber(f"2d distance for {self.camName}", distance_2d_to_tag)

                # print(distance_2d_to_tag)

                # Calculate the rotation of the camera to the tag. The rotation of the robot + rotation of the camera - the angle of the target
                cam_to_tag_rotation = Rotation2d(
                    prevEstPoseSingleTag.rotation().radians()
                    + self.robotToCam.rotation().Z()
                    - target_x_angle
                )
                # SmartDashboard.putNumber(f"cam to tag rotation for {self.camName}", cam_to_tag_rotation.degrees())

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
                # SmartDashboard.putNumber(f"field to camera translation x for {self.camName}", field_to_camera_translation.X())
                # SmartDashboard.putNumber(f"field to camera translation y for {self.camName}", field_to_camera_translation.Y())

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
                #     Transform2d(-self.robotToCam.X(), self.robotToCam.Y(), Rotation2d()) ##Throwing a negative on the cam y seemed to work, I hate this
                # )
                # Use previous angle (gyro) at the time for robot rotation
                robot_pose = Pose2d(
                    robot_pose.translation(), prevEstPoseSingleTag.rotation()
                )
                # SmartDashboard.putNumber(f"robot pose for {self.camName} x", robot_pose.X())
                # SmartDashboard.putNumber(f"robot pose for {self.camName} y", robot_pose.Y())
                # SmartDashboard.putNumber(f"robot pose for {self.camName} theta", robot_pose.rotation().degrees())
                # print(robot_pose)
                # z_dist / (math.tan(self.robotToCam.rotation().Y() + target_y_angle))
                # absolute_angle = yaw.radians() + target_x_angle

                # x_dist = distance_2d * math.cos(absolute_angle)
                # y_dist = distance_2d * math.sin(absolute_angle)

                # distance = (
                #     target.getBestCameraToTarget().translation().norm()
                # )  # distance from camera to target in meters
                # print("distance: ", distance)
                # # Calculate the position of the target to the camera  in the camera coordinate system (meters)
                # # Use spherical coordinates to calculate the x, y, and z distances
                # z_dist = -1 * distance * math.cos((math.pi / 2) - target_y_angle)
                # y_dist = -1 * (
                #     distance
                #     * math.sin(target_x_angle)
                #     * math.sin((math.pi / 2) - target_y_angle)
                # )
                # x_dist = (
                #     distance
                #     * math.cos(target_x_angle)
                #     * math.sin((math.pi / 2) - target_y_angle)
                # )
                # print("x dist: ", x_dist)
                # print("y dist: ", y_dist)
                # print("z dist: ", z_dist)

                # camToTarget = Transform3d(
                #     Translation3d(x_dist, y_dist, z_dist), Rotation3d()
                # )  # Create a Pose3d object with the calculated x, y, and z distances, and no rotation

                # # Calculate the position of the robot on the field in the field coordinate system (meters) from the tag pose and the camera to target transform
                # fieldPose = self._toFieldPose(tagFieldPose, camToTarget)
                # # print(fieldPose)

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
