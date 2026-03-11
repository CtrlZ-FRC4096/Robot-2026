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
        # self.cameraDistortVector = const.CAM_DICT[camName][0]
        # self.cameraIntrinsMatrix = const.CAM_DICT[camName][1]

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
        # self.counter += 1
        self.poseEstimates = []
        self.zEstimates = []
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
            zEst = tagFieldPose.Z() - self.robotToCam.Z() - math.sin(self.robotToCam.rotation().Y() +  target_y_angle) * distance_3d
            self.zEstimates.append(zEst)
            self.poseSingleTag.append([robot_pose, tgtID, target.getPoseAmbiguity(), zEst])
            self.singleTagIDs.append(tgtID)
            



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

class WrapperedPhotonCameraIntakeFuel:
    def __init__(self, camName, robotToCam, intakeCam=True):
        # setVersionCheckEnabled(False)

        self.cam = PhotonCamera(camName)
        # TODO is this really the name of the camera or is this just as a reminder? Camera1,2,3,or 4??
        self.camName = camName
        self.fuel_seen_field_relative : list[Translation3d] = []
        self.new_observations : list[Translation2d] = []
        self.robotToCam: Transform3d = robotToCam

        self.memory_duration = 0.25
        self.merge_radius = 0.2

        self.fuel_memory : list[tuple[Translation2d, float]]= []
    
    def update(self, curPose : Pose2d, zCoord : float, gyro : Rotation3d):
        res = self.cam.getLatestResult()
        self.fuel_seen_field_relative = []
        self.new_observations = []
        curTrans = curPose.translation()
        cur_time = res.getTimestampSeconds()


        robot_to_field = Transform3d(
            Translation3d(curPose.X(), curPose.Y(), zCoord),
            gyro
        )
        for target in res.getTargets():
            tgt_x_angle = target.getYaw()
            tgt_y_angle = target.getPitch()

            dist_3d = target.getBestCameraToTarget().translation().norm()

            x_cam = dist_3d * math.cos(tgt_y_angle) * math.cos(tgt_x_angle)
            y_cam = dist_3d * math.cos(tgt_y_angle) * math.sin(tgt_x_angle)
            z_cam = dist_3d * math.sin(tgt_y_angle)
            
            fuel_in_cam = Translation3d(x_cam, y_cam, z_cam)
            fuel_in_robot = fuel_in_cam.rotateBy(self.robotToCam.inverse().rotation()) + self.robotToCam.inverse().translation()
            
            fuel_in_field = fuel_in_robot.rotateBy(robot_to_field.rotation()) + robot_to_field.translation()
            
            self.fuel_seen_field_relative.append(fuel_in_field)
            
            if fuel_in_field.Z() <= 0.15:
                self.new_observations.append(fuel_in_field.toTranslation2d())
         
        for fuel in self.new_observations:
            matched = False # flag if it has already been seen
            for i, (mem_pos, mem_time) in enumerate(self.fuel_memory):
                if fuel.distance(mem_pos) < self.merge_radius:
                    self.fuel_memory[i] = (fuel, cur_time)
                    matched = True
                    break
                
            if not matched:
                self.fuel_memory.append((fuel, cur_time))

        self.fuel_memory = [
            (pos, timestamp) for (pos, timestamp) in self.fuel_memory
            if (cur_time - timestamp < self.memory_duration)
        ]

    def getBestPtIntake(self, curPose : Pose2d):
        if not self.fuel_memory:
            return None
        
        pts = np.array([[pos.X(), pos.Y()] for pos, time in self.fuel_memory])
        trans = curPose.translation()

        diffs = pts[:, np.newaxis, :] - pts[np.newaxis, :, :]
        distance_matrix = np.sum(diffs**2, axis=-1)

        nearby_counts = np.sum(distance_matrix < 0.5625, axis=1)

        dists_to_robot = np.linalg.norm(pts - trans, axis=1)

        scores = (nearby_counts ** 2) / (dists_to_robot + 1.0)

        best_idx = np.argmax(scores)

        return self.fuel_memory[best_idx][0], scores[best_idx]
    
    def calculate_single_score(self, fuel : Translation2d, curPose: Pose2d):
        nearby_count = 0
        for other_pos, timestamp in self.fuel_memory:
            if fuel.distance(other_pos) < 0.75:
                nearby_count += 1
        
        dist_to_robot = curPose.translation().distance(fuel)

        return (nearby_count ** 2) / (dist_to_robot + 1.0)

    def getAllFuelSeen(self) -> list[Translation3d]:
        return self.fuel_seen_field_relative
    
    def getFuelFlat(self) -> list[Translation2d]:
        return self.new_observations
    
    def getFuelMemory(self):
        return self.fuel_memory

class WrapperedPhotonCameraBin:
    def __init__(self, camName, robotToCam):
        self.cam = PhotonCamera(camName)
        self.camName = camName
        self.robotToCam = robotToCam

        self.count = 0
        self.highest = 0

        self.hopper_fill = 10 # fuel to fill the bottom of the hopper
        self.offset = 0
        self.layers = 0
        self.layers_old = 0

        self.tsw = False # timer switch
        self.tsw_old = False
        self.mem = []
        self.timer = wpilib.Timer()
        self.timer.start()
    
    def update(self):
        res = self.cam.getLatestResult()
        fuel = res.getTargets()

        self.mem.append(len(fuel))
        if len(self.mem) > 1000:
            del self.mem[0]
        if (self.timer.get())%0.05 == 0:
            self.tsw = not self.tsw
        if self.tsw != self.tsw_old:
            if self.tsw:
                count1 = len(fuel)
            else:
                count2 = len(fuel)
            if abs(count1-count2) <= 2:
                self.count = (min(self.mem)+max(self.mem))/2 # median
            else:
                self.count = self.get_mode(self.mem)
            self.tsw_old = self.tsw
        
        if self.count > self.highest:
            self.highest = self.count

        if self.count-self.offset > self.hopper_fill:
            self.layers += 1
            self.offset += self.hopper_fill

    def is_full(self):
        return self.layers >= 2
    
    def get_mode(x):
        log = []
        for item in x:
            if (not item in [f[0] for f in log]) or (len(log) == 0):
                log.append([item, 0])
            log[[f[0] for f in log].index(item)][1] += 1
        return sorted(log, key=lambda x:x[1], reverse=True)[0][0]