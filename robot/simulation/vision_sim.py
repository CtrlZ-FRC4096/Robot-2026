"""PhotonVision simulation of the AprilTag cameras.

Each WrapperedPhotonCameraTag in the pose estimator gets a PhotonCameraSim
at its real robot-to-camera transform. Every loop the cameras "see" the 2026
tags from the true (physics) robot pose, with pixel noise and latency, and
publish results to NetworkTables exactly where a coprocessor would; the robot
code reads them through its normal PhotonCamera path and fuses them into the
pose estimate.
"""

import numpy as np
from photonlibpy.simulation import PhotonCameraSim, SimCameraProperties, VisionSystemSim
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpilib import SmartDashboard

import const

# OV9281 at 1280x800 with the team's camera_1 calibration.
RESOLUTION = (1280, 800)
FPS = 30
# photonlibpy annotates these as milliseconds but uses them as seconds
# (its own presets pass 30e-3), so they're in seconds here.
AVG_LATENCY = 0.035
LATENCY_STD_DEV = 0.005
CALIB_ERROR_PX = (0.35, 0.10)  # average, standard deviation
MAX_SIGHT_RANGE = 6.0          # m (the robot code ignores tags past 5 m)


class VisionSim:
    def __init__(self, cameras):
        layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)
        self.system = VisionSystemSim("main")
        self.system.addAprilTags(layout)
        distortion, intrinsics = const.CAM_DICT["camera_1"]
        for camera in cameras:
            props = SimCameraProperties()
            props.setCalibrationFromIntrinsics(*RESOLUTION, np.array(intrinsics), np.array(distortion))
            props.setCalibError(*CALIB_ERROR_PX)
            props.setFPS(FPS)
            props.setAvgLatency(AVG_LATENCY)
            props.setLatencyStdDev(LATENCY_STD_DEV)
            camera_sim = PhotonCameraSim(camera.cam, props, layout)
            camera_sim.setMaxSightRange(MAX_SIGHT_RANGE)
            self.system.addCamera(camera_sim, camera.robotToCam)
        SmartDashboard.putData("Vision Sim Field", self.system.getDebugField())

    def update(self, true_pose):
        self.system.update(true_pose)
