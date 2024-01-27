import time
from typing import Optional

import ntcore
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
import robotpy_apriltag as apriltag
from wpimath.geometry import Transform3d, Pose2d

from config.constants import Coprocessor, Physical

camera = PhotonCamera(Coprocessor.CAMERA_NAME)
estimator = PhotonPoseEstimator(
    apriltag.loadAprilTagLayoutField(apriltag.AprilTagField.k2024Crescendo),
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    camera,
    Physical.ROBOT_TO_CAMERA_TRANSFORMATION,
)
estimator.multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY


def get_estimated_global_pose() -> Optional[EstimatedRobotPose]:
    return estimator.update()


def get_estimated_global_pose_2d() -> Optional[Pose2d]:
    result = estimator.update()
    if result:
        return result.estimatedPose.toPose2d()


def get_tags() -> dict[int, Transform3d]:
    photon_result = camera.getLatestResult().getTargets()

    tags = {}
    for target in photon_result:
        # Skip target if its pose is too ambiguous
        if target.poseAmbiguity > 0.2:
            continue

        tags[target.fiducialId] = target.bestCameraToTarget

    return tags


if __name__ == "__main__":
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startServer()
    print("NT server started!")

    while True:
        time.sleep(0.02)

        # Robot pose estimation
        pose = get_estimated_global_pose()
        # Check if the pose is valid
        if pose:
            pose = pose.estimatedPose
            print(f"X: {pose.x}, Y: {pose.y}, Z: {pose.z}")

        # AprilTag-relative transforms
        """
        targets = get_tags()
        for i, transform in targets.items():
            print(f"ID: {i}, X: {transform.x}, Y: {transform.y}, Z: {transform.z}")
        """
