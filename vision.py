import time
from typing import Optional

import ntcore
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
import robotpy_apriltag as apriltag
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from wpimath.geometry import Transform3d, Pose2d, Transform2d

from config.constants import CoprocessorConstants

tag_camera = PhotonCamera(CoprocessorConstants.APRIL_TAG_CAMERA_NAME)
estimator = PhotonPoseEstimator(
    apriltag.loadAprilTagLayoutField(apriltag.AprilTagField.k2024Crescendo),
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    tag_camera,
    CoprocessorConstants.ROBOT_TO_CAMERA_TRANSFORMATION,
)
estimator.multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY

note_camera = PhotonCamera(CoprocessorConstants.NOTE_CAMERA_NAME)


def get_estimated_global_pose() -> Optional[EstimatedRobotPose]:
    return estimator.update()


def get_estimated_global_pose_2d() -> Optional[Pose2d]:
    result = estimator.update()
    if result:
        return result.estimatedPose.toPose2d()


def get_tags() -> dict[int, Transform3d]:
    photon_result = tag_camera.getLatestResult().getTargets()

    tags = {}
    for target in photon_result:
        # Skip target if its pose is too ambiguous
        if target.poseAmbiguity > 0.2:
            continue

        tags[target.fiducialId] = target.bestCameraToTarget

    return tags


def get_closest_note() -> Optional[PhotonTrackedTarget]:
    targets = note_camera.getLatestResult().getTargets()

    if not targets:
        return

    # The target with the highest area will be the NOTE closest to us
    closest_target = targets[0]
    for target in targets:
        if target.area > closest_target.area:
            closest_target = target

    return closest_target


if __name__ == "__main__":
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startServer()
    print("NT server started!")

    while True:
        time.sleep(0.02)

        note = get_closest_note()
        if note:
            transform = note.bestCameraToTarget
            print(f"dYaw: {note.yaw}")
            print(f"(feet) x: {transform.x_feet}, y: {transform.y_feet}, z: {transform.z_feet}")

        """
        # Robot pose estimation
        pose = get_estimated_global_pose()
        # Check if the pose is valid
        if pose:
            pose = pose.estimatedPose
            print(f"X: {pose.x}, Y: {pose.y}, Z: {pose.z}")

        # AprilTag-relative transforms
        targets = get_tags()
        for i, transform in targets.items():
            print(f"ID: {i}, X: {transform.x}, Y: {transform.y}, Z: {transform.z}")
        """
