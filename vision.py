import time
from typing import Optional

import ntcore
from photonlibpy.photonCamera import PhotonCamera
from wpimath.geometry import Pose3d, Transform3d

from config.constants import Coprocessor, Physical

camera = PhotonCamera(Coprocessor.CAMERA_NAME)


def get_estimated_global_pose() -> Optional[Pose3d]:
    photon_result = camera.getLatestResult().multiTagResult.estimatedPose

    # Reject results that are too ambiguous or those for which no tags are found
    if photon_result.ambiguity > 0.2 or not photon_result.isPresent:
        return

    camera_pose = Pose3d(photon_result.best.translation(), photon_result.best.rotation())
    robot_pose = camera_pose.transformBy(Physical.ROBOT_TO_CAMERA_TRANSFORMATION.inverse())

    return robot_pose


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

        # Multi-tag pose estimation
        pose = get_estimated_global_pose()
        # Check if the pose is valid
        if pose:
            print(f"X: {pose.x}, Y: {pose.y}, Z: {pose.z}")

        # AprilTag-relative transforms
        targets = get_tags()
        for i, transform in targets.items():
            print(f"ID: {i}, X: {transform.x}, Y: {transform.y}, Z: {transform.z}")
