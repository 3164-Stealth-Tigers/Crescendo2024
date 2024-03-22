from typing import Optional

import wpilib
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose2d, Rotation2d

ll = NetworkTableInstance.getDefault().getTable("limelight")
botpose = ll.getDoubleArrayTopic("botpose_wpiblue").subscribe([])


def get_estimated_pose() -> Optional[Pose2d]:
    pose = botpose.get()

    # NT error
    if not pose:
        return

    # Invalid LL data
    if pose[0] == 0.0:
        return

    print(f"Target found: X: {pose[0]}  Y: {pose[1]}  Yaw: {pose[5]}deg")

    return Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5]))