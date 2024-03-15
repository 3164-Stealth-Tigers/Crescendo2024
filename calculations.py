import math
from collections import UserDict

import wpilib
from wpimath.geometry import Pose2d, Rotation2d

from config.constants import FieldConstants


def distance_to_speaker(robot_pose: Pose2d) -> float:
    # Locate the correct SPEAKER depending on which alliance we're on
    if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue:
        speaker_pos = FieldConstants.BLUE_SPEAKER_POSITION
    else:
        speaker_pos = FieldConstants.RED_SPEAKER_POSITION

    return robot_pose.translation().distance(speaker_pos)


def angle_to_speaker(robot_pose: Pose2d) -> Rotation2d:
    pass


def floor_key(d, key):
    return max(k for k in d if k < key)


def ceil_key(d, key):
    return min(k for k in d if k > key)


class InterpolatingDict(UserDict):
    def __getitem__(self, key):
        try:
            return self.data[key]
        except KeyError:
            pass

        x1 = floor_key(self.data, key)
        x2 = ceil_key(self.data, key)

        y1 = self.data[x1]
        y2 = self.data[x2]

        return y1 + (((key - x1) * (y2 - y1)) / (x2 - x1))
