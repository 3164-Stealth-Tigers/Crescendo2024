import wpilib
from wpimath.geometry import Pose2d

from config.constants import Field


def distance_to_speaker(robot_pose: Pose2d) -> float:
    # Locate the correct SPEAKER depending on which alliance we're on
    if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue:
        speaker_pos = Field.BLUE_SPEAKER_POSITION
    else:
        speaker_pos = Field.RED_SPEAKER_POSITION

    return robot_pose.translation().distance(speaker_pos)