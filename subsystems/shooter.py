import commands2
from wpimath.geometry import Rotation2d


class Shooter(commands2.Subsystem):

    def run_shooter(self, output_speed: float):
        """Run the shooter at a specified speed

        :param output_speed: The flywheel speed in m/s
        """

    def set_angle(self, angle: Rotation2d):
        """Move the shooter to a specified angle

        :param angle: The angle above horizontal
        """

    @property
    def flywheel_speed(self) -> float:
        """The speed of the flywheels in m/s"""

    @property
    def angle(self) -> Rotation2d:
        """The angle above horizontal of the shooter"""

    @property
    def ready_to_shoot(self) -> bool:
        """Does the shooter have possession of a NOTE, ready to shoot it?"""
