import commands2
from wpimath.geometry import Rotation2d

from subsystems import Shooter
from swervepy import SwerveDrive


class AlignShooterToSpeakerCommand(commands2.Command):

    def __init__(self, swerve: SwerveDrive, shooter: Shooter):
        super().__init__()

        self.swerve = swerve
        self.shooter = shooter

    @property
    def desired_shooter_angle(self) -> Rotation2d:
        pass

    def execute(self):
        self.shooter.set_angle(self.desired_shooter_angle)
