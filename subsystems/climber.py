import enum
from typing import Optional

import commands2
import rev
import wpimath.controller

from config.constants import ClimberConstants


class Direction(enum.Enum):
    """A vertical direction"""

    UP: enum.auto()
    DOWN: enum.auto()


class Climber(commands2.Subsystem):

    def __init__(self):
        super().__init__()
        
        self._motor = rev.CANSparkMax(ClimberConstants.MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless)
        self._encoder = self._motor.getEncoder()
        self._controller = self._motor.getPIDController()

        # Setup limits
        # Hard limit switch on the bottom
        self._bottom_limit = self._motor.getForwardLimitSwitch(ClimberConstants.BOTTOM_LIMIT_SWITCH_TYPE)
        self._bottom_limit.enableLimitSwitch(True)

        # Convert from rotations to metres with a gear ratio
        self._encoder.setPositionConversionFactor(ClimberConstants.SPOOL_CIRCUMFERENCE / ClimberConstants.GEAR_RATIO)
        # Convert from rpm to metres/sec
        self._encoder.setVelocityConversionFactor(ClimberConstants.SPOOL_CIRCUMFERENCE / ClimberConstants.GEAR_RATIO / 60)

        self._feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(ClimberConstants.CLIMBER_kS, ClimberConstants.CLIMBER_kV)

    @property
    def extension_height(self) -> float:
        """The height (in metres) that the climber has extended from dead bottom"""
        raise NotImplementedError

    def move(self, direction: Direction):
        """Move the climber at a set speed either up or down

        :param direction: The direction to move the climber. UP designates away from the floor.
        """
        velocity = ClimberConstants.CLIMBER_SPEED * (1 if direction is Direction.UP else -1)
        ff = self._feedforward.calculate(velocity)
        self._controller.setReference(velocity, rev.CANSparkMax.ControlType.kVelocity, arbFeedforward=ff)

    def run_climber_power(self, power: float):
        self._motor.set(power)

    @property
    def at_limit(self) -> Optional[Direction]:
        """The limit the climber has hit, if any.

        UP designates that the climber is fully extended.
        DOWN designates that the climber is at dead bottom.
        None designates that the climber hasn't hit a limit.
        """
        if self._bottom_limit.get():
            return Direction.UP
