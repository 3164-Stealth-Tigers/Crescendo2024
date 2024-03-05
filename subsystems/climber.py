import enum
from typing import Optional

import commands2


class Direction(enum.Enum):
    """A vertical direction"""

    UP: enum.auto()
    DOWN: enum.auto()


class Climber(commands2.Subsystem):

    @property
    def extension_height(self) -> float:
        """The height (in metres) that the climber has extended from dead bottom"""

    def move(self, direction: Direction):
        """Move the climber at a set speed either up or down

        :param direction: The direction to move the climber. UP designates away from the floor.
        """

    @property
    def at_limit(self) -> Optional[Direction]:
        """The limit the climber has hit, if any.

        UP designates that the climber is fully extended.
        DOWN designates that the climber is at dead bottom.
        None designates that the climber hasn't hit a limit.
        """
