import commands2


class Intake(commands2.Subsystem):

    def run_intake(self):
        """Begin running the intake"""

    def stop_intake(self):
        """Stop in-taking"""

    @property
    def has_possession(self) -> bool:
        """Does the intake have secure possession of a NOTE?"""

    def run_exchange(self):
        """Run the motors to exchange the NOTE between intake and shooter"""

    def stop_exchange(self):
        """Stop the exchange motors"""
