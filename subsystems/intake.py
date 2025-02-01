import commands2
import rev
import wpimath.controller
from wpiutil import SendableBuilder

from config.constants import IntakeConstants


class Intake(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self._motor = rev.SparkMax(IntakeConstants.MOTOR_ID, rev.SparkBase.MotorType.kBrushless)
        self._encoder = self._motor.getEncoder()
        self._controller = self._motor.getClosedLoopController()

        # Feedforward in rotations (distance unit)
        self._feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            IntakeConstants.INTAKE_kS, IntakeConstants.INTAKE_kV
        )

        settings = rev.SparkMaxConfig()
        # Conversion factor for gear ratio
        settings.encoder.velocityConversionFactor(IntakeConstants.GEAR_RATIO)
        settings.closedLoop.P(IntakeConstants.INTAKE_kP)
        self._motor.configure(settings, rev.SparkBase.ResetMode.kNoResetSafeParameters, rev.SparkBase.PersistMode.kNoPersistParameters)

    def run_intake(self):
        ff = self._feedforward.calculate(IntakeConstants.INTAKE_VELOCITY)
        self._controller.setReference(
            IntakeConstants.INTAKE_VELOCITY, rev.BaseConfig.ControlType.kVelocity, arbFeedforward=ff
        )

    def run_intake_power(self, power: float):
        self._motor.set(power)

    def stop_intake(self):
        self._motor.set(0)

    @property
    def velocity(self) -> float:
        """The first intake roller's velocity in RPM"""
        return self._encoder.getVelocity()

    @property
    def has_possession(self) -> bool:
        """Does the intake have secure possession of a NOTE?"""
        raise NotImplementedError

    def run_intake_command(self) -> commands2.Command:
        return commands2.RunCommand(self.run_intake, self).finallyDo(lambda _: self.stop_intake())

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.addDoubleProperty("Roller Velocity (RPM)", lambda: self.velocity, lambda _: None)
        builder.addDoubleProperty("Motor Percent Output", self._motor.getAppliedOutput, self.run_intake_power)
