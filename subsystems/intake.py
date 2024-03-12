import commands2
import rev
import wpimath.controller

from config.constants import IntakeConstants


class Intake(commands2.Subsystem):

    def __init__(self):
        super().__init__()

        self._motor = rev.CANSparkMax(IntakeConstants.MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless)
        self._encoder = self._motor.getEncoder()
        self._controller = self._motor.getPIDController()

        # Feedforward in rotations (distance unit)
        self._feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(IntakeConstants.INTAKE_kS,
                                                                            IntakeConstants.INTAKE_kV)

        # Conversion factor for gear ratio
        self._encoder.setVelocityConversionFactor(IntakeConstants.GEAR_RATIO)

        self._controller.setP(IntakeConstants.INTAKE_kP)

    def run_intake(self):
        ff = self._feedforward.calculate(IntakeConstants.INTAKE_VELOCITY)
        self._controller.setReference(IntakeConstants.INTAKE_VELOCITY, rev.CANSparkMax.ControlType.kVelocity,
                                      arbFeedforward=ff)

    def run_intake_power(self, power: float):
        self._motor.set(power)

    def stop_intake(self):
        self._motor.set(0)

    @property
    def has_possession(self) -> bool:
        """Does the intake have secure possession of a NOTE?"""
        raise NotImplementedError

    def run_intake_command(self) -> commands2.Command:
        return commands2.RunCommand(self.run_intake, self).finallyDo(lambda _: self.stop_intake())
