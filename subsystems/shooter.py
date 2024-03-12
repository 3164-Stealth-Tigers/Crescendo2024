import commands2
import rev
import wpimath.controller
from wpimath.geometry import Rotation2d

from config.constants import ShooterConstants


class Shooter(commands2.Subsystem):

    def __init__(self):
        super().__init__()

        self._flywheel_motor = rev.CANSparkMax(ShooterConstants.FLYWHEEL_LEFT_MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless)
        self._flywheel_encoder = self._flywheel_motor.getEncoder()
        self._flywheel_controller = self._flywheel_motor.getPIDController()

        self._flywheel_motor.restoreFactoryDefaults()
        self._flywheel_motor.setSmartCurrentLimit(40)
        self._flywheel_motor.setSecondaryCurrentLimit(60)

        # Convert from rpm to metres/sec with a gear ratio
        self._flywheel_encoder.setVelocityConversionFactor(ShooterConstants.LARGE_WHEEL_CIRCUMFERENCE / 60 / ShooterConstants.FLYWHEEL_GEAR_RATIO)

        self._flywheel_controller.setFeedbackDevice(self._flywheel_encoder)
        self._flywheel_controller.setP(ShooterConstants.FLYWHEEL_kP)
        self._flywheel_controller.setD(ShooterConstants.FLYWHEEL_kD)

        self._flywheel_feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(ShooterConstants.FLYWHEEL_kS,
                                                                                     ShooterConstants.FLYWHEEL_kV)

        self._flywheel_slave = rev.CANSparkMax(ShooterConstants.FLYWHEEL_RIGHT_MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless)
        self._flywheel_slave.restoreFactoryDefaults()
        self._flywheel_slave.setSmartCurrentLimit(40)
        self._flywheel_slave.setSecondaryCurrentLimit(60)
        self._flywheel_slave.setInverted(True)
        # flywheel_slave.follow(self._flywheel_motor, invert=True)

        self._pivot_motor = rev.CANSparkMax(ShooterConstants.PIVOT_MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless)
        self._pivot_encoder = self._pivot_motor.getAbsoluteEncoder(rev.SparkAbsoluteEncoder.Type.kDutyCycle)
        self._pivot_controller = self._pivot_motor.getPIDController()

        self._pivot_controller.setFeedbackDevice(self._pivot_encoder)
        self._pivot_controller.setP(ShooterConstants.PIVOT_kP)
        self._pivot_controller.setD(ShooterConstants.PIVOT_kD)

    def run_flywheel_velocity(self, output_speed: float):
        """Run the flywheel at a specified speed

        :param output_speed: The large flywheel speed in m/s
        """
        ff = self._flywheel_feedforward.calculate(output_speed)
        self._flywheel_controller.setReference(output_speed, rev.CANSparkMax.ControlType.kVelocity, arbFeedforward=ff)

    def run_flywheel_power(self, output_power: float):
        self._flywheel_motor.set(output_power)
        self._flywheel_slave.set(output_power)

    def stop_flywheel(self):
        """Stop the flywheel"""
        self._flywheel_motor.set(0)

    def set_angle(self, angle: Rotation2d):
        """Move the shooter to a specified angle

        :param angle: The angle above horizontal
        """
        self._pivot_controller.setReference(angle.degrees(), rev.CANSparkMax.ControlType.kPosition)

    def run_pivot_power(self, power: float):
        self._pivot_motor.set(power)

    @property
    def flywheel_speed(self) -> float:
        """The speed of the flywheels in m/s"""
        return self._flywheel_encoder.getVelocity()

    @property
    def angle(self) -> Rotation2d:
        """The angle above horizontal of the shooter"""
        return Rotation2d.fromDegrees(self._pivot_encoder.getPosition())

    @property
    def ready_to_shoot(self) -> bool:
        """Does the shooter have possession of a NOTE, ready to shoot it?"""
        raise NotImplementedError
