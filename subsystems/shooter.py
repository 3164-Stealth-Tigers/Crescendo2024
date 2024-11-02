import math

import commands2
import rev
import wpilib
import wpimath.controller
from wpimath.geometry import Rotation2d
from wpiutil import SendableBuilder

from config.constants import ShooterConstants


class Shooter(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self._conveyor_motor_left = rev.CANSparkMax(
            ShooterConstants.CONVEYOR_LEFT_MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless
        )
        self._conveyor_encoder = self._conveyor_motor_left.getEncoder()
        self._conveyor_controller = self._conveyor_motor_left.getPIDController()

        self._conveyor_motor_left.restoreFactoryDefaults()
        self._conveyor_motor_left.setSmartCurrentLimit(80)
        self._conveyor_motor_left.setInverted(ShooterConstants.INVERT_LEFT_CONVEYOR)
        self._conveyor_motor_left.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)

        # Convert from rpm to metres/sec with a gear ratio
        self._conveyor_encoder.setVelocityConversionFactor(
            ShooterConstants.FLYWHEEL_CIRCUMFERENCE / 60 / ShooterConstants.FLYWHEEL_GEAR_RATIO
        )

        self._conveyor_controller.setFeedbackDevice(self._conveyor_encoder)
        self._conveyor_controller.setP(ShooterConstants.FLYWHEEL_kP)
        self._conveyor_controller.setD(ShooterConstants.FLYWHEEL_kD)

        self._conveyor_motor_right = rev.CANSparkMax(
            ShooterConstants.CONVEYOR_RIGHT_MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless
        )
        self._conveyor_motor_right.restoreFactoryDefaults()
        self._conveyor_motor_right.setSmartCurrentLimit(80)
        self._conveyor_motor_right.setInverted(ShooterConstants.INVERT_RIGHT_CONVEYOR)
        self._conveyor_motor_right.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)

        # Define flywheel motors
        self._flywheel_motor_left = rev.CANSparkMax(
            ShooterConstants.FLYWHEEL_LEFT_MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless
        )
        self._flywheel_encoder = self._flywheel_motor_left.getEncoder()
        self._flywheel_controller = self._flywheel_motor_left.getPIDController()

        self._flywheel_encoder.setVelocityConversionFactor(math.pi * 3)

        self._flywheel_motor_left.restoreFactoryDefaults()
        self._flywheel_motor_left.setSmartCurrentLimit(80)
        self._flywheel_motor_left.setInverted(ShooterConstants.INVERT_LEFT_FLYWHEEL)
        self._flywheel_motor_left.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)

        self._flywheel_motor_right = rev.CANSparkMax(
            ShooterConstants.FLYWHEEL_RIGHT_MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless
        )

        self._flywheel_motor_right.restoreFactoryDefaults()
        self._flywheel_motor_right.setSmartCurrentLimit(80)
        self._flywheel_motor_right.setInverted(ShooterConstants.INVERT_RIGHT_FLYWHEEL)
        self._flywheel_motor_right.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)

        self._flywheel_feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            ShooterConstants.FLYWHEEL_kS, ShooterConstants.FLYWHEEL_kV
        )

        self._pivot_motor = rev.CANSparkMax(ShooterConstants.PIVOT_MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless)
        self._pivot_encoder = self._pivot_motor.getAbsoluteEncoder(rev.SparkAbsoluteEncoder.Type.kDutyCycle)
        self._pivot_controller = self._pivot_motor.getPIDController()

        self._pivot_motor.restoreFactoryDefaults()
        self._pivot_motor.setInverted(ShooterConstants.INVERT_PIVOT_MOTOR)
        self._pivot_controller.setFeedbackDevice(self._pivot_encoder)
        self._pivot_controller.setP(ShooterConstants.PIVOT_kP)
        self._pivot_controller.setD(ShooterConstants.PIVOT_kD)

        self._pivot_motor.setSmartCurrentLimit(60)
        self._pivot_motor.setSecondaryCurrentLimit(70)

        # Position conversion factor and inversion MUST be set before zero offset
        self._pivot_encoder.setPositionConversionFactor(360)
        self._pivot_encoder.setInverted(ShooterConstants.INVERT_PIVOT_ENCODER)
        self._pivot_encoder.setZeroOffset(ShooterConstants.PIVOT_OFFSET)

        self._pivot_controller.setPositionPIDWrappingEnabled(True)
        self._pivot_controller.setPositionPIDWrappingMaxInput(360)
        self._pivot_controller.setPositionPIDWrappingMinInput(0)

        # self._pivot_motor.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, False)
        # self._pivot_motor.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, ShooterConstants.PIVOT_FORWARD_LIMIT)

        # self._pivot_motor.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, False)
        # self._pivot_motor.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, ShooterConstants.PIVOT_REVERSE_LIMIT)

    def run_flywheel_power(self, output_power: float):
        self._flywheel_motor_right.set(output_power)
        self._flywheel_motor_left.set(output_power)

    def run_conveyor_power(self, output_power: float):
        self._conveyor_motor_right.set(output_power)
        self._conveyor_motor_left.set(output_power)

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

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.addDoubleProperty("Shooter Angle (deg)", lambda: self.angle.degrees(), lambda _: None)
        builder.addDoubleProperty("Flywheel Velocity", lambda: self.flywheel_speed, lambda _: None)
        builder.addDoubleProperty(
            "Flywheel Percent Output", self._conveyor_motor_left.getAppliedOutput, self.run_flywheel_power
        )

    def shooter_angle_command(self, angle: Rotation2d):
        return commands2.RunCommand(lambda: self.set_angle(angle), self).finallyDo(
            lambda _: self.run_pivot_power(0)
        )
