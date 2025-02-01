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

        self._conveyor_motor_left = rev.SparkMax(
            ShooterConstants.CONVEYOR_LEFT_MOTOR_ID, rev.SparkBase.MotorType.kBrushless
        )
        self._conveyor_encoder = self._conveyor_motor_left.getEncoder()
        #self._conveyor_controller = self._conveyor_motor_left.getClosedLoopController()

        conveyor_settings = rev.SparkMaxConfig()

        conveyor_settings.smartCurrentLimit(60)
        conveyor_settings.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)

        # Convert from rpm to metres/sec with a gear ratio
        conveyor_settings.encoder.velocityConversionFactor(
            ShooterConstants.FLYWHEEL_CIRCUMFERENCE / 60 / ShooterConstants.FLYWHEEL_GEAR_RATIO
        )

        #self._conveyor_controller.setFeedbackDevice(self._conveyor_encoder)
        #self._conveyor_controller.setP(ShooterConstants.FLYWHEEL_kP)
        #self._conveyor_controller.setD(ShooterConstants.FLYWHEEL_kD)

        # Inversion for left motor; will need to be changed for right motor
        conveyor_settings.inverted(ShooterConstants.INVERT_LEFT_CONVEYOR)

        self._conveyor_motor_left.configure(conveyor_settings, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        self._conveyor_motor_right = rev.SparkMax(
            ShooterConstants.CONVEYOR_RIGHT_MOTOR_ID, rev.SparkBase.MotorType.kBrushless
        )
        conveyor_settings.inverted(ShooterConstants.INVERT_RIGHT_CONVEYOR)
        self._conveyor_motor_right.configure(conveyor_settings, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        # Define flywheel motors
        self._flywheel_motor_left = rev.SparkMax(
            ShooterConstants.FLYWHEEL_LEFT_MOTOR_ID, rev.SparkBase.MotorType.kBrushless
        )
        self._flywheel_encoder = self._flywheel_motor_left.getEncoder()
        #self._flywheel_controller = self._flywheel_motor_left.getPIDController()

        flywheel_settings = rev.SparkMaxConfig()

        flywheel_settings.encoder.velocityConversionFactor(math.pi * 3)
        flywheel_settings.smartCurrentLimit(60, 100)
        flywheel_settings.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)

        flywheel_settings.inverted(ShooterConstants.INVERT_LEFT_FLYWHEEL)
        self._flywheel_motor_left.configure(flywheel_settings, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        self._flywheel_motor_right = rev.SparkMax(
            ShooterConstants.FLYWHEEL_RIGHT_MOTOR_ID, rev.SparkBase.MotorType.kBrushless
        )
        flywheel_settings.inverted(ShooterConstants.INVERT_RIGHT_FLYWHEEL)
        self._flywheel_motor_right.configure(flywheel_settings, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        self._flywheel_feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            ShooterConstants.FLYWHEEL_kS, ShooterConstants.FLYWHEEL_kV
        )

        self._pivot_motor = rev.SparkMax(ShooterConstants.PIVOT_MOTOR_ID, rev.SparkBase.MotorType.kBrushless)
        self._pivot_encoder = self._pivot_motor.getAbsoluteEncoder()
        #self._pivot_controller = self._pivot_motor.getPIDController()

        pivot_settings = rev.SparkMaxConfig()

        pivot_settings.inverted(ShooterConstants.INVERT_PIVOT_MOTOR)
        #self._pivot_controller.setFeedbackDevice(self._pivot_encoder)
        #self._pivot_controller.setP(ShooterConstants.PIVOT_kP)
        #self._pivot_controller.setD(ShooterConstants.PIVOT_kD)

        pivot_settings.smartCurrentLimit(60)

        # Position conversion factor and inversion MUST be set before zero offset
        pivot_settings.absoluteEncoder.positionConversionFactor(360)
        pivot_settings.absoluteEncoder.inverted(ShooterConstants.INVERT_PIVOT_ENCODER)
        pivot_settings.absoluteEncoder.zeroOffset(ShooterConstants.PIVOT_OFFSET)

        self._pivot_motor.configure(pivot_settings, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        #self._pivot_controller.setPositionPIDWrappingEnabled(False)
        # self._pivot_controller.setPositionPIDWrappingMaxInput(360)
        # self._pivot_controller.setPositionPIDWrappingMinInput(0)

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
        # TODO
        #self._pivot_controller.setReference(angle.degrees(), rev.CANSparkMax.ControlType.kPosition)

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
