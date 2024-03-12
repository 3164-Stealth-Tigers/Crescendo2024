import phoenix5
from wpimath.geometry import Translation2d, Rotation2d

from swervepy import u
from swervepy.impl import (
    Falcon500CoaxialDriveComponent,
    Falcon500CoaxialAzimuthComponent,
    Pigeon2Gyro,
    AbsoluteCANCoder,
    CoaxialSwerveModule,
)

from .constants import SwerveConstants

DRIVE_COMPONENT_CLASS = Falcon500CoaxialDriveComponent
STEERING_COMPONENT_CLASS = Falcon500CoaxialAzimuthComponent
GYRO_CLASS = Pigeon2Gyro
ABSOLUTE_ENCODER_CLASS = AbsoluteCANCoder

DRIVE_COMPONENT_PARAMS = DRIVE_COMPONENT_CLASS.Parameters(
    wheel_circumference=SwerveConstants.WHEEL_CIRCUMFERENCE,
    gear_ratio=SwerveConstants.DRIVE_GEAR_RATIO,
    max_speed=SwerveConstants.MAX_SPEED,
    open_loop_ramp_rate=SwerveConstants.DRIVE_OPEN_LOOP_RAMP_RATE,
    closed_loop_ramp_rate=SwerveConstants.DRIVE_CLOSED_LOOP_RAMP_RATE,
    continuous_current_limit=SwerveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT,
    peak_current_limit=SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT,
    peak_current_duration=0.1,
    neutral_mode=phoenix5.NeutralMode.Coast,
    kP=SwerveConstants.DRIVE_kP,
    kI=0,
    kD=0,
    kS=SwerveConstants.DRIVE_kS,
    kV=SwerveConstants.DRIVE_kV,
    kA=SwerveConstants.DRIVE_kA,
    invert_motor=SwerveConstants.DRIVE_MOTOR_INVERTED,
)
STEERING_COMPONENT_PARAMS = STEERING_COMPONENT_CLASS.Parameters(
    gear_ratio=SwerveConstants.STEERING_GEAR_RATIO,
    max_angular_velocity=SwerveConstants.MAX_ANGULAR_SPEED,
    ramp_rate=0,
    continuous_current_limit=SwerveConstants.STEERING_CONTINUOUS_CURRENT_LIMIT,
    peak_current_limit=SwerveConstants.STEERING_PEAK_CURRENT_LIMIT,
    peak_current_duration=0.1,
    neutral_mode=phoenix5.NeutralMode.Brake,
    kP=SwerveConstants.STEERING_kP,
    kI=0,
    kD=0,
    invert_motor=SwerveConstants.STEERING_MOTOR_INVERTED,
)

wheel_base = SwerveConstants.WHEEL_BASE.m_as(u.m)
track_width = SwerveConstants.TRACK_WIDTH.m_as(u.m)
MODULE_LOCATIONS = {
    "FL": (wheel_base / 2, track_width / 2),
    "FR": (wheel_base / 2, -track_width / 2),
    "BL": (-wheel_base / 2, track_width / 2),
    "BR": (-wheel_base / 2, -track_width / 2),
}

# Construct final components
MODULES = tuple(
    CoaxialSwerveModule(
        DRIVE_COMPONENT_CLASS(getattr(SwerveConstants, f"{i}_DRIVE_ID"), DRIVE_COMPONENT_PARAMS),
        STEERING_COMPONENT_CLASS(
            getattr(SwerveConstants, f"{i}_STEERING_ID"),
            Rotation2d.fromDegrees(getattr(SwerveConstants, f"{i}_ENCODER_OFFSET")),
            STEERING_COMPONENT_PARAMS,
            ABSOLUTE_ENCODER_CLASS(getattr(SwerveConstants, f"{i}_ENCODER_ID")),
        ),
        Translation2d(*MODULE_LOCATIONS[i]),
    )
    for i in ("FL", "FR", "BL", "BR")
)
GYRO = GYRO_CLASS(SwerveConstants.GYRO_ID, SwerveConstants.GYRO_INVERTED)
