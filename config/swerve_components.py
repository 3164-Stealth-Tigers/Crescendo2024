import rev
from wpimath.geometry import Translation2d, Rotation2d

from swervepy import u
from swervepy.impl import (
    NEOCoaxialDriveComponent,
    NEOCoaxialAzimuthComponent,
    Pigeon2Gyro,
    AbsoluteCANCoder,
    CoaxialSwerveModule,
)

from .constants import Physical, Software, Electrical, Mechanical

DRIVE_COMPONENT_CLASS = NEOCoaxialDriveComponent
STEERING_COMPONENT_CLASS = NEOCoaxialAzimuthComponent
GYRO_CLASS = Pigeon2Gyro
ABSOLUTE_ENCODER_CLASS = AbsoluteCANCoder

DRIVE_COMPONENT_PARAMS = DRIVE_COMPONENT_CLASS.Parameters(
    wheel_circumference=Physical.WHEEL_CIRCUMFERENCE,
    gear_ratio=Mechanical.DRIVE_GEAR_RATIO,
    max_speed=Physical.MAX_SPEED,
    open_loop_ramp_rate=Electrical.DRIVE_OPEN_LOOP_RAMP_RATE,
    closed_loop_ramp_rate=Electrical.DRIVE_CLOSED_LOOP_RAMP_RATE,
    continuous_current_limit=Electrical.DRIVE_CONTINUOUS_CURRENT_LIMIT,
    peak_current_limit=Electrical.DRIVE_PEAK_CURRENT_LIMIT,
    neutral_mode=rev.CANSparkMax.IdleMode.kCoast,
    kP=Software.DRIVE_kP,
    kI=0,
    kD=0,
    kS=Software.DRIVE_kS,
    kV=Software.DRIVE_kV,
    kA=Software.DRIVE_kA,
    invert_motor=Mechanical.DRIVE_MOTOR_INVERTED,
)
STEERING_COMPONENT_PARAMS = STEERING_COMPONENT_CLASS.Parameters(
    gear_ratio=Mechanical.STEERING_GEAR_RATIO,
    max_angular_velocity=Physical.MAX_ANGULAR_SPEED,
    ramp_rate=0,
    continuous_current_limit=Electrical.STEERING_CONTINUOUS_CURRENT_LIMIT,
    peak_current_limit=Electrical.STEERING_PEAK_CURRENT_LIMIT,
    neutral_mode=rev.CANSparkMax.IdleMode.kBrake,
    kP=Software.STEERING_kP,
    kI=0,
    kD=0,
    invert_motor=Mechanical.STEERING_MOTOR_INVERTED,
)

wheel_base = Physical.WHEEL_BASE.m_as(u.m)
track_width = Physical.TRACK_WIDTH.m_as(u.m)
MODULE_LOCATIONS = {
    "FL": (wheel_base / 2, track_width / 2),
    "FR": (wheel_base / 2, -track_width / 2),
    "BL": (-wheel_base / 2, track_width / 2),
    "BR": (-wheel_base / 2, -track_width / 2),
}

# Construct final components
MODULES = tuple(
    CoaxialSwerveModule(
        DRIVE_COMPONENT_CLASS(getattr(Electrical, f"{i}_DRIVE_ID"), DRIVE_COMPONENT_PARAMS),
        STEERING_COMPONENT_CLASS(
            getattr(Electrical, f"{i}_STEERING_ID"),
            Rotation2d.fromDegrees(getattr(Electrical, f"{i}_ENCODER_OFFSET")),
            STEERING_COMPONENT_PARAMS,
            ABSOLUTE_ENCODER_CLASS(getattr(Electrical, f"{i}_ENCODER_ID")),
        ),
        Translation2d(*MODULE_LOCATIONS[i]),
    )
    for i in ("FL", "FR", "BL", "BR")
)
GYRO = GYRO_CLASS(Electrical.GYRO_ID, Mechanical.GYRO_INVERTED)
