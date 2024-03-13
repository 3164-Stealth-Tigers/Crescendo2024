import copy

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

FRONT_DRIVE_COMPONENT_PARAMS = DRIVE_COMPONENT_CLASS.Parameters(
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
    invert_motor=SwerveConstants.FRONT_DRIVE_MOTOR_INVERTED,
)
FRONT_STEERING_COMPONENT_PARAMS = STEERING_COMPONENT_CLASS.Parameters(
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
    invert_motor=SwerveConstants.FRONT_STEERING_MOTOR_INVERTED,
)

BACK_DRIVE_COMPONENT_PARAMS = copy.deepcopy(FRONT_DRIVE_COMPONENT_PARAMS)
BACK_DRIVE_COMPONENT_PARAMS.invert_motor = SwerveConstants.BACK_DRIVE_MOTOR_INVERTED

BACK_STEERING_COMPONENT_PARAMS = copy.deepcopy(FRONT_STEERING_COMPONENT_PARAMS)
BACK_STEERING_COMPONENT_PARAMS.invert_motor = SwerveConstants.BACK_STEERING_MOTOR_INVERTED

wheel_base = SwerveConstants.WHEEL_BASE.m_as(u.m)
track_width = SwerveConstants.TRACK_WIDTH.m_as(u.m)
MODULE_LOCATIONS = {
    "FL": (wheel_base / 2, track_width / 2),
    "FR": (wheel_base / 2, -track_width / 2),
    "BL": (-wheel_base / 2, track_width / 2),
    "BR": (-wheel_base / 2, -track_width / 2),
}

# Construct final components
MODULES = (
    CoaxialSwerveModule(
        DRIVE_COMPONENT_CLASS(SwerveConstants.FL_DRIVE_ID, FRONT_DRIVE_COMPONENT_PARAMS),
        STEERING_COMPONENT_CLASS(
            SwerveConstants.FL_STEERING_ID,
            Rotation2d.fromDegrees(SwerveConstants.FL_ENCODER_OFFSET),
            FRONT_STEERING_COMPONENT_PARAMS,
            ABSOLUTE_ENCODER_CLASS(SwerveConstants.FL_ENCODER_ID, SwerveConstants.INVERT_FRONT_ENCODER),
        ),
        Translation2d(*MODULE_LOCATIONS["FL"])
    ),
    CoaxialSwerveModule(
        DRIVE_COMPONENT_CLASS(SwerveConstants.FR_DRIVE_ID, FRONT_DRIVE_COMPONENT_PARAMS),
        STEERING_COMPONENT_CLASS(
            SwerveConstants.FR_STEERING_ID,
            Rotation2d.fromDegrees(SwerveConstants.FR_ENCODER_OFFSET),
            FRONT_STEERING_COMPONENT_PARAMS,
            ABSOLUTE_ENCODER_CLASS(SwerveConstants.FR_ENCODER_ID, SwerveConstants.INVERT_FRONT_ENCODER),
        ),
        Translation2d(*MODULE_LOCATIONS["FR"])
    ),
    CoaxialSwerveModule(
        DRIVE_COMPONENT_CLASS(SwerveConstants.BL_DRIVE_ID, BACK_DRIVE_COMPONENT_PARAMS),
        STEERING_COMPONENT_CLASS(
            SwerveConstants.BL_STEERING_ID,
            Rotation2d.fromDegrees(SwerveConstants.BL_ENCODER_OFFSET),
            BACK_STEERING_COMPONENT_PARAMS,
            ABSOLUTE_ENCODER_CLASS(SwerveConstants.BL_ENCODER_ID, SwerveConstants.INVERT_BACK_ENCODER),
        ),
        Translation2d(*MODULE_LOCATIONS["BL"])
    ),
    CoaxialSwerveModule(
        DRIVE_COMPONENT_CLASS(SwerveConstants.BR_DRIVE_ID, BACK_DRIVE_COMPONENT_PARAMS),
        STEERING_COMPONENT_CLASS(
            SwerveConstants.BR_STEERING_ID,
            Rotation2d.fromDegrees(SwerveConstants.BR_ENCODER_OFFSET),
            BACK_STEERING_COMPONENT_PARAMS,
            ABSOLUTE_ENCODER_CLASS(SwerveConstants.BR_ENCODER_ID, SwerveConstants.INVERT_BACK_ENCODER),
        ),
        Translation2d(*MODULE_LOCATIONS["BR"])
    ),
)
GYRO = GYRO_CLASS(SwerveConstants.GYRO_ID, SwerveConstants.GYRO_INVERTED)
