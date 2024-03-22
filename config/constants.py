# TODO: Update drivetrain constants for new robot once electrical and mechanical are done with it

import math

import rev
from wpimath.geometry import Transform3d, Rotation3d, Translation2d, Rotation2d

from swervepy import u, TrajectoryFollowerParameters


class SwerveConstants:
    TRACK_WIDTH = 24.75 * u.inch
    WHEEL_BASE = 24.75 * u.inch
    WHEEL_CIRCUMFERENCE = 4 * math.pi * u.inch

    MAX_SPEED = 4.5 * u.m / u.s
    MAX_ANGULAR_SPEED = 11.5 * u.rad / u.s

    DRIVE_GEAR_RATIO = 6.75  # SDS Mk4i L2
    STEERING_GEAR_RATIO = 150 / 7  # SDS Mk4i

    FRONT_DRIVE_MOTOR_INVERTED = False
    FRONT_STEERING_MOTOR_INVERTED = False
    BACK_DRIVE_MOTOR_INVERTED = True
    BACK_STEERING_MOTOR_INVERTED = True
    GYRO_INVERTED = False

    DRIVE_CONTINUOUS_CURRENT_LIMIT = 40
    STEERING_CONTINUOUS_CURRENT_LIMIT = 25
    DRIVE_PEAK_CURRENT_LIMIT = 60
    STEERING_PEAK_CURRENT_LIMIT = 40

    # Time in seconds for drive motors to ramp up to commanded speed
    DRIVE_OPEN_LOOP_RAMP_RATE = 0.25
    DRIVE_CLOSED_LOOP_RAMP_RATE = 0

    # Swerve module absolute encoder offsets (in degrees)
    FL_ENCODER_OFFSET = 195.117188
    FR_ENCODER_OFFSET = 2.988281
    BL_ENCODER_OFFSET = 228.515625
    BR_ENCODER_OFFSET = 38.496094

    INVERT_FRONT_ENCODER = False
    INVERT_BACK_ENCODER = False

    FL_DRIVE_ID = 1
    FL_STEERING_ID = 2
    FL_ENCODER_ID = 1
    FR_DRIVE_ID = 3
    FR_STEERING_ID = 4
    FR_ENCODER_ID = 2
    BL_DRIVE_ID = 5
    BL_STEERING_ID = 6
    BL_ENCODER_ID = 3
    BR_DRIVE_ID = 7
    BR_STEERING_ID = 8
    BR_ENCODER_ID = 4
    GYRO_ID = 0

    # FIELD_RELATIVE: True if "forward" means "down the field"; False if
    # "forward" means "in the direction the robot is facing".  A True value
    # requires a (non-Dummy) gyro.
    FIELD_RELATIVE = True

    # DRIVE_OPEN_LOOP: True if we're not using PID control *for velocity targeting*,
    # i.e. when a target velocity is calculated, do we use the corresponding
    # CoaxialDriveComponent's follow_velocity_open() method (set motor output
    # proportionally based on target and max velocities) or
    # follow_velocity_closed() method (put motor in PID control mode and set
    # target velocity).
    DRIVE_OPEN_LOOP = True

    # PID constants for steering motors. Steering motors always
    # use closed-loop control.
    STEERING_kP = 0.3

    # PID and feedforward constants for drive motors
    DRIVE_kP = 0
    DRIVE_kS = 0
    DRIVE_kV = 0
    DRIVE_kA = 0


class IntakeConstants:
    MOTOR_ID = 10

    GEAR_RATIO = 3 / 1

    INTAKE_VELOCITY = 2  # m/s

    INTAKE_kS = 0
    INTAKE_kV = 0
    INTAKE_kP = 0


class ClimberConstants:
    MOTOR_ID = 14

    CLIMBER_SPEED = 0

    CLIMBER_kV = 0
    CLIMBER_kS = 0
    GEAR_RATIO = 15
    SPOOL_CIRCUMFERENCE = 0
    BOTTOM_LIMIT_SWITCH_TYPE = rev.SparkLimitSwitch.Type.kNormallyOpen


class ShooterConstants:
    FLYWHEEL_LEFT_MOTOR_ID = 13
    FLYWHEEL_RIGHT_MOTOR_ID = 12
    PIVOT_MOTOR_ID = 11

    INVERT_LEFT_FLYWHEEL = True
    INVERT_RIGHT_FLYWHEEL = False

    FLYWHEEL_GEAR_RATIO = 3 / 1
    LARGE_WHEEL_CIRCUMFERENCE = (3 * u.inch).m_as(u.m)

    FLYWHEEL_kS = 0
    FLYWHEEL_kV = 0
    FLYWHEEL_kP = 0
    FLYWHEEL_kD = 0

    PIVOT_kD = 0
    PIVOT_kP = 0.02

    # Offset so the shooter angle is 10 at vertical
    PIVOT_OFFSET = 61.779743 - 10  # Degrees
    INVERT_PIVOT_ENCODER = False
    INVERT_PIVOT_MOTOR = True

    PIVOT_FORWARD_LIMIT = 56.20429611206054 + 10
    PIVOT_REVERSE_LIMIT = 0 + 10


class AutoConstants:
    # PID constants for drivetrain following positions
    ANGULAR_POSITION_kP = 7  # Rad/s per radian of error
    LINEAR_POSITION_kP = 1  # m/s per metre of error

    TRAJECTORY_PARAMS = TrajectoryFollowerParameters(
        SwerveConstants.MAX_SPEED,
        ANGULAR_POSITION_kP,
        LINEAR_POSITION_kP,
    )

    # Shooter Angles
    # Degrees from horizontal. CCW+ from the left
    SHOOTER_LOADING_ANGLE = Rotation2d.fromDegrees(0 + 10)  # 0 + 10
    SPEAKER_SCORING_ANGLE = Rotation2d.fromDegrees(45 + 10)  # TODO: Find angle
    AMP_SCORING_ANGLE = Rotation2d.fromDegrees(320)  # TODO: Find angle


class CoprocessorConstants:
    APRIL_TAG_CAMERA_NAME = "Arducam_OV9782_USB_Camera"
    NOTE_CAMERA_NAME = "Razer_Kiyo_Pro"

    # Transformation from the center of the robot (X,Y center of frame; Z bottom of frame)
    # to the camera (X on the front plate/side with the lens; Y,Z center of the camera).
    # In metres
    ROBOT_TO_CAMERA_TRANSFORMATION = Transform3d(-0.3164, 0, 0.1472, Rotation3d(0, (-45 * u.deg).m_as(u.rad), 0))


class OperationConstants:
    DRIVER_JOYSTICK_ID = 0
    OPERATOR_JOYSTICK_ID = 1
    TEST_JOYSTICK_ID = 2


class FieldConstants:
    # Unless otherwise specified, all positions/poses are global (i.e., relative to the blue alliance origin)
    BLUE_SPEAKER_POSITION = Translation2d(0, 5.5462)
    RED_SPEAKER_POSITION = Translation2d(16.5252, 5.5462)

    BLUE_AMP_POSITION = Translation2d(0, 5.5462)
    RED_AMP_POSITION = Translation2d(16.5252, 5.5462)
