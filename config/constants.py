import math

from wpimath.geometry import Transform3d, Rotation3d, Pose2d, Transform2d, Translation2d

from swervepy import u


class Physical:
    TRACK_WIDTH = 21.73 * u.inch
    WHEEL_BASE = 21.73 * u.inch
    WHEEL_CIRCUMFERENCE = 4 * math.pi * u.inch

    MAX_SPEED = 4.5 * u.m / u.s
    MAX_ANGULAR_SPEED = 11.5 * u.rad / u.s

    ROBOT_TO_CAMERA_TRANSFORMATION = Transform3d(0, 0, 0, Rotation3d(0, 0, 0))


class Mechanical:
    DRIVE_GEAR_RATIO = 6.75  # SDS Mk4i L2
    STEERING_GEAR_RATIO = 150 / 7  # SDS Mk4i

    DRIVE_MOTOR_INVERTED = False
    STEERING_MOTOR_INVERTED = True
    GYRO_INVERTED = False


class Electrical:
    DRIVE_CONTINUOUS_CURRENT_LIMIT = 40
    STEERING_CONTINUOUS_CURRENT_LIMIT = 25
    DRIVE_PEAK_CURRENT_LIMIT = 60
    STEERING_PEAK_CURRENT_LIMIT = 40

    # Time in seconds for drive motors to ramp up to commanded speed
    DRIVE_OPEN_LOOP_RAMP_RATE = 0.25
    DRIVE_CLOSED_LOOP_RAMP_RATE = 0

    # Swerve module absolute encoder offsets (in degrees)
    FL_ENCODER_OFFSET = 107.226562
    FR_ENCODER_OFFSET = 160.136719
    BL_ENCODER_OFFSET = 307.089844
    BR_ENCODER_OFFSET = 355.693359

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


class Software:
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
    STEERING_kP = 0.01

    # PID and feedforward constants for drive motors
    DRIVE_kP = 0
    DRIVE_kS = 0
    DRIVE_kV = 0
    DRIVE_kA = 0

    # PID constants for drivetrain following positions
    ANGULAR_POSITION_kP = 1  # Rad/s per radian of error


class Coprocessor:
    CAMERA_NAME = "Arducam_OV9782_USB_Camera"


class Operation:
    DRIVER_JOYSTICK_ID = 0
    TEST_JOYSTICK_ID = 2


class Field:
    # Unless otherwise specified, all positions/poses are global (i.e., relative to the blue alliance origin)
    BLUE_SPEAKER_POSITION = Translation2d(0, 5.5462)
    RED_SPEAKER_POSITION = Translation2d(16.5252, 5.5462)
