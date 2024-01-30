import math

import commands2
import wpilib
import pathplannerlib as pp
from commands2.sysid import SysIdRoutine

import vision
from config import swerve_components
from config.constants import Operation, Physical, Software, Field
from swervepy import SwerveDrive

from commands.swerve import ski_stop_command
from oi import XboxDriver, PS4Driver


class RobotContainer:
    def __init__(self):
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        # Driver Xbox controller
        self.driver_stick = XboxDriver(Operation.DRIVER_JOYSTICK_ID)
        # To use with a PS4 controller:
        # self.driver_stick = PS4Driver(Operation.DRIVER_JOYSTICK_ID)

        # Xbox controller for running SysId and test functions
        self.test_stick = commands2.button.CommandXboxController(Operation.TEST_JOYSTICK_ID)

        # Construct the swerve drivetrain
        self.swerve = SwerveDrive(
            swerve_components.MODULES,
            swerve_components.GYRO,
            Physical.MAX_SPEED,
            Physical.MAX_ANGULAR_SPEED,
            vision.get_estimated_global_pose_2d,
        )

        # Experimental: Set a default callback for rotation so that we can change it to something else
        # then change it back later
        self.default_turn_source = self.driver_stick.turn

        self.teleop_command = self.swerve.teleop_command(
            self.driver_stick.forward,
            self.driver_stick.strafe,
            self.default_turn_source,
            Software.FIELD_RELATIVE,
            Software.DRIVE_OPEN_LOOP,
        )
        # The teleop command will run whenever no other command is running on the Swerve subsystem
        # (e.g., autonomous, ski stop)
        self.swerve.setDefaultCommand(self.teleop_command)
        # Publish information about the state of the command to Smart Dashboard
        wpilib.SmartDashboard.putData(self.teleop_command)

        # Initialize other subsystems here

        self.configure_button_bindings()

    def get_autonomous_command(self):
        return self.swerve.follow_trajectory_command(
            pp.path.PathPlannerPath.fromPathFile("Drive Forward"),
            Software.TRAJECTORY_PARAMS,
            True,
            True,
        )

    def configure_button_bindings(self):
        """Bind buttons on the Xbox controllers to run Commands"""

        # Reset the robot's heading reading to 0
        self.driver_stick.reset_gyro.onTrue(commands2.InstantCommand(self.swerve.zero_heading))

        # Toggle field-relative control
        self.driver_stick.toggle_field_relative.onTrue(
            commands2.InstantCommand(self.teleop_command.toggle_field_relative)
        )

        # Point the wheels in an 'X' direction to make the robot harder to push
        # Cancels when the driver starts driving again
        self.driver_stick.ski_stop.onTrue(ski_stop_command(self.swerve).until(self.driver_stick.is_movement_commanded))

        # Experimental: Change the rotation input source with a button
        # fmt: off
        self.driver_stick.look_at_speaker.onTrue(
            commands2.InstantCommand(lambda: setattr(self.teleop_command, "rotation", self.alternate_turn_source))
        ).onFalse(
            commands2.InstantCommand(lambda: setattr(self.teleop_command, "rotation", self.default_turn_source))
        )
        # fmt: on

        # SysId commands
        # Y: Dynamic forward
        self.test_stick.y().whileTrue(self.swerve.sys_id_dynamic(SysIdRoutine.Direction.kForward))
        # X: Dynamic backward
        self.test_stick.x().whileTrue(self.swerve.sys_id_dynamic(SysIdRoutine.Direction.kReverse))
        # B: Quasistatic forward
        self.test_stick.b().whileTrue(self.swerve.sys_id_quasistatic(SysIdRoutine.Direction.kForward))
        # A: Quasistatic backward
        self.test_stick.a().whileTrue(self.swerve.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))

    def alternate_turn_source(self):
        # Return a percentage from -1 to 1 that may be used in lieu of a joystick turning input.
        # This method calculates the angular velocity required to turn the robot toward the SPEAKER slot.

        # NOTE: This would probably be better as its own command. Just need to retain the ability
        # to translate the robot with joysticks while the command is active.

        # Locate the correct SPEAKER depending on which alliance we're on
        if wpilib.DriverStation.getAlliance() is wpilib.DriverStation.Alliance.kBlue:
            speaker_pos = Field.BLUE_SPEAKER_POSITION
        else:
            speaker_pos = Field.RED_SPEAKER_POSITION

        bot_pose = self.swerve.pose

        # Calculate the angle from the robot to the SPEAKER
        dy = speaker_pos.y - bot_pose.y
        dx = speaker_pos.x - bot_pose.x
        theta = math.atan2(dy, dx)

        # Calculate the required change in robot heading angle
        theta_error = theta - bot_pose.rotation().radians()
        print(f"Theta Error: {theta_error}")

        # Apply a proportional constant to the rotational error, producing a desired angular velocity
        output = Software.ANGULAR_POSITION_kP * theta_error

        # Return the desired angular velocity as a percentage from -1 to 1
        return output / self.swerve.max_angular_velocity
