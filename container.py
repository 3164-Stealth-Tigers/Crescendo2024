import math

import commands2
import wpilib
import pathplannerlib as pp
from commands2.sysid import SysIdRoutine
from wpimath.geometry import Rotation2d, Translation2d

import limelight
from config import swerve_components
from config.constants import OperationConstants, AutoConstants, FieldConstants, SwerveConstants
from subsystems import Shooter, Intake, Climber
from swervepy import SwerveDrive, u

from commands.swerve import ski_stop_command, drive_command
from oi import XboxDriver, PS4Driver, XboxOperator, DanielXboxOperator


class RobotContainer:
    def __init__(self):
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        # Driver Xbox controller
        self.driver_stick = XboxDriver(OperationConstants.DRIVER_JOYSTICK_ID)
        # To use with a PS4 controller:
        # self.driver_stick = PS4Driver(Operation.DRIVER_JOYSTICK_ID)

        self.operator_stick = DanielXboxOperator(OperationConstants.OPERATOR_JOYSTICK_ID)

        # Xbox controller for running SysId and test functions
        # self.test_stick = commands2.button.CommandXboxController(OperationConstants.TEST_JOYSTICK_ID)

        # Construct the swerve drivetrain
        self.swerve = SwerveDrive(
            swerve_components.MODULES,
            swerve_components.GYRO,
            SwerveConstants.MAX_SPEED,
            SwerveConstants.MAX_ANGULAR_SPEED,
            limelight.get_estimated_pose,
        )

        # Experimental: Set a default callback for rotation so that we can change it to something else
        # then change it back later
        self.default_turn_source = self.driver_stick.turn

        self.teleop_command = self.swerve.teleop_command(
            self.driver_stick.forward,
            self.driver_stick.strafe,
            self.default_turn_source,
            SwerveConstants.FIELD_RELATIVE,
            SwerveConstants.DRIVE_OPEN_LOOP,
        )
        # The teleop command will run whenever no other command is running on the Swerve subsystem
        # (e.g., autonomous, ski stop)
        self.swerve.setDefaultCommand(self.teleop_command)
        # Publish information about the state of the command to Smart Dashboard
        wpilib.SmartDashboard.putData(self.teleop_command)

        # Initialize other subsystems here
        self.shooter = Shooter()
        shooter_command = commands2.RunCommand(
            lambda: self.shooter.run_flywheel_power(self.operator_stick.flywheel())
        ).alongWith(commands2.RunCommand(lambda: self.shooter.run_pivot_power(self.operator_stick.pivot())))
        shooter_command.addRequirements(self.shooter)
        self.shooter.setDefaultCommand(shooter_command)
        wpilib.SmartDashboard.putData(self.shooter)

        self.intake = Intake()
        self.intake.setDefaultCommand(
            commands2.RunCommand(lambda: self.intake.run_intake_power(self.operator_stick.intake()), self.intake)
        )
        wpilib.SmartDashboard.putData(self.intake)

        self.climber = Climber()
        self.climber.setDefaultCommand(
            commands2.RunCommand(lambda: self.climber.run_climber_power(self.operator_stick.climber()), self.climber)
        )

        self.configure_button_bindings()

        # Setup autonomous
        self.auto_chooser = wpilib.SendableChooser()
        self.setup_autonomous()
        wpilib.SmartDashboard.putData("Auto Chooser", self.auto_chooser)

    def setup_autonomous(self):
        initial_translation = self.swerve.pose.translation()
        goal_distance = 2.54  # 100 inch
        drive_fwd_cmd = drive_command(self.swerve, 1, 0, 0).until(
            lambda: self.swerve.pose.translation().distance(initial_translation) >= goal_distance
        )
        self.auto_chooser.setDefaultOption("Drive Forward", drive_fwd_cmd)

        self.auto_chooser.addOption("Do Nothing", commands2.Command())

        score_preloaded_note_auto = commands2.ParallelCommandGroup(
            self.swerve.teleop_command(
                lambda: 0,
                lambda: 0,
                self.look_at_speaker,
                SwerveConstants.FIELD_RELATIVE,
                SwerveConstants.DRIVE_OPEN_LOOP,
            ).until(lambda: -0.01 < self.look_at_speaker() < 0.01),
            commands2.RunCommand(
                lambda: self.shooter.set_angle(AutoConstants.SPEAKER_SCORING_ANGLE),
                self.shooter,
            ).finallyDo(lambda _: self.shooter.run_pivot_power(0)).withTimeout(2)
        ).andThen(
            commands2.RunCommand(
                lambda: self.shooter.run_flywheel_power(1),
                self.shooter,
            ).finallyDo(lambda _: self.shooter.run_flywheel_power(0)).withTimeout(1)
        )
        self.auto_chooser.addOption("Score Preloaded Note [WIP]", score_preloaded_note_auto)

    def get_autonomous_command(self):
        return self.auto_chooser.getSelected()

    def configure_button_bindings(self):
        """Bind buttons on the Xbox controllers to run Commands"""

        ##################
        # Swerve Buttons #
        ##################

        # Reset the robot's heading reading to 0
        self.driver_stick.reset_gyro.onTrue(commands2.InstantCommand(self.swerve.zero_heading))

        # Toggle field-relative control
        self.driver_stick.toggle_field_relative.onTrue(
            commands2.InstantCommand(self.teleop_command.toggle_field_relative)
        )

        self.driver_stick.reset_pose_to_vision.onTrue(
            commands2.InstantCommand(self.swerve.reset_odometry_to_vision)
        )

        # Point the wheels in an 'X' direction to make the robot harder to push
        # Cancels when the driver starts driving again
        self.driver_stick.ski_stop.onTrue(ski_stop_command(self.swerve).until(self.driver_stick.is_movement_commanded))

        # Experimental: Change the rotation input source with a button
        # fmt: off
        self.driver_stick.look_at_speaker.onTrue(
            commands2.InstantCommand(lambda: setattr(self.teleop_command, "rotation", self.look_at_speaker))
        ).onFalse(
            commands2.InstantCommand(lambda: setattr(self.teleop_command, "rotation", self.default_turn_source))
        )
        self.driver_stick.look_at_amp.onTrue(
            commands2.InstantCommand(lambda: setattr(self.teleop_command, "rotation", self.look_at_amp))
        ).onFalse(
            commands2.InstantCommand(lambda: setattr(self.teleop_command, "rotation", self.default_turn_source))
        )
        # fmt: on

        """
        # SysId commands
        # Y: Dynamic forward
        self.test_stick.y().whileTrue(self.swerve.sys_id_dynamic(SysIdRoutine.Direction.kForward))
        # X: Dynamic backward
        self.test_stick.x().whileTrue(self.swerve.sys_id_dynamic(SysIdRoutine.Direction.kReverse))
        # B: Quasistatic forward
        self.test_stick.b().whileTrue(self.swerve.sys_id_quasistatic(SysIdRoutine.Direction.kForward))
        # A: Quasistatic backward
        self.test_stick.a().whileTrue(self.swerve.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))
        """

        ###################
        # Shooter Buttons #
        ###################
        # TODO: Operator buttons for loading (intake), speaker, and amp angle

        # Go to vertical
        self.operator_stick.intake_height.whileTrue(
            self.shooter.shooter_angle_command(AutoConstants.SHOOTER_LOADING_ANGLE)
        )
        # Go to speaker scoring angle
        self.operator_stick.speaker_height.whileTrue(
            self.shooter.shooter_angle_command(AutoConstants.SPEAKER_SCORING_ANGLE)
        )
        # Go to amp scoring angle
        self.operator_stick.amp_height.whileTrue(
            self.shooter.shooter_angle_command(AutoConstants.AMP_SCORING_ANGLE)
        )

    def look_with_shooter(self, target: Translation2d):
        # Return a percentage from -1 to 1 that may be used in lieu of a joystick turning input.
        # This method calculates the angular velocity required to turn the robot toward the SPEAKER slot.

        # NOTE: This would probably be better as its own command. Just need to retain the ability
        # to translate the robot with joysticks while the command is active.

        bot_pose = self.swerve.pose

        # Calculate the angle from the robot's position to the SPEAKER's position
        dy = target.y - bot_pose.y
        dx = target.x - bot_pose.x
        theta = math.atan2(dy, dx)

        # Calculate the required change in robot heading angle
        # Add pi to make the robot "look" with its shooter facing forward instead of its front
        theta_error = theta - (bot_pose.rotation().radians() + math.pi)
        # Two possible changes in heading angle are possible--find the smaller one
        theta_error = (theta_error + math.pi) % (2 * math.pi) - math.pi
        print(f"Theta Error: {theta_error / math.pi:.3f}pi")

        # Apply a proportional constant to the rotational error, producing a desired angular velocity
        output = AutoConstants.ANGULAR_POSITION_kP * theta_error

        # Return the desired angular velocity as a percentage from -1 to 1
        return output / self.swerve.max_angular_velocity

    def look_at_speaker(self):
        # Locate the correct SPEAKER depending on which alliance we're on
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue:
            speaker_pos = FieldConstants.BLUE_SPEAKER_POSITION
        else:
            speaker_pos = FieldConstants.RED_SPEAKER_POSITION

        return self.look_with_shooter(speaker_pos)

    def look_at_amp(self):
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue:
            speaker_pos = FieldConstants.BLUE_AMP_POSITION
        else:
            speaker_pos = FieldConstants.BLUE_SPEAKER_POSITION

        return self.look_with_shooter(speaker_pos)

