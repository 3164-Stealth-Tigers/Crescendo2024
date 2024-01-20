import commands2
import wpilib

import vision
from config import swerve_components
from config.constants import Operation, Physical, Software
from swervepy import SwerveDrive

from commands.swerve import ski_stop_command
from oi import XboxDriver


class RobotContainer:
    def __init__(self):
        # Driver Xbox controller
        self.stick = XboxDriver(Operation.DRIVER_JOYSTICK_ID)

        # Construct the swerve drivetrain
        self.swerve = SwerveDrive(
            swerve_components.MODULES,
            swerve_components.GYRO,
            Physical.MAX_SPEED,
            Physical.MAX_ANGULAR_SPEED,
            vision.get_estimated_global_pose_2d,
        )

        self.teleop_command = self.swerve.teleop_command(
            self.stick.forward,
            self.stick.strafe,
            self.stick.turn,
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
        return commands2.PrintCommand("Autonomous not implemented!")

    def configure_button_bindings(self):
        """Bind buttons on the Xbox controllers to run Commands"""

        # Reset the robot's heading reading to 0
        self.stick.reset_gyro.onTrue(commands2.InstantCommand(self.swerve.zero_heading))

        # Toggle field-relative control
        self.stick.toggle_field_relative.onTrue(commands2.InstantCommand(self.teleop_command.toggle_field_relative))

        # Point the wheels in an 'X' direction to make the robot harder to push
        # Cancels when the driver starts driving again
        self.stick.ski_stop.onTrue(ski_stop_command(self.swerve).until(self.stick.is_movement_commanded))
