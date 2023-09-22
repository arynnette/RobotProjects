// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;

import frc.robot.Constants.*;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), Constants.DriveConstants.driveDirectoryName));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    TeleopDrive closedLoopAbsDrive = new TeleopDrive(
      m_swerveSubsystem,
      () -> MathUtil.applyDeadband(m_driverController.getLeftY(), Constants.DriveConstants.DRIVE_DEADBAND),
      () -> MathUtil.applyDeadband(m_driverController.getLeftX(), Constants.DriveConstants.DRIVE_DEADBAND),
      () -> -m_driverController.getRightX(),
      () -> -m_driverController.getRightY(),
      false
    );

    AbsoluteFieldDrive closedFieldAbsDrive = new AbsoluteFieldDrive(m_swerveSubsystem,
    () -> MathUtil.applyDeadband(m_driverController.getLeftY(), Constants.DriveConstants.DRIVE_DEADBAND),
    () -> MathUtil.applyDeadband(m_driverController.getLeftX(), Constants.DriveConstants.DRIVE_DEADBAND),
    () -> m_driverController.getRawAxis(2),
    false);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
