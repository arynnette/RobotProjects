// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

import java.util.function.DoubleSupplier;
import java.util.List;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation2d;

public class TeleopDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private SwerveSubsystem swerve;
  private DoubleSupplier vx;
  private DoubleSupplier vy;
  private DoubleSupplier headingX;
  private DoubleSupplier headingY;
  private boolean isOpenLoop;

  public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier headingX, DoubleSupplier headingY, boolean isOpenLoop) {
    this.swerve = swerve;
    this.vx = vx;
    this.vy = vy;
    this.headingX = headingX;
    this.headingY = headingY;
    this.isOpenLoop = isOpenLoop;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds goalSpeeds = swerve.getTargetVelocities(
      vx.getAsDouble(),
      vy.getAsDouble(),
      headingX.getAsDouble(),
      headingY.getAsDouble()
    );

    Translation2d chassisTranslation = SwerveController.getTranslation2d(goalSpeeds);
    chassisTranslation = SwerveMath.limitVelocity(
      chassisTranslation,
      swerve.getFieldRelativeVelocity(),
      swerve.getPose(),
      Constants.DriveConstants.LOOP_TIME,
      Constants.PhysicalConstants.ROBOT_MASS,
      List.of(Constants.PhysicalConstants.PHYS_WRIST), 
      swerve.getSwerveConfigs()
    );

    SmartDashboard.putString("Translation", chassisTranslation.toString());
    swerve.drive(chassisTranslation, goalSpeeds.omegaRadiansPerSecond, true, isOpenLoop);
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
