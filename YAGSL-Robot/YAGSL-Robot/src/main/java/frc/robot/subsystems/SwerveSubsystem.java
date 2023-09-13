// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.io.IOException;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  SwerveDrive swerve;

  public SwerveSubsystem(File swerveDirectory) {
    File swerveJsonDirectory = swerveDirectory;
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerve = new SwerveParser(swerveJsonDirectory).createSwerveDrive();
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

    public void drive(Translation2d translation, double theta, boolean isFieldOriented, boolean isOpenLoop) {
      swerve.drive(translation, theta, isFieldOriented, isOpenLoop);
    }

    public Pose2d getPose() {
      return swerve.getPose();
    }

    public void resetOdom(Pose2d pose) {
      swerve.resetOdometry(pose);
  }

  public void setChassisSpeeds(ChassisSpeeds cs) {
    swerve.setChassisSpeeds(cs);
  }

  public void addTrajectory(Trajectory traj) {
    swerve.postTrajectory(traj);
  }

  public void zeroGyro() {
    swerve.zeroGyro();
  }

  public void motorsBrake(boolean brake) {
    swerve.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading() {
    return swerve.getYaw();
  }

  public ChassisSpeeds getTargetVelocities(double x, double y, double thetaX, double thetaY) {
    x = Math.pow(x, 3);
    y = Math.pow(y, 3);
    return swerve.getSwerveController().getTargetSpeeds(x, y, thetaX, thetaY, getHeading().getRadians());
  }

  public ChassisSpeeds getFieldRelativeVelocity() {
    return swerve.getFieldVelocity();
  }

  public ChassisSpeeds getRobotRelativeVelocity() {
    return swerve.getRobotVelocity();
  }

  public SwerveDriveConfiguration getSwerveConfigs() {
    return swerve.swerveDriveConfiguration;
  }

  public void lockDrive() {
    swerve.lockPose();
  }



  @Override
  public void periodic() {
    
  }
}
