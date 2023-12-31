// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class PhysicalConstants {
      public static final double ROBOT_MASS = 98.1; // kilograms
      public static final Matter PHYS_WRIST = new Matter(new Translation3d(0, 0, 0), ROBOT_MASS);

  }

  public static class DriveConstants {
      public static final double DRIVE_DEADBAND = 0.05;
      public static final String driveDirectoryName = "swerve";
      public static final double LOOP_TIME = 1.0;

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class VisionConstants {
    String kCamera1Name = "cam1";
    String kCamera2Name = "cam2";
  }
}
