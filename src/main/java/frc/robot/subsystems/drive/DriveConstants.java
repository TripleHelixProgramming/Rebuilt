// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.MotorConstants.NEOVortexConstants;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 3.0; // Commented out on reefscape bot
  public static final double odometryFrequency = 100.0; // Hz
  public static final double wheelBase = Units.inchesToMeters(27);
  public static final double trackWidth = Units.inchesToMeters(21);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(2.287);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(-0.273);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(-1.399);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(1.846);

  // Device CAN IDs
  public static final int gyroCanId = 0;

  public static final int frontLeftDriveCanId = 28;
  public static final int backLeftDriveCanId = 12;
  public static final int frontRightDriveCanId = 20;
  public static final int backRightDriveCanId = 10;

  public static final int frontLeftTurnCanId = 29;
  public static final int backLeftTurnCanId = 13;
  public static final int frontRightTurnCanId = 21;
  public static final int backRightTurnCanId = 11;

  public static final int frontLeftTurnAbsoluteEncoderCanId = 43;
  public static final int backLeftTurnAbsoluteEncoderCanId = 45;
  public static final int frontRightTurnAbsoluteEncoderCanId = 33;
  public static final int backRightTurnAbsoluteEncoderCanId = 31;

  // Drive motor configuration
  public static final double wheelRadiusMeters = 0.0235;
  public static final double driveMotorReduction = 6.75; // SDS MK4 L2
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final double turnMotorReduction = 12.8; // SDS MK4
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Absolute turn encoder configuration
  public static final boolean turnEncoderInverted = false;

  // Relative turn encoder configuration
  public static final double turnEncoderPositionFactor =
      (2 * Math.PI) / turnMotorReduction; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / turnMotorReduction; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              NEOVortexConstants.kDefaultCurrentLimit,
              1),
          moduleTranslations);
}
