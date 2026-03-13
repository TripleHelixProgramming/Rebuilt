// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DriveConstants;

public class VisionConstants {

  public static String customAprilTagLayoutPath =
      Filesystem.getDeployDirectory() + "/stemgym-2026.json";
  public static Boolean useCustomAprilTagLayout = false;
  public static AprilTagFields defaultAprilTagFieldLayout = AprilTagFields.k2026RebuiltAndymark;

  // Camera names, must match names configured on coprocessor
  public static String cameraFrontRightName = "OV2311_TH_2026_FR";
  public static String cameraFrontLeftName = "OV2311_TH_2026_FL";
  public static String cameraBackRightName = "OV2311_TH_2026_RR";
  public static String cameraBackLeftName = "OV2311_TH_2026_RL";

  // Robot to camera transforms
  public static Transform3d robotToFrontRightCamera =
      new Transform3d(
          Inches.of(-10.572),
          Inches.of(-12.337),
          Inches.of(16.688),
          // pitch 20 degrees up, yaw 55 degrees right
          new Rotation3d(new Quaternion(0.8735, -0.0802, -0.1540, -0.4547)));
  public static Transform3d robotToFrontLeftCamera =
      new Transform3d(
          Inches.of(-10.572),
          Inches.of(12.337),
          Inches.of(16.688),
          // pitch 20 degrees up, yaw 55 degrees left
          new Rotation3d(new Quaternion(0.8735, 0.0802, -0.1540, 0.4547)));
  public static Transform3d robotToBackRightCamera =
      new Transform3d(
          Inches.of(-13.1623),
          Inches.of(-12.1623),
          Inches.of(20.26674),
          // pitch 15 degrees up, yaw 135 degrees right
          new Rotation3d(new Quaternion(-0.3794, 0.1206, 0.0500, 0.9160)));
  public static Transform3d robotToBackLeftCamera =
      new Transform3d(
          Inches.of(-13.1623),
          Inches.of(12.1623),
          Inches.of(20.26674),
          // pitch 15 degrees up, yaw 135 degrees left
          new Rotation3d(new Quaternion(0.3794, 0.1206, -0.0500, 0.9160)));

  public static Distance minRobotWidth = Inches.of(36.875);

  // Pose filtering thresholds
  public static double ambiguityTolerance = 0.15;
  public static Distance tagDistanceTolerance = Meters.of(4.0);
  public static final double tagDistanceToleranceMeters = tagDistanceTolerance.in(Meters);

  public static Distance elevationTolerance = Meters.of(0.25);
  public static final double elevationToleranceMeters = elevationTolerance.in(Meters);
  public static Angle rollTolerance = Degrees.of(5);
  public static final double rollToleranceRadians = rollTolerance.in(Radians);
  public static Angle pitchTolerance = Degrees.of(5);
  public static final double pitchToleranceRadians = pitchTolerance.in(Radians);

  // Cached values for arena boundary calculations
  public static final double minRobotWidthHalfMeters = minRobotWidth.div(2.0).in(Meters);
  public static final double fieldXLenMeters = frc.game.Field.field_x_len.in(Meters);
  public static final double fieldYLenMeters = frc.game.Field.field_y_len.in(Meters);
  public static Distance maxTravelDistance =
      DriveConstants.drivetrainSpeedLimit.times(Seconds.of(Robot.defaultPeriodSecs));

  // Velocity consistency check thresholds
  public static double velocityCheckTimeoutSeconds =
      0.5; // Skip check if no observation for this long
  public static final double maxReasonableVelocityMps =
      DriveConstants.drivetrainSpeedLimit.in(MetersPerSecond) * 1.5; // Allow some margin

  // Cross-camera correlation thresholds
  // When multiple cameras report similar poses at similar times, we boost confidence.
  // This helps validate observations and reject outliers from miscalibrated cameras.
  public static double correlationTimeWindowSeconds =
      0.050; // 50ms - cameras may not fire simultaneously
  public static Distance correlationPoseThreshold =
      Meters.of(0.15); // How close poses must be to "agree"
  public static final double correlationPoseThresholdMeters = correlationPoseThreshold.in(Meters);
  public static double correlationBoostFactor =
      1.3; // Score multiplier when corroborated by majority

  // Standard deviation baselines
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  public static double maxStdDev = 1.0; // Meters
  public static double minScore = linearStdDevBaseline / maxStdDev;

  // Feature flags
  public static boolean kLogIndividualCameraPoses = false;
  public static boolean kLogSummaryPoses = false;
  public static boolean kLogAcceptedPoses = true;
  public static boolean kLogRejectedPoses = false;

  // Logging frequency (1 = every cycle, 2 = every other cycle, etc.)
  // Higher values reduce CPU load but lose data granularity for replay
  public static int kLoggingDivisor = 2;
}
