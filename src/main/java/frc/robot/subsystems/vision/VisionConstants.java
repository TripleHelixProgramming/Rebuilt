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

  // Yaw consistency threshold - how much vision yaw can differ from gyro yaw
  // This catches ambiguous PnP solutions which almost always have incorrect yaw.
  // Set to 10 degrees to allow for minor calibration differences while rejecting
  // bad PnP solutions that typically have 30-90+ degree yaw errors.
  public static Angle yawTolerance = Degrees.of(10);
  public static final double yawToleranceRadians = yawTolerance.in(Radians);

  // Cached values for arena boundary calculations
  public static final double minRobotWidthHalfMeters = minRobotWidth.div(2.0).in(Meters);
  public static final double fieldXLenMeters = frc.game.Field.field_x_len.in(Meters);
  public static final double fieldYLenMeters = frc.game.Field.field_y_len.in(Meters);

  // Velocity consistency check thresholds
  public static double velocityCheckTimeoutSeconds =
      0.5; // Skip check if no observation for this long
  public static final double maxReasonableVelocityMps =
      DriveConstants.drivetrainSpeedLimit.in(MetersPerSecond) * 1.5; // Allow some margin

  // Score returned by velocityConsistency when it can't verify the observation
  // (no history from this camera, or last observation is older than velocityCheckTimeoutSeconds).
  // Previously this was 1.0 (perfect pass), which let bad first-in-a-while poses through
  // unchallenged. Setting this below 1.0 expresses uncertainty: "I can't confirm this pose
  // is consistent, so it needs stronger evidence from other tests to be accepted."
  // At 0.7, a typical bad pose (~0.61 base) drops to ~0.58 and gets rejected at minScore=0.6.
  // A typical good single-tag observation (~0.75 base) drops to ~0.71, still well above 0.6.
  // At startup (robot placed on field), all cameras get this penalty, but the cross-camera
  // correlation boost (1.3x) compensates — agreeing cameras still converge the pose quickly.
  public static double velocityUncertainScore = 0.6;

  // Cross-camera correlation thresholds
  // When multiple cameras report similar poses at similar times, we boost confidence.
  // This helps validate observations and reject outliers from miscalibrated cameras.
  // Set to 150ms to allow cameras that don't fire simultaneously to still be fused.
  // Analysis showed cameras often fire 80-150ms apart, so 50ms was too narrow.
  public static double correlationTimeWindowSeconds = 0.150;
  public static Distance correlationPoseThreshold =
      Meters.of(0.15); // How close poses must be to "agree"
  public static final double correlationPoseThresholdMeters = correlationPoseThreshold.in(Meters);
  public static double correlationBoostFactor =
      1.4; // Score multiplier when corroborated by majority

  // Standard deviation baselines
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Single-camera observations get this multiplier on stddev to reduce their influence.
  // This reduces jitter from multiple single-camera observations reporting slightly
  // different poses. Multi-camera fused observations (which agree) get no penalty.
  // A value of 3.0 means single-camera observations have 3x less influence than fused.
  public static double singleCameraStdDevMultiplier = 3.0;

  public static double maxStdDev = 1.0; // Meters

  // Minimum score for a vision observation to be accepted into the pose estimator.
  // Observations below this threshold are rejected outright.
  // Previously derived as linearStdDevBaseline / maxStdDev = 0.02, which accepted nearly
  // everything. Log analysis showed that bad poses (wrong PnP solution, ~6m off) scored
  // 0.58-0.61 after the velocityUncertainScore penalty, while good single-tag observations
  // scored 0.65+. Setting minScore to 0.6 rejects the penalized bad poses while preserving
  // 97%+ of legitimate observations. Multi-tag observations (0.9+) are unaffected.
  public static double minScore = 0.65;

  // Number of accepted observations required before enabling the yawConsistency check via
  // the vision-only path (independent of poseAsserted). After this many accepted poses,
  // the heading is assumed to have enough field-relative meaning to validate new observations.
  public static int yawConsistencyMinAccepted = 10;

  // Feature flags
  public static boolean kLogIndividualCameraPoses = false;
  public static boolean kLogSummaryPoses = false;
  public static boolean kLogAcceptedPoses = true;
  public static boolean kLogRejectedPoses = true;

  // Logging frequency (1 = every cycle, 2 = every other cycle, etc.)
  // Higher values reduce CPU load but loses data granularity for replay
  public static int kLoggingDivisor = 2;

  // Vision processing interval (1 = every loop, 5 = every 5th loop = 10Hz at 50Hz robot loop)
  // Higher values batch more observations together for fusion, reducing jitter but adding latency.
  // At 5 loops (100ms batches), cameras have time to all report before fusion decides what agrees.
  public static int processingIntervalLoops = 5;
}
