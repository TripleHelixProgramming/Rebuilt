// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DriveConstants;

public class VisionConstants {

  public static String customAprilTagLayoutPath = Filesystem.getDeployDirectory() + "/stemgym.json";
  public static Boolean useCustomAprilTagLayout = true;
  public static AprilTagFields defaultAprilTagFieldLayout = AprilTagFields.k2025ReefscapeAndyMark;

  // Camera names, must match names configured on coprocessor
  public static String cameraFrontRightName = "OV2311_TH_4";
  public static String cameraFrontLeftName = "OV2311_TH_3";
  public static String cameraBackRightName = "OV2311_TH_2";
  public static String cameraBackLeftName = "OV2311_TH_1";

  // Robot to camera transforms
  public static Transform3d robotToFrontRightCamera =
      new Transform3d(0.286, -0.332, 0.361, new Rotation3d(0, 0, 0.768));
  public static Transform3d robotToFrontLeftCamera =
      new Transform3d(0.292, 0.316, 0.361, new Rotation3d(0, 0, 5.498));
  public static Transform3d robotToBackRightCamera =
      new Transform3d(-0.259, -0.346, 0.628, new Rotation3d(0, 0, 4.311));
  public static Transform3d robotToBackLeftCamera =
      new Transform3d(-0.252, 0.341, 0.628, new Rotation3d(0, 0, 1.972));

  /**
   * the ratio of best:alternate pose reprojection errors, called ambiguity. This is between 0 and 1
   * (0 being no ambiguity, and 1 meaning both have the same reprojection error). Numbers above 0.2
   * are likely to be ambiguous.
   */
  public static double maxAmbiguity = 0.3;

  // Pose filtering thresholds
  public static Distance maxZError = Meters.of(0.75);
  public static Angle maxRollError = Degrees.of(30);
  public static Angle maxPitchError = Degrees.of(30);
  public static Distance maxTravelDistance =
      DriveConstants.maxDriveSpeed.times(Seconds.of(Robot.defaultPeriodSecs));

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Front Right
        1.0, // Front Left
        1.0, // Back Right
        1.0 // Back Left
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
