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

  public static String customAprilTagLayoutPath =
      Filesystem.getDeployDirectory() + "/stemgym-2026.json";
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

  public static Distance minRobotWidth = Inches.of(36.875);

  // Pose filtering thresholds
  public static double ambiguityTolerance = 0.15;
  public static Distance tagDistanceTolerance = Meters.of(4.0);

  public static Distance elevationTolerance = Meters.of(0.25);
  public static Angle rollTolerance = Degrees.of(5);
  public static Angle pitchTolerance = Degrees.of(5);
  public static Distance maxTravelDistance =
      DriveConstants.maxDriveSpeed.times(Seconds.of(Robot.defaultPeriodSecs));

  // Standard deviation baselines
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  public static double maxStdDev = 1.0; // Meters
  public static double minScore = linearStdDevBaseline / maxStdDev;
}
