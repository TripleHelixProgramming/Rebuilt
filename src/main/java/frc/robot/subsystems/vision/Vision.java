// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final Supplier<Pose2d> poseSupplier;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  LinearFilter ambiguityTestPassRate = LinearFilter.movingAverage(20);
  LinearFilter flatPoseTestPassRate = LinearFilter.movingAverage(20);
  LinearFilter withinBoundsTestPassRate = LinearFilter.movingAverage(20);
  LinearFilter moreThanZeroTagsTestPassRate = LinearFilter.movingAverage(20);

  LinearFilter[] cameraPassRate = {
    LinearFilter.movingAverage(20),
    LinearFilter.movingAverage(20),
    LinearFilter.movingAverage(20),
    LinearFilter.movingAverage(20)
  };

  public Vision(VisionConsumer consumer, Supplier<Pose2d> poseSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.poseSupplier = poseSupplier;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    var allTagPoses = new ArrayList<Pose3d>();
    var allRobotPoses = new ArrayList<Pose3d>();
    var allRobotPosesAccepted = new ArrayList<Pose3d>();
    var allRobotPosesRejected = new ArrayList<Pose3d>();

    // List to store acceptable observations along with their calculated standard deviations
    var acceptableObservations = new ArrayList<ObservationWithStdDev>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      var tagPoses = new ArrayList<Pose3d>();
      var robotPoses = new ArrayList<Pose3d>();
      var robotPosesAccepted = new ArrayList<Pose3d>();
      var robotPosesRejected = new ArrayList<Pose3d>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = getAprilTagLayout().getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean acceptPose =
            // Must have observed at least one tag
            moreThanZeroTags(observation)

                // Any single-tag observation must have low ambiguity
                && hasLowAmbiguity(observation)

                // Pose must be flat on the floor
                && isPoseFlat(observation)

                // Pose must be within the field boundaries
                && isWithinBoundaries(observation)

            // Pose must be within the max possible travel distance
            // TODO: Disable this filter during initial robot setup
            // || observation
            //         .pose()
            //         .toPose2d()
            //         .getTranslation()
            //         .getDistance(poseSupplier.get().getTranslation())
            //     > maxTravelDistance.in(Meters);
            ;

        // Add pose to log
        robotPoses.add(observation.pose());
        if (acceptPose) {
          robotPosesAccepted.add(observation.pose());
        } else {
          robotPosesRejected.add(observation.pose());
        }

        cameraPassRate[cameraIndex].calculate(acceptPose ? 1.0 : 0.0);

        // Skip if rejected
        if (!acceptPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            (observation.averageTagDistance() * observation.averageTagDistance())
                / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Pair the observation with its standard deviations and add it to the list
        acceptableObservations.add(
            new ObservationWithStdDev(
                observation, VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/PassRate",
          cameraPassRate[cameraIndex].lastValue());
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Sort the list of acceptable observations by timestamp
    acceptableObservations.sort(Comparator.comparingDouble(o -> o.observation.timestamp()));

    // Send sorted vision observations to the pose estimator
    for (ObservationWithStdDev obsWithStdDev : acceptableObservations) {
      consumer.accept(
          obsWithStdDev.observation.pose().toPose2d(),
          obsWithStdDev.observation.timestamp(),
          obsWithStdDev.stdDevs);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(Pose3d[]::new));

    Logger.recordOutput("Vision/Summary/AmbiguityTestPassRate", ambiguityTestPassRate.lastValue());
    Logger.recordOutput("Vision/Summary/FlatPoseTestPassRate", flatPoseTestPassRate.lastValue());
    Logger.recordOutput(
        "Vision/Summary/WithinBoundsTestPassRate", withinBoundsTestPassRate.lastValue());
    Logger.recordOutput(
        "Vision/Summary/MoreThanZeroTagsTestPassRate", moreThanZeroTagsTestPassRate.lastValue());
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  // Associate observations with their standard deviations
  public static record ObservationWithStdDev(PoseObservation observation, Matrix<N3, N1> stdDevs) {}

  // Caching for AprilTag layout
  public static AprilTagFieldLayout cachedLayout = null;

  /** Returns the AprilTag layout to use, loading it if necessary. */
  public static AprilTagFieldLayout getAprilTagLayout() {
    if (cachedLayout == null) {
      // Try to load custom layout only if requested and not connected to FMS
      if (useCustomAprilTagLayout && !DriverStation.isFMSAttached()) {
        try {
          cachedLayout = new AprilTagFieldLayout(customAprilTagLayoutPath);
        } catch (IOException e) {
          System.err.println("Error loading custom AprilTag layout: " + e.getMessage());
        }
      }
      // Otherwise load default layout
      if (cachedLayout == null) {
        cachedLayout = AprilTagFieldLayout.loadField(defaultAprilTagFieldLayout);
      }
    }
    return cachedLayout;
  }

  public Boolean hasLowAmbiguity(PoseObservation observation) {
    boolean pass = true;
    if (observation.tagCount() == 1) {
      pass = observation.ambiguity() < maxAmbiguity;
    }

    ambiguityTestPassRate.calculate(pass ? 1.0 : 0.0);
    return pass;
  }

  public Boolean isPoseFlat(PoseObservation observation) {
    boolean pass =
        Math.abs(observation.pose().getZ()) < maxZError.in(Meters)
            && Math.abs(observation.pose().getRotation().getX()) < maxRollError.in(Radians)
            && Math.abs(observation.pose().getRotation().getY()) < maxPitchError.in(Radians);

    flatPoseTestPassRate.calculate(pass ? 1.0 : 0.0);
    return pass;
  }

  public Boolean isWithinBoundaries(PoseObservation observation) {
    boolean pass =
        observation.pose().getX() > 0.0
            && observation.pose().getX() < getAprilTagLayout().getFieldLength()
            && observation.pose().getY() > 0.0
            && observation.pose().getY() < getAprilTagLayout().getFieldWidth();

    withinBoundsTestPassRate.calculate(pass ? 1.0 : 0.0);
    return pass;
  }

  public Boolean moreThanZeroTags(PoseObservation observation) {
    boolean pass = observation.tagCount() > 0;

    moreThanZeroTagsTestPassRate.calculate(pass ? 1.0 : 0.0);
    return pass;
  }
}
