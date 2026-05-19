// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

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
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionFilter.FusedObservation;
import frc.robot.subsystems.vision.VisionFilter.Test;
import frc.robot.subsystems.vision.VisionFilter.TestedObservation;
import frc.robot.util.VisionThread;
import frc.robot.util.VisionThread.VisionInputs;
import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final BooleanSupplier poseAssertedSupplier;
  private final Supplier<Rotation2d> headingSupplier;
  private final VisionIO[] io;
  private final VisionInputs[] visionInputs;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  // Initialize logging values
  private ArrayList<Pose3d> allTagPoses = new ArrayList<Pose3d>();
  private ArrayList<Pose3d> allRobotPoses = new ArrayList<Pose3d>();
  private ArrayList<Pose3d> allRobotPosesAccepted = new ArrayList<Pose3d>();
  private ArrayList<Pose3d> allRobotPosesRejected = new ArrayList<Pose3d>();

  // Buffer to accumulate observations across loops for batched processing
  // Cleared after processing every processingIntervalLoops
  private ArrayList<TestedObservation> observationBuffer = new ArrayList<TestedObservation>();

  // Initialize logging values
  private ArrayList<Pose3d> tagPoses = new ArrayList<Pose3d>();
  private ArrayList<Pose3d> robotPoses = new ArrayList<Pose3d>();
  private ArrayList<Pose3d> robotPosesAccepted = new ArrayList<Pose3d>();
  private ArrayList<Pose3d> robotPosesRejected = new ArrayList<Pose3d>();

  // Vision filter for scoring observations
  private final VisionFilter visionFilter = new VisionFilter();

  LinearFilter[] cameraPassRate = {
    LinearFilter.movingAverage(20),
    LinearFilter.movingAverage(20),
    LinearFilter.movingAverage(20),
    LinearFilter.movingAverage(20)
  };

  // Per-camera tracking for velocity consistency check
  private final Pose2d[] lastAcceptedPose;
  private final double[] lastAcceptedTimestamp;

  // Cycle counter for throttled logging
  private int loopCounter = 0;

  // Set true when multiple cameras independently agree on a pose, confirming the heading
  // is field-relative. Enables yawConsistency without requiring an authoritative pose from auto.
  private boolean headingCorroborated = false;

  // Vision tests to apply (remove from set to disable specific tests)
  public static final EnumSet<Test> enabledTests = VisionFilter.DEFAULT_ENABLED_TESTS;

  public Vision(
      VisionConsumer consumer,
      BooleanSupplier poseAssertedSupplier,
      Supplier<Rotation2d> headingSupplier,
      VisionIO... io) {
    this.consumer = consumer;
    this.poseAssertedSupplier = poseAssertedSupplier;
    this.headingSupplier = headingSupplier;
    this.io = io;

    // Initialize per-camera velocity tracking arrays
    this.lastAcceptedPose = new Pose2d[io.length];
    this.lastAcceptedTimestamp = new double[io.length];

    // Register each VisionIO with VisionThread for background polling
    this.visionInputs = new VisionInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      visionInputs[i] = VisionThread.getInstance().registerVisionIO(io[i]);
    }

    // Initialize inputs for AdvantageKit logging
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
   * Returns the yaw angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    /* TODO: don't use latestTargetObservation: it's just the most recent observation,
     *   not necessarily a good one, possibly even the default EMPTY_TARGET.
     */
    throw new RuntimeException("Don't call me until I'm fixed");
    // return inputs[cameraIndex].latestTargetObservation.yaw();
  }

  @Override
  public void periodic() {
    long visionStart = Constants.FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;
    loopCounter++;

    // Copy cached inputs from background thread (should be fast - volatile reads)
    for (int i = 0; i < io.length; i++) {
      visionInputs[i].getSnapshot().copyTo(inputs[i]);
    }
    long t1 = Constants.FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Log inputs via AdvantageKit (throttled - serialization is expensive)
    // Note: Throttling reduces CPU load but loses data granularity for replay
    if (loopCounter % kLoggingDivisor == 0) {
      for (int i = 0; i < io.length; i++) {
        Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
      }
    }
    long t2 = Constants.FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Initialize logging values
    allTagPoses.clear();
    allRobotPoses.clear();
    allRobotPosesAccepted.clear();
    allRobotPosesRejected.clear();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
      Logger.recordOutput(
          "Faults/Vision/Camera" + cameraIndex + "Disconnected", !inputs[cameraIndex].connected);

      // Initialize logging values
      tagPoses.clear();
      robotPoses.clear();
      robotPosesAccepted.clear();
      robotPosesRejected.clear();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = getAprilTagLayout().getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Score the observation using VisionFilter
        TestedObservation tested =
            visionFilter.scoreObservation(
                observation,
                cameraIndex,
                lastAcceptedPose[cameraIndex],
                lastAcceptedTimestamp[cameraIndex],
                (poseAssertedSupplier.getAsBoolean() || headingCorroborated)
                    ? headingSupplier.get()
                    : null,
                enabledTests);

        observationBuffer.add(tested);

        // Add pose to log
        robotPoses.add(observation.pose());
        if (tested.score() > minScore) {
          robotPosesAccepted.add(observation.pose());
        } else {
          robotPosesRejected.add(observation.pose());
        }

        cameraPassRate[cameraIndex].calculate(tested.score());
      }

      // Log camera datadata
      if (kLogIndividualCameraPoses) {
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
      }
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    long t3 = Constants.FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Process observations in batches every processingIntervalLoops
    // This allows cameras to accumulate observations before fusion decides what agrees
    if (loopCounter % processingIntervalLoops == 0) {
      // Remove unacceptable observations before fusion
      observationBuffer.removeIf(o -> o.score() < minScore);

      // Fuse correlated observations from multiple cameras into averaged poses
      // This reduces jitter from multiple cameras reporting slightly different poses
      ArrayList<FusedObservation> fusedObservations =
          visionFilter.fuseCorrelatedObservations(observationBuffer);

      // Sort fused observations by timestamp
      fusedObservations.sort((lhs, rhs) -> (int) Math.signum(lhs.timestamp() - rhs.timestamp()));

      for (var fused : fusedObservations) {
        // Calculate standard deviations
        // Single-camera observations get much higher stddev (less trust) to reduce jitter
        // Multi-camera fused observations get lower stddev (more trust)
        double cameraCountFactor = (fused.cameraCount() == 1) ? singleCameraStdDevMultiplier : 1.0;
        double linearStdDev = linearStdDevBaseline * cameraCountFactor / fused.score();
        double angularStdDev = angularStdDevBaseline * cameraCountFactor / fused.score();

        // Multi-camera agreement corroborates the heading, enabling yawConsistency
        if (fused.cameraCount() > 1) {
          headingCorroborated = true;
        }

        // Send fused vision observation to the pose estimator
        consumer.accept(
            fused.pose(),
            fused.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

        Logger.recordOutput("Vision/Summary/ObservationScore", fused.score());
        Logger.recordOutput("Vision/Summary/FusedCameraCount", fused.cameraCount());
      }

      // Clear buffer after processing
      observationBuffer.clear();
    }
    long t4 = Constants.FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Log summary data (throttled along with processInputs)
    if (loopCounter % kLoggingDivisor == 0) {
      if (kLogSummaryPoses) {
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
      }
      if (kLogAcceptedPoses) {
        Logger.recordOutput(
            "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(Pose3d[]::new));
      }
      if (kLogRejectedPoses) {
        Logger.recordOutput(
            "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(Pose3d[]::new));
      }
    }
    long t5 = Constants.FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Profiling output
    if (Constants.FeatureFlags.PROFILING_ENABLED) {
      long totalMs = (t5 - visionStart) / 1_000_000;
      if (totalMs > 5) {
        System.out.println(
            "[Vision] snapshot="
                + (t1 - visionStart) / 1_000_000
                + "ms processInputs="
                + (t2 - t1) / 1_000_000
                + "ms cameraLoop="
                + (t3 - t2) / 1_000_000
                + "ms consumer="
                + (t4 - t3) / 1_000_000
                + "ms summaryLog="
                + (t5 - t4) / 1_000_000
                + "ms total="
                + totalMs
                + "ms");
      }
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  // Caching for AprilTag layout (volatile for thread-safe lazy initialization)
  private static volatile AprilTagFieldLayout cachedLayout = null;

  /** Returns the AprilTag layout to use, loading it if necessary. Thread-safe. */
  public static synchronized AprilTagFieldLayout getAprilTagLayout() {
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
}
