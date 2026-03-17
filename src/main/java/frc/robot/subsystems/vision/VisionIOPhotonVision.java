// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  private final DoubleSubscriber fpsSubscriber;

  // Reusable collections to avoid allocations per loop
  private final Set<Short> tagIds = new HashSet<>();
  private final List<PoseObservation> poseObservations = new ArrayList<>();

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    fpsSubscriber =
        NetworkTableInstance.getDefault()
            .getTable("photonvision")
            .getSubTable(name)
            .getDoubleTopic("fps")
            .subscribe(0.0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    inputs.fps = fpsSubscriber.get();
    inputs.latencyMs = 0.0;
    inputs.bestReprojError = 0.0;

    // Clear reusable collections
    tagIds.clear();
    poseObservations.clear();

    for (var result : camera.getAllUnreadResults()) {
      // Track diagnostics from the most recent result (loop overwrites with latest)
      inputs.latencyMs = result.getLatencyMillis();
      // Update latest target observation
      if (result.hasTargets()) {
        var bestTarget = result.getBestTarget();
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(bestTarget.getYaw()),
                Rotation2d.fromDegrees(bestTarget.getPitch()),
                Rotation2d.fromDegrees(bestTarget.getSkew()),
                bestTarget.getArea(),
                bestTarget.getDetectedObjectConfidence(),
                bestTarget.getDetectedObjectClassID());
      } else {
        inputs.latestTargetObservation =
            new TargetObservation(Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero, 0, -1, -1);
      }

      // Add pose observation
      if (result.multitagResult.isPresent()) { // Multitag result
        var multitagResult = result.multitagResult.get();

        // Calculate robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Guard: avoid division by zero if targets list is unexpectedly empty.
        // In practice, multitagResult should only exist with 2+ targets, but we guard defensively.
        int targetCount = result.targets.size();
        double avgTagDistance = targetCount > 0 ? totalTagDistance / targetCount : 0.0;

        // Track reprojection error from the most recent multitag result
        inputs.bestReprojError = multitagResult.estimatedPose.bestReprojErr;

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                avgTagDistance, // Average tag distance
                PoseObservationType.PHOTONVISION)); // Observation type

      } else if (!result.targets.isEmpty()) { // Single tag result
        var target = result.targets.get(0);

        // Calculate robot pose
        var tagPoseOpt = Vision.getAprilTagLayout().getTagPose(target.fiducialId);
        if (tagPoseOpt.isPresent()) {
          // Cache the Optional value to avoid multiple .get() calls and ensure consistency
          Pose3d tagPose = tagPoseOpt.get();
          Transform3d fieldToTarget =
              new Transform3d(tagPose.getTranslation(), tagPose.getRotation());
          Transform3d cameraToTarget = target.bestCameraToTarget;
          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          // Add tag ID
          tagIds.add((short) target.fiducialId);

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  robotPose, // 3D pose estimate
                  target.poseAmbiguity, // Ambiguity
                  1, // Tag count
                  cameraToTarget.getTranslation().getNorm(), // Average tag distance
                  PoseObservationType.PHOTONVISION)); // Observation type
        }
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
