// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  // Camera simulation parameters (match real OV2311 cameras)
  private static final int CAMERA_WIDTH_PX = 800;
  private static final int CAMERA_HEIGHT_PX = 600;
  private static final double CAMERA_FOV_DIAG_DEG = 70.0; // OV2311 typical diagonal FOV
  private static final int CAMERA_FPS = 35;
  private static final double CAMERA_AVG_LATENCY_MS = 30.0;
  private static final double CAMERA_LATENCY_STDDEV_MS = 5.0;

  // Detection noise parameters (tune based on real camera behavior)
  private static final double CALIB_ERROR_AVG_PX = 0.25;
  private static final double CALIB_ERROR_STDDEV_PX = 0.08;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(Vision.getAprilTagLayout());
    }

    // Configure camera properties to match real OV2311 cameras
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(
        CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX, Rotation2d.fromDegrees(CAMERA_FOV_DIAG_DEG));
    cameraProperties.setFPS(CAMERA_FPS);
    cameraProperties.setAvgLatencyMs(CAMERA_AVG_LATENCY_MS);
    cameraProperties.setLatencyStdDevMs(CAMERA_LATENCY_STDDEV_MS);
    cameraProperties.setCalibError(CALIB_ERROR_AVG_PX, CALIB_ERROR_STDDEV_PX);

    cameraSim = new PhotonCameraSim(camera, cameraProperties, Vision.getAprilTagLayout());
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Guard: poseSupplier may return null during early initialization before
    // the drive subsystem is fully ready. Skip simulation update in that case.
    Pose2d robotPose = poseSupplier.get();
    if (robotPose != null) {
      visionSim.update(robotPose);
    }
    super.updateInputs(inputs);
  }
}
