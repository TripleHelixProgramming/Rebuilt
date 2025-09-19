package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public enum Camera implements Subsystem {
  // FrontRight("OV2311_TH_8", new Translation3d(0.248, -0.318, 0.513), new Rotation3d(0.0, 0.0,
  // 0.0)),
  // FrontLeft(
  //  "OV2311_TH_5", new Translation3d(0.222, 0.331, 0.513), new Rotation3d(0.0, 0, Math.PI / 2.0)),
  // BackRight(
  //  "OV2311_TH_6",
  // new Translation3d(-0.375, -0.331, 0.513),
  // new Rotation3d(0.0, 0.0, 3.0 * Math.PI / 2.0)),
  // BackLeft(
  //  "OV2311_TH_7", new Translation3d(-0.401, 0.318, 0.513), new Rotation3d(0.0, 0.0, Math.PI));

  // FrontRight("OV2311_TH_4", new Translation3d(0.286, -0.332, 0.361), new Rotation3d(0, 0,
  // 0.768)),
  FrontLeft("OV2311_TH_3", new Translation3d(0.292, 0.316, 0.361), new Rotation3d(0, 0, 5.498)),
  //  BackRight("OV2311_TH_2", new Translation3d(-0.259, -0.346, 0.628), new Rotation3d(0, 0,
  // 4.311)),
  BackLeft("OV2311_TH_1", new Translation3d(-0.252, 0.341, 0.628), new Rotation3d(0, 0, 1.972));

  public final String name;
  public final Transform3d transform;
  public final PhotonCamera device;

  private Optional<EstimatedRobotPose> pose;
  private final PhotonPoseEstimator poseEstimator;
  private Matrix<N3, N1> curStdDevs;

  private Camera(String name, Translation3d translation, Rotation3d rotation) {
    this.name = name;
    this.transform = new Transform3d(translation, rotation);
    this.device = new PhotonCamera(name);
    this.pose = Optional.empty();
    this.curStdDevs = VisionConstants.kMultiTagStdDevs;

    AprilTagFieldLayout tagLayout;
    try {
      tagLayout = new AprilTagFieldLayout(VisionConstants.kStemGymAprilTagLayoutPath);
    } catch (IOException e) {
      System.err.println("Error loading custom AprilTag layout: " + e.getMessage());
      tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    // AprilTagFieldLayout tagLayout =
    //     AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    this.poseEstimator =
        new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, transform);
    register();
  }

  /** Updates the pose estimate for this camera */
  @Override
  public void periodic() {
    device
        .getAllUnreadResults()
        .forEach(
            change -> {
              pose = poseEstimator.update(change);
              updateEstimationStdDevs(pose, change.getTargets());
            });
  }

  @Override
  public String getName() {
    return "Camera." + toString();
  }

  public Optional<Pose2d> getPose() {
    return getEstimatedGlobalPose().map(est -> est.estimatedPose.toPose2d());
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty.
   *
   * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return pose;
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = Constants.VisionConstants.kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = Constants.VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }
}
