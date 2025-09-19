package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import org.photonvision.EstimatedRobotPose;

public class Vision extends SubsystemBase {
  private final Drive drivetrain;

  public Vision(Drive drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    getPoseEstimates()
        .forEach(
            est ->
                drivetrain.addVisionMeasurement(
                    est.pose().estimatedPose.toPose2d(), est.pose().timestampSeconds, est.stdev()));
  }

  public Optional<Pose2d> getPose() {
    return getEstimatedGlobalPose().map(est -> est.estimatedPose.toPose2d());
  }

  /** Choose the pose estimate with the lowest maximum stdev. */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {

    return Arrays.stream(Camera.values())
        .sorted(
            (lhs, rhs) ->
                (int)
                    Math.signum(
                        lhs.getEstimationStdDevs().max() - rhs.getEstimationStdDevs().max()))
        .map(cam -> cam.getEstimatedGlobalPose())
        .filter(Optional::isPresent)
        .map(Optional::get)
        .findFirst();
  }

  public record EstimatedPoseWithStdevs(
      EstimatedRobotPose pose, Matrix<N3, N1> stdev, String name) {}

  /*
   * Returns (possibly empty) list of camera pose estimates sorted by
   * ascending timestamp.
   */
  public List<EstimatedPoseWithStdevs> getPoseEstimates() {
    record Tuple<T1, T2, T3>(T1 t1, T2 t2, T3 t3) {}

    return Arrays.stream(Camera.values())
        // return List.of(Camera.FrontRight).stream()
        .map(
            cam ->
                new Tuple<>(
                    cam.getEstimatedGlobalPose(), cam.getEstimationStdDevs(), cam.toString()))
        .filter(tuple -> tuple.t1.isPresent())
        .map(tuple -> new EstimatedPoseWithStdevs(tuple.t1.get(), tuple.t2, tuple.t3))
        .sorted(
            (lhs, rhs) -> (int) Math.signum(lhs.pose.timestampSeconds - rhs.pose.timestampSeconds))
        .collect(Collectors.toList());
  }
}
