package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;

public abstract class AutoMode {
  private final AutoFactory autoFactory;

  public AutoMode(Drive drivetrain) {
    autoFactory =
        new AutoFactory(
            drivetrain::getVisionPose,
            drivetrain::setOdometryPose,
            drivetrain::followTrajectory,
            false,
            drivetrain);
  }

  public AutoFactory getAutoFactory() {
    return this.autoFactory;
  }

  public abstract AutoRoutine getAutoRoutine();

  protected abstract AutoTrajectory getInitialTrajectory();

  public abstract String getName();

  public Optional<Pose2d> getInitialPose() {
    return getInitialTrajectory().getInitialPose();
  }
  ;

  public SwerveSample[] getLoggableTrajectory() {
    SwerveSample[] trajArray = new SwerveSample[0];
    return getInitialTrajectory().getRawTrajectory().samples().toArray(trajArray);
  }
}
