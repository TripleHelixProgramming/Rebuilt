package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import java.util.Optional;

public abstract class AutoMode {
  private final AutoFactory autoFactory;
  protected final Drive drive;
  protected final Feeder feeder;
  protected final Intake intake;

  public AutoMode(Drive drivetrain, Feeder feeder, Intake intake) {
    this.drive = drivetrain;
    this.feeder = feeder;
    this.intake = intake;
    autoFactory =
        new AutoFactory(
            drivetrain::getPose,
            drivetrain::setPose,
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

  public SwerveSample[] getLoggableTrajectory() {
    SwerveSample[] trajArray = new SwerveSample[0];
    return getInitialTrajectory().getRawTrajectory().samples().toArray(trajArray);
  }

  protected Command stopDrive() {
    return DriveCommands.getStopCommand(drive);
  }

  protected Command shakeAndFeed(double timeoutSeconds) {
    return Commands.parallel(feeder.getSpinForwardCommand(), intake.getShakeIntakeCommand())
        .withTimeout(timeoutSeconds);
  }
}
