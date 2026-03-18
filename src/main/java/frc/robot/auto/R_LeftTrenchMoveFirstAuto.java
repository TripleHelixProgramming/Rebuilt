package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class R_LeftTrenchMoveFirstAuto extends AutoMode {
  private final AutoRoutine routine;
  private final AutoTrajectory redLeftNeutralZone;
  private final AutoTrajectory redLeftTransitionToNZ;

  public R_LeftTrenchMoveFirstAuto(
      Drive drivetrain,
      Feeder feederSubsystem,
      Intake intakeSubsystem,
      Launcher launcherSubsystem) {
    super(drivetrain, feederSubsystem, intakeSubsystem);
    routine = getAutoFactory().newRoutine("R_LeftTrenchMoveFirstAuto");
    redLeftNeutralZone = routine.trajectory("RedLeftNinetyNeutralZone");
    redLeftTransitionToNZ = routine.trajectory("RedLeftTransitionToNZ");
  }

  @Override
  public String getName() {
    return "RedLeftTrenchMoveFirstAuto";
  }

  @Override
  public AutoTrajectory getInitialTrajectory() {
    return redLeftNeutralZone;
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    routine
        .active()
        .onTrue(
            Commands.sequence(
                redLeftNeutralZone.resetOdometry(),
                Commands.parallel(
                    redLeftNeutralZone.cmd(),
                    Commands.sequence(
                        Commands.waitSeconds(0.8), intake.getDeployCommand().withTimeout(9.2)))));

    redLeftNeutralZone
        .done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(drive::stop, drive),
                shakeAndFeed(5.0),
                Commands.parallel(
                    redLeftTransitionToNZ.cmd(), intake.getDeployCommand().withTimeout(5.0))));

    redLeftTransitionToNZ
        .done()
        .onTrue(
            Commands.sequence(Commands.runOnce(drive::stop, drive), shakeAndFeed(5.0)));

    return routine;
  }
}
