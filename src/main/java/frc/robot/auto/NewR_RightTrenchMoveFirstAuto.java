package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class NewR_RightTrenchMoveFirstAuto extends AutoMode {
  private final AutoRoutine routine;
  private final AutoTrajectory newRedRightNeutralZone;
  private final AutoTrajectory redRightTransitionToNZ;

  public NewR_RightTrenchMoveFirstAuto(
      Drive drivetrain,
      Feeder feederSubsystem,
      Intake intakeSubsystem,
      Launcher launcherSubsystem) {
    super(drivetrain, feederSubsystem, intakeSubsystem);
    routine = getAutoFactory().newRoutine("NewR_RightTrenchMoveFirstAuto");
    newRedRightNeutralZone = routine.trajectory("NewRedRightNeutralZone");
    redRightTransitionToNZ = routine.trajectory("RedRightTransitionToNZ");
  }

  @Override
  public String getName() {
    return "RedRightTrenchMoveFirstAuto";
  }

  @Override
  public AutoTrajectory getInitialTrajectory() {
    return newRedRightNeutralZone;
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    routine
        .active()
        .onTrue(
            Commands.sequence(
                newRedRightNeutralZone.resetOdometry(),
                Commands.parallel(
                    newRedRightNeutralZone.cmd(),
                    Commands.sequence(
                        Commands.waitSeconds(0.8), intake.getDeployCommand().withTimeout(9.2)))));

    newRedRightNeutralZone
        .done()
        .onTrue(
            Commands.sequence(
                stopDrive(),
                shakeAndFeed(5.0),
                Commands.parallel(
                    redRightTransitionToNZ.cmd(), intake.getDeployCommand().withTimeout(5.0))));

    redRightTransitionToNZ.done().onTrue(Commands.sequence(stopDrive(), shakeAndFeed(5.0)));

    return routine;
  }
}
