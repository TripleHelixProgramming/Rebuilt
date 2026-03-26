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
  private final AutoTrajectory NewRedRightNeutralZone;
  private final AutoTrajectory NewRedRightTransition;

  public NewR_RightTrenchMoveFirstAuto(
      Drive drivetrain,
      Feeder feederSubsystem,
      Intake intakeSubsystem,
      Launcher launcherSubsystem) {
    super(drivetrain, feederSubsystem, intakeSubsystem);
    routine = getAutoFactory().newRoutine("NewR_RightTrenchMoveFirstAuto");
    NewRedRightNeutralZone = routine.trajectory("NewRedRightNeutralZone");
    NewRedRightTransition = routine.trajectory("NewRedRightTransition");
  }

  @Override
  public String getName() {
    return "NewRedRightNeutralZone";
  }

  @Override
  public AutoTrajectory getInitialTrajectory() {
    return NewRedRightNeutralZone;
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    routine
        .active()
        .onTrue(
            Commands.sequence(
                NewRedRightNeutralZone.resetOdometry(),
                Commands.parallel(
                    NewRedRightNeutralZone.cmd(),
                    Commands.sequence(
                        Commands.waitSeconds(0.8), intake.getDeployCommand().withTimeout(9.2)))));

    NewRedRightNeutralZone.done()
        .onTrue(
            Commands.sequence(
                stopDrive(),
                shakeAndFeed(5.0),
                Commands.parallel(
                    NewRedRightTransition.cmd(), intake.getDeployCommand().withTimeout(5.0))));

    NewRedRightTransition.done().onTrue(Commands.sequence(stopDrive(), shakeAndFeed(5.0)));

    return routine;
  }
}
