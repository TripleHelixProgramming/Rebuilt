package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class NewR_LeftTrenchMoveFirstAuto extends AutoMode {
  private final AutoRoutine routine;
  private final AutoTrajectory NewRedLeftNeutralZone;
  private final AutoTrajectory NewRedLeftTransition;

  public NewR_LeftTrenchMoveFirstAuto(
      Drive drivetrain,
      Feeder feederSubsystem,
      Intake intakeSubsystem,
      Launcher launcherSubsystem) {
    super(drivetrain, feederSubsystem, intakeSubsystem);
    routine = getAutoFactory().newRoutine("NewR_LeftTrenchMoveFirstAuto");
    NewRedLeftNeutralZone = routine.trajectory("NewRedLeftNeutralZone");
    NewRedLeftTransition = routine.trajectory("NewRedLeftTransition");
  }

  @Override
  public String getName() {
    return "RedLeftTrenchMoveFirstAuto";
  }

  @Override
  public AutoTrajectory getInitialTrajectory() {
    return NewRedLeftNeutralZone;
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    routine
        .active()
        .onTrue(
            Commands.sequence(
                NewRedLeftNeutralZone.resetOdometry(),
                Commands.parallel(
                    NewRedLeftNeutralZone.cmd(),
                    Commands.sequence(
                        Commands.waitSeconds(0.8), intake.getDeployCommand().withTimeout(9.2)))));

    NewRedLeftNeutralZone.done()
        .onTrue(
            Commands.sequence(
                stopDrive(),
                shakeAndFeed(5.0),
                Commands.parallel(
                    NewRedLeftTransition.cmd(), intake.getDeployCommand().withTimeout(5.0))));

    NewRedLeftTransition.done().onTrue(Commands.sequence(stopDrive(), shakeAndFeed(5.0)));

    return routine;
  }
}
