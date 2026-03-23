package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class NewB_RightTrenchMoveFirstAuto extends AutoMode {
  private final AutoRoutine routine;
  private final AutoTrajectory NewBlueRightNeutralZone;
  private final AutoTrajectory NewBlueRightTransition;

  public NewB_RightTrenchMoveFirstAuto(
      Drive drivetrain,
      Feeder feederSubsystem,
      Intake intakeSubsystem,
      Launcher launcherSubsystem) {
    super(drivetrain, feederSubsystem, intakeSubsystem);
    routine = getAutoFactory().newRoutine("NewB_RightTrenchMoveFirstAuto");
    NewBlueRightNeutralZone = routine.trajectory("NewBlueRightNeutralZone");
    NewBlueRightTransition = routine.trajectory("NewBlueRightTransition");
  }

  @Override
  public String getName() {
    return "NewB_RightTrenchMoveFirstAuto";
  }

  @Override
  public AutoTrajectory getInitialTrajectory() {
    return NewBlueRightNeutralZone;
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    routine
        .active()
        .onTrue(
            Commands.sequence(
                NewBlueRightNeutralZone.resetOdometry(),
                Commands.parallel(
                    NewBlueRightNeutralZone.cmd(),
                    Commands.sequence(
                        Commands.waitSeconds(0.8), intake.getDeployCommand().withTimeout(9.2)))));

    NewBlueRightNeutralZone.done()
        .onTrue(
            Commands.sequence(
                stopDrive(),
                shakeAndFeed(5.0),
                Commands.parallel(
                    NewBlueRightTransition.cmd(), intake.getDeployCommand().withTimeout(5.0))));

    NewBlueRightTransition.done().onTrue(Commands.sequence(stopDrive(), shakeAndFeed(5.0)));

    return routine;
  }
}
