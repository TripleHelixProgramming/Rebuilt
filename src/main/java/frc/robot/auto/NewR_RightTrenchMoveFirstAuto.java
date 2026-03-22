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
  private final AutoTrajectory NewRedRightNeurealZone;
  private final AutoTrajectory redRightTransitionToNZ;

  public NewR_RightTrenchMoveFirstAuto(
      Drive drivetrain,
      Feeder feederSubsystem,
      Intake intakeSubsystem,
      Launcher launcherSubsystem) {
    super(drivetrain, feederSubsystem, intakeSubsystem);
    routine = getAutoFactory().newRoutine("NewR_RightTrenchMoveFirstAuto");
    NewRedRightNeurealZone = routine.trajectory("NewRedRightNeurealZone");
    redRightTransitionToNZ = routine.trajectory("RedRightTransitionToNZ");
  }

  @Override
  public String getName() {
    return "NewRedRightNeurealZone";
  }

  @Override
  public AutoTrajectory getInitialTrajectory() {
    return NewRedRightNeurealZone;
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    routine
        .active()
        .onTrue(
            Commands.sequence(
                NewRedRightNeurealZone.resetOdometry(),
                Commands.parallel(
                    NewRedRightNeurealZone.cmd(),
                    Commands.sequence(
                        Commands.waitSeconds(0.8), intake.getDeployCommand().withTimeout(9.2)))));

    NewRedRightNeurealZone.done()
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
