package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class NewB_LeftTrenchMoveFirstAuto extends AutoMode {
  private final AutoRoutine routine;
  private final AutoTrajectory BlueLeftNinetyNeutralZone;
  private final AutoTrajectory NewBlueLeftTransition;

  public NewB_LeftTrenchMoveFirstAuto(
      Drive drivetrain,
      Feeder feederSubsystem,
      Intake intakeSubsystem,
      Launcher launcherSubsystem) {
    super(drivetrain, feederSubsystem, intakeSubsystem);
    routine = getAutoFactory().newRoutine("NewB_LeftTrenchMoveFirstAuto");
    BlueLeftNinetyNeutralZone = routine.trajectory("BlueLeftNinetyNeutralZone");
    NewBlueLeftTransition = routine.trajectory("NewBlueLeftTransition");
  }

  @Override
  public String getName() {
    return "NewB_LeftTrenchMoveFirstAuto";
  }

  @Override
  public AutoTrajectory getInitialTrajectory() {
    return BlueLeftNinetyNeutralZone;
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    routine
        .active()
        .onTrue(
            Commands.sequence(
                BlueLeftNinetyNeutralZone.resetOdometry(),
                Commands.parallel(
                    BlueLeftNinetyNeutralZone.cmd(),
                    Commands.sequence(
                        Commands.waitSeconds(0.8), intake.getDeployCommand().withTimeout(9.2)))));

    BlueLeftNinetyNeutralZone.done()
        .onTrue(
            Commands.sequence(
                stopDrive(),
                shakeAndFeed(5.0),
                Commands.parallel(
                    NewBlueLeftTransition.cmd(), intake.getDeployCommand().withTimeout(5.0))));

    NewBlueLeftTransition.done().onTrue(Commands.sequence(stopDrive(), shakeAndFeed(5.0)));

    return routine;
  }
}
