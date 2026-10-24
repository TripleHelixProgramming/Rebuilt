package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class RedMiddleDepotShootAuto extends AutoMode {
  private final AutoRoutine routine;
  private final AutoTrajectory RedMiddleDepotShootAuto;

  public RedMiddleDepotShootAuto(
      Drive drivetrain,
      Feeder feederSubsystem,
      Intake intakeSubsystem,
      Launcher launcherSubsystem) {
    super(drivetrain, feederSubsystem, intakeSubsystem);
    routine = getAutoFactory().newRoutine("RedMiddleDepotShootAuto");
    RedMiddleDepotShootAuto = routine.trajectory("RedMiddleDepotShootAuto");
  }

  @Override
  public String getName() {
    return "RedLeftTrenchMoveFirstAuto";
  }

  @Override
  public AutoTrajectory getInitialTrajectory() {
    return RedMiddleDepotShootAuto;
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    routine
        .active()
        .onTrue(
            Commands.sequence(
                RedMiddleDepotShootAuto.resetOdometry(),
                Commands.parallel(
                    RedMiddleDepotShootAuto.cmd(),
                    Commands.sequence(
                        Commands.waitSeconds(0.8), intake.getDeployCommand().withTimeout(9.2)))));
    return routine;
  }
}
