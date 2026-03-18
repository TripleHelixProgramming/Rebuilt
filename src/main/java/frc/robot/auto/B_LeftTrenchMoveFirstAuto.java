package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class B_LeftTrenchMoveFirstAuto extends AutoMode {
  private final AutoRoutine routine;
  private final AutoTrajectory blueLeftNeutralZone;
  private final AutoTrajectory blueLeftTransitionToNZ;

  public B_LeftTrenchMoveFirstAuto(
      Drive drivetrain,
      Feeder feederSubsystem,
      Intake intakeSubsystem,
      Launcher launcherSubsystem) {
    super(drivetrain, feederSubsystem, intakeSubsystem);
    routine = getAutoFactory().newRoutine("B_LeftTrenchMoveFirstAuto");
    blueLeftNeutralZone = routine.trajectory("BlueLeftNinetyNeutralZone");
    blueLeftTransitionToNZ = routine.trajectory("BlueLeftTransitionToNZ");
  }

  @Override
  public String getName() {
    return "BlueLeftTrenchMoveFirstAuto";
  }

  @Override
  public AutoTrajectory getInitialTrajectory() {
    return blueLeftNeutralZone;
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    routine
        .active()
        .onTrue(
            Commands.sequence(
                blueLeftNeutralZone.resetOdometry(),
                Commands.parallel(
                    blueLeftNeutralZone.cmd(),
                    Commands.sequence(
                        Commands.waitSeconds(0.8), intake.getDeployCommand().withTimeout(9.2)))));

    blueLeftNeutralZone
        .done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(drive::stop, drive),
                shakeAndFeed(5.0),
                Commands.parallel(
                    blueLeftTransitionToNZ.cmd(), intake.getDeployCommand().withTimeout(5.0))));

    blueLeftTransitionToNZ
        .done()
        .onTrue(
            Commands.sequence(Commands.runOnce(drive::stop, drive), shakeAndFeed(5.0)));

    return routine;
  }
}
