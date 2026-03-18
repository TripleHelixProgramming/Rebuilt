package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class B_RightTrenchMoveFirstAuto extends AutoMode {
  Drive drive;
  Feeder feeder;
  Intake intake;
  Launcher launcher;

  public B_RightTrenchMoveFirstAuto(
      Drive drivetrain,
      Feeder feederSubsystem,
      Intake intakeSubsystem,
      Launcher launcherSubsystem) {
    super(drivetrain);
    drive = drivetrain;
    feeder = feederSubsystem;
    intake = intakeSubsystem;
    launcher = launcherSubsystem;
  }

  // Define routine
  AutoRoutine routine = super.getAutoFactory().newRoutine("B_RightTrenchMoveFirstAuto");

  // Load trajectories
  AutoTrajectory blueRightNeutralZone = routine.trajectory("BlueRightNinetyNeutralZone");
  AutoTrajectory blueRightTransitionToNZ = routine.trajectory("BlueRightTransitionToNZ");

  @Override
  public String getName() {
    return "BlueRightTrenchMoveFirstAuto";
  }

  @Override
  public AutoTrajectory getInitialTrajectory() {
    return blueRightNeutralZone;
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    routine
        .active()
        .onTrue(
            Commands.sequence(
                blueRightNeutralZone.resetOdometry(),
                Commands.parallel(
                    blueRightNeutralZone.cmd(),
                    Commands.sequence(
                        Commands.waitSeconds(0.8), intake.getDeployCommand().withTimeout(9.2)))));

    blueRightNeutralZone
        .done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(drive::stop, drive),
                Commands.parallel(
                        Commands.startEnd(feeder::spinForward, () -> {}, feeder),
                        intake.getShakeIntakeCommand())
                    .withTimeout(5.0),
                Commands.runOnce(feeder::stop, feeder),
                Commands.parallel(
                    blueRightTransitionToNZ.cmd(), intake.getDeployCommand().withTimeout(5.0))));

    blueRightTransitionToNZ
        .done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(drive::stop, drive),
                Commands.parallel(
                        Commands.startEnd(feeder::spinForward, () -> {}, feeder),
                        intake.getShakeIntakeCommand())
                    .withTimeout(5.0),
                Commands.runOnce(feeder::stop, feeder)));

    return routine;
  }
}
