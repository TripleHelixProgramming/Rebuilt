package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class B_LeftTrenchMoveFirstAuto extends AutoMode {
  Drive drive;
  // Hopper hopper;
  Feeder feeder;
  Intake intake;
  Launcher launcher;

  public B_LeftTrenchMoveFirstAuto(
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
  AutoRoutine routine = super.getAutoFactory().newRoutine("B_LeftTrenchMoveFirstAuto");

  // Load trajectories
  AutoTrajectory blueLeftNeutralZone = routine.trajectory("BlueLeftNeutralZone");
  AutoTrajectory blueLeftTransitionToNZ = routine.trajectory("BlueLeftTransitionToNZ");

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
                    blueLeftNeutralZone.cmd(), intake.getDeployCommand().withTimeout(10.0))));

    blueLeftNeutralZone
        .done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(drive::stop, drive),
                Commands.startEnd(launcher::desaturateTurret, () -> {}, launcher).withTimeout(0.5),
                Commands.startEnd(feeder::spinForward, () -> {}, feeder).withTimeout(5.0),
                Commands.parallel(
                    blueLeftTransitionToNZ.cmd(), intake.getDeployCommand().withTimeout(5.0))));

    blueLeftTransitionToNZ
        .done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(drive::stop, drive),
                Commands.startEnd(feeder::spinForward, () -> {}, feeder).withTimeout(5.0)));

    return routine;
  }
}
