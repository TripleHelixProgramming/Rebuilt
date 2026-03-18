package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class R_RightTrenchAuto extends AutoMode {
  Drive drive;
  // Hopper hopper;
  Feeder feeder;
  Intake intake;
  Launcher launcher;

  public R_RightTrenchAuto(
      Drive drivetrain,
      // Hopper hopperSubsystem,
      Feeder feederSubsystem,
      Intake intakeSubsystem,
      Launcher launcherSubsystem) {
    super(drivetrain);
    drive = drivetrain;
    // hopper = hopperSubsystem;
    feeder = feederSubsystem;
    intake = intakeSubsystem;
    launcher = launcherSubsystem;
  }

  // Define routine
  AutoRoutine routine = super.getAutoFactory().newRoutine("R_RightTrenchAuto");

  // Load trajectories
  AutoTrajectory redRightNeutralZone = routine.trajectory("RedRightNeutralZone");
  AutoTrajectory redRightTransitionToNZ = routine.trajectory("RedRightTransitionToNZ");

  @Override
  public String getName() {
    return "RedRightTrenchAuto";
  }

  @Override
  public AutoTrajectory getInitialTrajectory() {
    return redRightNeutralZone;
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    routine
        .active()
        .onTrue(
            Commands.sequence(
                redRightNeutralZone.resetOdometry(),
                DriveCommands.getChassisAimingCommand(drive, launcher::getTurretDesaturationDelta)
                    .withTimeout(1.5),
                Commands.startEnd(feeder::spinForward, () -> {}, feeder).withTimeout(3.0),
                Commands.runOnce(feeder::stop, feeder),
                Commands.parallel(
                    redRightNeutralZone.cmd(), intake.getDeployCommand().withTimeout(10.0))));

    redRightNeutralZone
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
                    redRightTransitionToNZ.cmd(), intake.getDeployCommand().withTimeout(5.0))));

    redRightTransitionToNZ
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
