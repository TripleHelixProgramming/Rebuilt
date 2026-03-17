package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class R_LeftTrenchAuto extends AutoMode {
  Drive drive;
  // Hopper hopper;
  Feeder feeder;
  Intake intake;
  Launcher launcher;

  public R_LeftTrenchAuto(
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
  AutoRoutine routine = super.getAutoFactory().newRoutine("R_LeftTrenchAuto");

  // Load trajectories
  AutoTrajectory redLeftNeutralZone = routine.trajectory("RedLeftNeutralZone");
  AutoTrajectory redLeftTransitionToNZ = routine.trajectory("RedLeftTransitionToNZ");

  @Override
  public String getName() {
    return "RedLeftTrenchAuto";
  }

  @Override
  public AutoTrajectory getInitialTrajectory() {
    return redLeftNeutralZone;
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    routine
        .active()
        .onTrue(
            Commands.sequence(
                redLeftNeutralZone.resetOdometry(),
                DriveCommands.getChassisAimingCommand(drive, launcher::getTurretDesaturationDelta)
                    .withTimeout(1.5),
                Commands.startEnd(feeder::spinForward, () -> {}, feeder).withTimeout(3.0),
                Commands.runOnce(feeder::stop, feeder),
                Commands.parallel(
                    redLeftNeutralZone.cmd(), intake.getDeployCommand().withTimeout(10.0))));

    redLeftNeutralZone
        .done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(drive::stop, drive),
                Commands.startEnd(feeder::spinForward, () -> {}, feeder).withTimeout(5.0),
                Commands.runOnce(feeder::stop, feeder),
                Commands.parallel(
                    redLeftTransitionToNZ.cmd(),
                    Commands.sequence(
                        Commands.waitSeconds(1.0), intake.getDeployCommand().withTimeout(4.0)))));

    redLeftTransitionToNZ
        .done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(drive::stop),
                Commands.startEnd(feeder::spinForward, () -> {}, feeder).withTimeout(5.0),
                Commands.runOnce(feeder::stop, feeder)));

    return routine;
  }
}
