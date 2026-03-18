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
  private final Launcher launcher;
  private final AutoRoutine routine;
  private final AutoTrajectory redLeftNeutralZone;
  private final AutoTrajectory redLeftTransitionToNZ;

  public R_LeftTrenchAuto(
      Drive drivetrain,
      Feeder feederSubsystem,
      Intake intakeSubsystem,
      Launcher launcherSubsystem) {
    super(drivetrain, feederSubsystem, intakeSubsystem);
    launcher = launcherSubsystem;
    routine = getAutoFactory().newRoutine("R_LeftTrenchAuto");
    redLeftNeutralZone = routine.trajectory("RedLeftNeutralZone");
    redLeftTransitionToNZ = routine.trajectory("RedLeftTransitionToNZ");
  }

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
                feeder.getSpinForwardCommand().withTimeout(3.0),
                Commands.parallel(
                    redLeftNeutralZone.cmd(), intake.getDeployCommand().withTimeout(10.0))));

    redLeftNeutralZone
        .done()
        .onTrue(
            Commands.sequence(
                stopDrive(),
                shakeAndFeed(5.0),
                Commands.parallel(
                    redLeftTransitionToNZ.cmd(),
                    Commands.sequence(
                        Commands.waitSeconds(1.0), intake.getDeployCommand().withTimeout(4.0)))));

    redLeftTransitionToNZ.done().onTrue(Commands.sequence(stopDrive(), shakeAndFeed(5.0)));

    return routine;
  }
}
