package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class B_LeftTrenchAuto extends AutoMode {
  private final Launcher launcher;
  private final AutoRoutine routine;
  private final AutoTrajectory blueLeftNeutralZone;
  private final AutoTrajectory blueLeftTransitionToNZ;

  public B_LeftTrenchAuto(
      Drive drivetrain,
      Feeder feederSubsystem,
      Intake intakeSubsystem,
      Launcher launcherSubsystem) {
    super(drivetrain, feederSubsystem, intakeSubsystem);
    launcher = launcherSubsystem;
    routine = getAutoFactory().newRoutine("B_LeftTrenchAuto");
    blueLeftNeutralZone = routine.trajectory("BlueLeftNeutralZone");
    blueLeftTransitionToNZ = routine.trajectory("BlueLeftTransitionToNZ");
  }

  @Override
  public String getName() {
    return "BlueLeftTrenchAuto";
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
                DriveCommands.getChassisAimingCommand(drive, launcher::getTurretDesaturationDelta)
                    .withTimeout(1.5),
                feeder.getSpinForwardCommand().withTimeout(3.0),
                Commands.parallel(
                    blueLeftNeutralZone.cmd(), intake.getDeployCommand().withTimeout(10.0))));

    blueLeftNeutralZone
        .done()
        .onTrue(
            Commands.sequence(
                stopDrive(),
                shakeAndFeed(5.0),
                Commands.parallel(
                    blueLeftTransitionToNZ.cmd(), intake.getDeployCommand().withTimeout(5.0))));

    blueLeftTransitionToNZ.done().onTrue(Commands.sequence(stopDrive(), shakeAndFeed(5.0)));

    return routine;
  }
}
