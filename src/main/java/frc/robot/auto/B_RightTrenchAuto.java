package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class B_RightTrenchAuto extends AutoMode {
  private final Launcher launcher;
  private final AutoRoutine routine;
  private final AutoTrajectory blueRightNeutralZone;
  private final AutoTrajectory blueRightTransitionToNZ;

  public B_RightTrenchAuto(
      Drive drivetrain,
      Feeder feederSubsystem,
      Intake intakeSubsystem,
      Launcher launcherSubsystem) {
    super(drivetrain, feederSubsystem, intakeSubsystem);
    launcher = launcherSubsystem;
    routine = getAutoFactory().newRoutine("B_RightTrenchAuto");
    blueRightNeutralZone = routine.trajectory("BlueRightNeutralZone");
    blueRightTransitionToNZ = routine.trajectory("BlueRightTransitionToNZ");
  }

  @Override
  public String getName() {
    return "BlueRightTrenchAuto";
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
                DriveCommands.getChassisAimingCommand(drive, launcher::getTurretDesaturationDelta)
                    .withTimeout(1.5),
                feeder.getSpinForwardCommand().withTimeout(3.0),
                Commands.parallel(
                    blueRightNeutralZone.cmd(), intake.getDeployCommand().withTimeout(10.0))));

    blueRightNeutralZone
        .done()
        .onTrue(
            Commands.sequence(
                stopDrive(),
                shakeAndFeed(5.0),
                Commands.parallel(
                    blueRightTransitionToNZ.cmd(), intake.getDeployCommand().withTimeout(5.0))));

    blueRightTransitionToNZ.done().onTrue(Commands.sequence(stopDrive(), shakeAndFeed(5.0)));

    return routine;
  }
}
