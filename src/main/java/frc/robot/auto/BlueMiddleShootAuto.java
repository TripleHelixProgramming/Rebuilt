package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class BlueMiddleShootAuto extends AutoMode {
  private final Launcher launcher;
  private final AutoRoutine routine;
  private final AutoTrajectory blueMiddleShoot;


  public BlueMiddleShootAuto(
      Drive drivetrain,
      Feeder feederSubsystem,
      Intake intakeSubsystem,
      Launcher launcherSubsystem) {
    super(drivetrain, feederSubsystem, intakeSubsystem);
    launcher = launcherSubsystem;
    routine = getAutoFactory().newRoutine("BlueMiddleShootAuto");
    blueMiddleShoot = routine.trajectory("BlueMiddleShootAuto");
  }

  @Override
  public String getName() {
    return "BlueMiddleShootAuto";
  }

  @Override
  public AutoTrajectory getInitialTrajectory() {
    return blueMiddleShoot;
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    routine
        .active()
        .onTrue(
            Commands.sequence(
                blueMiddleShoot.resetOdometry(),
                DriveCommands.getChassisAimingCommand(drive, launcher::getTurretDesaturationDelta)
                    .withTimeout(1.5),
                feeder.getSpinForwardCommand().withTimeout(3.0),
                Commands.parallel(
                    blueMiddleShoot.cmd())));

    blueMiddleShoot
        .done()
        .onTrue(
            Commands.sequence(
                stopDrive(),
                shakeAndFeed(5.0),
                Commands.parallel(
                    blueMiddleShoot.cmd(), intake.getDeployCommand().withTimeout(5.0))));

    blueMiddleShoot.done().onTrue(Commands.sequence(stopDrive(), shakeAndFeed(5.0)));

    return routine;
  }
}
