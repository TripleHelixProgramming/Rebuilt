package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class B_LeftTrenchAuto extends AutoMode {
  Hopper hopper;
  Feeder feeder;
  Intake intake;
  Launcher launcher;

  public B_LeftTrenchAuto(
      Drive drivetrain,
      Hopper hopperSubsystem,
      Feeder feederSubsystem,
      Intake intakeSubsystem,
      Launcher launcherSubsystem) {
    super(drivetrain);
    hopper = hopperSubsystem;
    feeder = feederSubsystem;
    intake = intakeSubsystem;
    launcher = launcherSubsystem;
  }

  // Define routine
  AutoRoutine routine = super.getAutoFactory().newRoutine("B_LeftTrenchAuto");

  // Load trajectories
  AutoTrajectory blueLeftNeutralZone = routine.trajectory("BlueLeftNeutralZone");
  AutoTrajectory blueLeftTransitionToNZ = routine.trajectory("BlueLeftTransitionToNZ");

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
                Commands.parallel(
                    blueLeftNeutralZone.cmd(),
                    Commands.sequence(
                        // Commands.runOnce(hopper::deploy, hopper),
                        intake.getDeployCommand().withTimeout(10.0)))));

    blueLeftNeutralZone
        .done()
        .onTrue(
            Commands.sequence(
                Commands.startEnd(feeder::spinForward, () -> {}, feeder).withTimeout(5.0),
                Commands.parallel(
                    blueLeftTransitionToNZ.cmd(), intake.getDeployCommand().withTimeout(8.0))));

    blueLeftTransitionToNZ
        .done()
        .onTrue(Commands.startEnd(feeder::spinForward, () -> {}, feeder).withTimeout(5.0));

    return routine;
  }
}
