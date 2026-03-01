package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class B_RightTrenchAuto extends AutoMode {
  Hopper hopper;
  Feeder feeder;
  Intake intake;
  Launcher launcher;

  public B_RightTrenchAuto(
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
  AutoRoutine routine = super.getAutoFactory().newRoutine("B_RightTrenchAuto");

  // Load trajectories
  AutoTrajectory blueRightNeutralZone = routine.trajectory("BlueRightNeutralZone");
  AutoTrajectory blueRightTransitionToNZ = routine.trajectory("BlueRightTransitionToNZ");

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
                Commands.parallel(
                    blueRightNeutralZone.cmd(),
                    Commands.sequence(
                        // Commands.runOnce(hopper::deploy, hopper),
                        intake.getDeployCommand().withTimeout(10.0)))));

    blueRightNeutralZone
        .done()
        .onTrue(
            Commands.sequence(
                Commands.startEnd(feeder::spinForward, () -> {}, feeder).withTimeout(5.0),
                Commands.parallel(
                    blueRightTransitionToNZ.cmd(), intake.getDeployCommand().withTimeout(8.0))));

    blueRightTransitionToNZ
        .done()
        .onTrue(Commands.startEnd(feeder::spinForward, () -> {}, feeder).withTimeout(5.0));

    return routine;
  }
}
