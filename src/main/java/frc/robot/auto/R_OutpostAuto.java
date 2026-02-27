package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class R_OutpostAuto extends AutoMode {
  Hopper hopper;
  Feeder feeder;
  Intake intake;
  Launcher launcher;

  public R_OutpostAuto(
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
  AutoRoutine routine = super.getAutoFactory().newRoutine("R_OutpostAuto");

  // Load trajectories
  AutoTrajectory redStartToOutpost = routine.trajectory("RedStartToOutpost");

  @Override
  public String getName() {
    return "RedOutpostAuto";
  }

  @Override
  public AutoTrajectory getInitialTrajectory() {
    return redStartToOutpost;
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    routine
        .active()
        .onTrue(
            Commands.sequence(
                redStartToOutpost.resetOdometry(),
                Commands.parallel(
                    redStartToOutpost.cmd(),
                    Commands.sequence(
                        // Commands.runOnce(hopper::deploy, hopper),
                        Commands.startEnd(intake::intakeFuel, () -> {}, intake)
                            .withTimeout(10.0)))));

    redStartToOutpost
        .done()
        .onTrue(Commands.startEnd(feeder::spinForward, () -> {}, feeder).withTimeout(5.0));

    return routine;
  }
}
