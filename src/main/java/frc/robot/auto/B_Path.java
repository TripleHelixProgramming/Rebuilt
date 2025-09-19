package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class B_Path extends AutoMode {

  public B_Path(Drive drivetrain) {
    super(drivetrain);
  }

  // Define routine
  AutoRoutine routine = super.getAutoFactory().newRoutine("bluesquared");

  // Load the routine's trajectories
  AutoTrajectory trajectory = routine.trajectory("bluesquare");

  @Override
  public String getName() {
    return "bluesquared";
  }

  @Override
  public AutoTrajectory getInitialTrajectory() {
    return trajectory;
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    // spotless:off
    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(
        Commands.sequence(
            trajectory.resetOdometry(), 
            trajectory.cmd()));
    // spotless:on

    return routine;
  }
}
