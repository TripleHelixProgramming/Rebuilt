package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class TraversingTheBump extends AutoMode {
  public TraversingTheBump(Drive drivetrain) {
    super(drivetrain);
  }

  AutoRoutine routine = super.getAutoFactory().newRoutine("traversingTheBump");

  AutoTrajectory trajectory = routine.trajectory("traversingTheBump");

  @Override
  public String getName() {
    return "TraversingTheBump";
  }

  @Override
  public AutoTrajectory getInitialTrajectory() {
    return trajectory;
  }

  @Override
  public AutoRoutine getAutoRoutine() {
    routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));

    return routine;
  }
}
