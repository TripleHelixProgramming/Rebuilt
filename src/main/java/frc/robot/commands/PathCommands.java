package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

public class PathCommands {

  private PathCommands() {}

  /**
   * Drive to the target pose.
   *
   * @param drive The drivetrain subsystem
   * @param targetPose The target pose in absolute coordinates
   * @return The path-following command
   */
  public static Command goToTargetPose(Drive drive, Pose2d targetPose) {
    var supplier =
        new Supplier<Command>() {
          @Override
          public Command get() {
            List<Waypoint> waypoints =
                PathPlannerPath.waypointsFromPoses(drive.getPose(), targetPose);

            return AutoBuilder.followPath(createPath(waypoints, targetPose.getRotation()));
          }
        };
    return new DeferredCommand(supplier, Set.of(drive));
  }

  /**
   * Drive to the nearest point located the lead distance away from the target point, then drive
   * forward by the lead distance until arriving at the target point.
   *
   * @param drive The drivetrain subsystem
   * @param targetPoint The target point in absolute coordinates
   * @param leadDistance The offset distance where the robot should line up facing the target point
   *     before moving forward
   * @return The path-following command
   */
  public static Command dockToTargetPoint(
      Drive drive, Translation2d targetPoint, Distance leadDistance) {
    var supplier =
        new Supplier<Command>() {
          @Override
          public Command get() {
            var initialPose = drive.getPose();
            var heading = targetPoint.minus(initialPose.getTranslation()).getAngle();
            var finalPose = new Pose2d(targetPoint, heading);
            List<Waypoint> waypoints =
                PathPlannerPath.waypointsFromPoses(
                    initialPose,
                    finalPose.plus(
                        new Transform2d(leadDistance.unaryMinus(), Meters.of(0), Rotation2d.kZero)),
                    finalPose);

            return AutoBuilder.followPath(createPath(waypoints, heading));
          }
        };
    return new DeferredCommand(supplier, Set.of(drive));
  }

  /**
   * Drive to a pose that is offset from the target pose by the lead distance, then drive forward by
   * the lead distance until arriving at the target pose.
   *
   * @param drive The drivetrain subsystem
   * @param targetPose The target pose in absolute coordinates
   * @param leadDistance The offset distance where the robot should line up behind the target pose
   *     before moving forward
   * @return The path-following command
   */
  public static Command dockToTargetPose(Drive drive, Pose2d targetPose, Distance leadDistance) {
    var supplier =
        new Supplier<Command>() {
          @Override
          public Command get() {
            List<Waypoint> waypoints =
                PathPlannerPath.waypointsFromPoses(
                    drive.getPose(),
                    targetPose.plus(
                        new Transform2d(leadDistance.unaryMinus(), Meters.of(0), Rotation2d.kZero)),
                    targetPose);

            return AutoBuilder.followPath(createPath(waypoints, targetPose.getRotation()));
          }
        };
    return new DeferredCommand(supplier, Set.of(drive));
  }

  /**
   * Drive forward by the lead distance.
   *
   * @param drive The drivetrain subsystem
   * @param leadDistance The distance to move forward
   * @return The path-following command
   */
  public static Command advanceForward(Drive drive, Distance leadDistance) {
    var supplier =
        new Supplier<Command>() {
          @Override
          public Command get() {
            var initialPose = drive.getPose();
            var waypoints =
                PathPlannerPath.waypointsFromPoses(
                    initialPose,
                    initialPose.plus(
                        new Transform2d(leadDistance, Meters.of(0), Rotation2d.kZero)));

            return AutoBuilder.followPath(createPath(waypoints, initialPose.getRotation()));
          }
        };
    return new DeferredCommand(supplier, Set.of(drive));
  }

  public static Command getPathFromFileCommand() {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

      // Create a path following command using AutoBuilder. This will also trigger event markers.
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  /**
   * @param waypoints A list of waypoints that the path will connect
   * @param endRotation The orientation of the robot at the end of the path
   * @return Path that follows the waypoints and ends with the correct orientation
   */
  private static PathPlannerPath createPath(List<Waypoint> waypoints, Rotation2d endRotation) {
    var path =
        new PathPlannerPath(
            waypoints,
            pathFollowingConstraints,
            // The ideal starting state, this is only relevant for pre-planned paths,
            // so can be null for on-the-fly paths.
            null,
            new GoalEndState(0.0, endRotation));

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;

    return path;
  }
}
