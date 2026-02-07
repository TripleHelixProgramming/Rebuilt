package frc.game;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.game.Field.Region;
import frc.robot.Robot;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class GameState {

  enum GamePhase {
    None("0:00 - 0:00"),
    Autonomous("0:20 - 0:00"),
    Transition("2:20 - 2:10"),
    Shift1("2:10 - 1:45"),
    Shift2("1:45 - 1:20"),
    Shift3("1:20 - 0:55"),
    Shift4("0:55 - 0:30"),
    EndGame("0:30 - 0:00");

    public static final List<GamePhase> TELEOP =
        List.of(Transition, Shift1, Shift2, Shift3, Shift4, EndGame);

    final double countDownFrom;
    final double countDownUntil;

    private GamePhase(String timer) {
      var times = timer.split("-");
      this.countDownFrom = parseSeconds(times[0]);
      this.countDownUntil = parseSeconds(times[1]);
    }

    private static int parseSeconds(String time) {
      var parts = time.trim().split(":");
      return Integer.parseInt(parts[0]) * 60 + Integer.parseInt(parts[1]);
    }
  }

  private static Alliance autoWinner;
  private static Alliance myAlliance;

  public static GamePhase getCurrentPhase() {
    if (!DriverStation.isDSAttached() && !DriverStation.isFMSAttached()) {
      return GamePhase.None;
    }
    if (DriverStation.isAutonomous()) {
      return GamePhase.Autonomous;
    }
    // Must be in match and teleop
    var t = getMatchTime();
    for (var gamePhase : GamePhase.TELEOP) {
      if (t <= gamePhase.countDownFrom && t > gamePhase.countDownUntil) {
        return gamePhase;
      }
    }
    return GamePhase.None;
  }

  public static Optional<Alliance> getMyAlliance() {
    if (myAlliance == null) {
      myAlliance = DriverStation.getAlliance().orElse(null);
    }
    return Optional.ofNullable(myAlliance);
  }

  public static Optional<Alliance> getAutoWinner() {
    if (autoWinner == null) {
      var gameData = DriverStation.getGameSpecificMessage();
      if (gameData.length() > 0) {
        autoWinner =
            switch (gameData.charAt(0)) {
              case 'B' -> Alliance.Blue;
              case 'R' -> Alliance.Red;
              default -> null;
            };
      }
    }
    return Optional.ofNullable(autoWinner);
  }

  public static boolean isMyHubActive() {
    switch (getCurrentPhase()) {
      case None:
      case Autonomous:
      case Transition:
      case EndGame:
        return true;
      case Shift1:
      case Shift3:
        return autoWinner != null && myAlliance != null && autoWinner != myAlliance;
      case Shift2:
      case Shift4:
        return autoWinner != null && myAlliance != null && autoWinner == myAlliance;
      default:
        return false;
    }
  }

  public static double getMatchTime() {
    return DriverStation.getMatchTime();
  }

  public static Optional<Alliance> getAlliance() {
    return DriverStation.getAlliance();
  }

  public static void logValues() {
    getMyAlliance();
    getAutoWinner();
    Logger.recordOutput("GameState/IsDSAttached", DriverStation.isDSAttached());
    Logger.recordOutput("GameState/IsFMSAttached", DriverStation.isFMSAttached());
    Logger.recordOutput("GameState/MatchType", DriverStation.getMatchType());
    Logger.recordOutput("GameState/IsAutonomus", DriverStation.isAutonomous());
    Logger.recordOutput("GameState/MatchTime", DriverStation.getMatchTime());
    Logger.recordOutput("GameState/AutoWinner", autoWinner);
    Logger.recordOutput("GameState/Alliance", myAlliance);
    Logger.recordOutput("GameState/GameData", DriverStation.getGameSpecificMessage());
    Logger.recordOutput("GameState/CurrentPhase", getCurrentPhase());
    Logger.recordOutput("GameState/IsMyHubActive", isMyHubActive());
  }

  public static Pose3d getTarget(Pose2d robotPose) {
    if (Robot.getAlliance() == Alliance.Red) {
      if (Region.RedZone.contains(robotPose)) {
        return Field.redHubCenter;
      }
      if (robotPose.getMeasureY().lt(Field.centerField_y_pos)) {
        return Field.redLeftTarget;
      }
      return Field.redRightTarget;
    }
    if (Region.BlueZone.contains(robotPose)) {
      return Field.blueHubCenter;
    }
    if (robotPose.getMeasureY().lt(Field.centerField_y_pos)) {
      return Field.blueLeftTarget;
    }
    return Field.blueRightTarget;
  }
}
