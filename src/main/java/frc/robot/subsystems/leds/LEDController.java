package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.game.GameState;
import frc.robot.Robot;

/**
 * A subsystem to control the LEDs on the robot.
 *
 * <p>Physical strips are defined in {@link LEDStrip}, logical views in {@link LEDSeries}, and
 * custom patterns in {@link LEDCustomPattern}.
 *
 * <p>Apply patterns directly to series: {@code LEDSeries.ALL.applyPattern(pattern)}
 *
 * <p>This is a singleton class. Access the instance via {@link #getInstance()}.
 */
public class LEDController extends SubsystemBase {

  private static LEDController instance;

  public static synchronized LEDController getInstance() {
    if (instance == null) {
      instance = new LEDController();
    }
    return instance;
  }

  private LEDController() {
    LEDStrip.startAll();
  }

  @Override
  public void periodic() {
    LEDStrip.updateAll();
  }

  // ==================== CONTEXT-AWARE DISPLAYS ====================

  /**
   * Displays pose-seek feedback on the LEDs. Shows how to move the robot to reach a target pose.
   *
   * <ul>
   *   <li><b>Heading (MIDDLE):</b> White = correct, Cyan = rotate CW, Magenta = rotate CCW
   *   <li><b>X (TOP):</b> White = correct, Green = forward, Red = backward
   *   <li><b>Y (BOTTOM):</b> White = correct, Green on one side = move that direction
   * </ul>
   */
  private void displayPoseSeek(Pose2d currentPose, Pose2d targetPose) {
    // var delta = targetPose.minus(currentPose);

    // // Heading feedback on MIDDLE
    // var theta = MathUtil.inputModulus(delta.getRotation().getDegrees(), -180, 180);
    // Color headingColor =
    //     Math.abs(theta) < LEDConstants.kPoseSeekHeadingToleranceDegrees
    //         ? Color.kWhite
    //         : theta > 0 ? Color.kMagenta : Color.kCyan;
    // LEDSeries.MIDDLE.applyPattern(LEDPattern.solid(headingColor));

    // // X feedback on TOP
    // var x = delta.getTranslation().getMeasureX().in(Centimeters);
    // Color xColor =
    //     Math.abs(x) < LEDConstants.kPoseSeekXToleranceCm
    //         ? Color.kWhite
    //         : x > 0 ? Color.kGreen : Color.kRed;
    // LEDSeries.TOP.applyPattern(LEDPattern.solid(xColor));

    // // Y feedback on BOTTOM
    // var y = delta.getTranslation().getMeasureY().in(Centimeters);
    // if (Math.abs(y) < LEDConstants.kPoseSeekYToleranceCm) {
    //   LEDSeries.BOTTOM.applyPattern(LEDPattern.solid(Color.kWhite));
    // } else {
    //   LEDSeries yView = y > 0 ? LEDSeries.LEFT_BOTTOM : LEDSeries.RIGHT_BOTTOM;
    //   LEDSeries otherView = y > 0 ? LEDSeries.RIGHT_BOTTOM : LEDSeries.LEFT_BOTTOM;
    //   yView.applyPattern(LEDPattern.solid(Color.kGreen));
    //   otherView.applyPattern(LEDPattern.solid(Color.kBlack));
    // }
  }

  public static LEDPattern solidBlackPattern = LEDPattern.solid(Color.kBlack);
  public static LEDPattern solidYellowPattern = LEDPattern.solid(Color.kYellow);

  public static LEDPattern autoSelectionPattern = LEDCustomPattern.countingBlocks(
    () -> Robot.autoSelector.get().get().getOptionNumber(),
    () -> Robot.autoSelector.get().get().getAllianceColor(),
    LEDConstants.kLEDsPerBlock,
    LEDConstants.kLEDsBetweenBlocks);

  public static LEDPattern hubCountdownPattern = LEDCustomPattern.progressBar(
    // Percent full
    () -> {
      var t = GameState.getMatchTime();
      var phase = GameState.getCurrentPhase();
      return phase.remainingAt(t) / phase.duration();
    },
    // Fill color
    () -> {
      if (GameState.isMyHubActive() && GameState.getMyAlliance() == Alliance.Red) {
        return Color.kRed;
      }
      return Color.kBlue;
    },
    // Background color
    Color.kBlack);


  public void displayAutoSelection() {
    Robot.autoSelector.get().ifPresentOrElse(
        autoOption -> LEDSeries.ALL.applyPattern(autoSelectionPattern),
        () -> LEDSeries.ALL.applyPattern(solidYellowPattern.blink(Seconds.of(0.5))));

    // Display yellow at end pixel if alliance disagreement
    DriverStation.getAlliance().ifPresent(alliance -> {
      if (alliance != Robot.allianceSelector.getAllianceColor()) {
        LEDSeries.ALL.setLED(LEDSeries.ALL.getLength()-1, Color.kYellow);
      }
    });
  }

  public void displayHubCountdown() {
    LEDSeries.ALL.applyPattern(hubCountdownPattern);
  }

  public void clear() {
      clear(LEDSeries.ALL);
  }

  public void clear(LEDSeries series, LEDSeries ... more) {
    series.applyPattern(solidBlackPattern);
    for (var another : more) {
      another.applyPattern(solidBlackPattern);
    }
  }
}
