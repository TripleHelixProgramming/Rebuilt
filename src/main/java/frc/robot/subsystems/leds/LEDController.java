package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.game.GameState;
import frc.robot.Robot;
import java.util.function.Supplier;

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

  /**
   * Returns the singleton instance of the LED controller, creating it if necessary.
   *
   * @return the LED controller instance
   */
  public static synchronized LEDController getInstance() {
    if (instance == null) {
      instance = new LEDController();
    }
    return instance;
  }

  private LEDController() {
    LEDStrip.startAll();
  }

  /** Pushes LED buffer data to all physical strips each cycle. */
  @Override
  public void periodic() {
    LEDStrip.updateAll();
  }

  // ==================== CONTEXT-AWARE DISPLAYS ====================

  /**
   * Displays pose-seek feedback on a horizontal LED strip. Shows how to move the robot to reach a
   * target pose.
   *
   * <p>Layout: [Y_LEFT] [ROT_LEFT] [X_CENTER] [ROT_RIGHT] [Y_RIGHT]
   *
   * <p>Indices: [0-1] [2-3] [4-7] [8-9] [10-11]
   *
   * <ul>
   *   <li><b>Y (ends):</b> Green on side to move toward, red on opposite side. White if correct.
   *   <li><b>Rotation (inner):</b> Cyan = rotate CW, Magenta = rotate CCW. White if correct.
   *   <li><b>X (center):</b> Green = forward, Red = backward. White if correct.
   * </ul>
   *
   * @param currentPose the robot's current pose
   * @param targetPose the target pose to reach
   */
  public void displayPoseSeek(Pose2d currentPose, Pose2d targetPose) {
    var delta = targetPose.minus(currentPose);

    // Transform field-relative delta to robot-relative coordinates
    var robotRelativeDelta =
        delta.getTranslation().rotateBy(currentPose.getRotation().unaryMinus());

    // X feedback on center LEDs (robot-relative: positive = forward)
    var x = robotRelativeDelta.getMeasureX().in(Centimeters);
     if (Math.abs(x) < LEDConstants.kPoseSeekHeadingToleranceDegrees) {
      LEDSeries.POSE_X.applyPattern(LEDPattern.solid(Color.kWhite));
    } else if (x > 0) {
      // Need to move forward
      LEDSeries.POSE_X.applyPattern(LEDPattern.solid(Color.kRed));
    } else {
      // Need to move backward
      LEDSeries.POSE_X.applyPattern(LEDPattern.solid(Color.kGreen));
    }

    // Heading feedback on rotation LEDs (angular error is frame-independent)
    var theta = MathUtil.inputModulus(delta.getRotation().getDegrees(), -180, 180);
     if (Math.abs(theta) < LEDConstants.kPoseSeekHeadingToleranceDegrees) {
      LEDSeries.POSE_ROTATION.applyPattern(LEDPattern.solid(Color.kWhite));
    } else if (theta > 0) {
      // Need to rotate ccw
      LEDSeries.POSE_ROTATION_X.applyPattern(LEDPattern.solid(Color.kRed));
      LEDSeries.POSE_ROTATION_Y.applyPattern(LEDPattern.solid(Color.kGreen));
    } else {
      // Need to rotate cw
      LEDSeries.POSE_ROTATION_X.applyPattern(LEDPattern.solid(Color.kGreen));
      LEDSeries.POSE_ROTATION_Y.applyPattern(LEDPattern.solid(Color.kRed));
    }

    // Y feedback on end LEDs (robot-relative: positive = move left)
    var y = robotRelativeDelta.getMeasureY().in(Centimeters);
    if (Math.abs(y) < LEDConstants.kPoseSeekYToleranceCm) {
      LEDSeries.POSE_Y.applyPattern(LEDPattern.solid(Color.kWhite));
    } else if (y > 0) {
      // Need to move left
      LEDSeries.POSE_Y.applyPattern(LEDPattern.solid(Color.kGreen));
    } else {
      // Need to move right
      LEDSeries.POSE_Y.applyPattern(LEDPattern.solid(Color.kRed));
    }
  }

  // ==================== PRE-ALLOCATED PATTERNS ====================

  /** Solid black pattern (LEDs off). */
  public static final LEDPattern solidBlackPattern = LEDPattern.solid(Color.kBlack);

  /** Solid yellow pattern. */
  public static final LEDPattern solidYellowPattern = LEDPattern.solid(Color.kYellow);

  /** Solid green pattern. */
  public static final LEDPattern solidGreenPattern = LEDPattern.solid(Color.kGreen);

  /** Bounce ripple pattern in yellow (spindexing, not on target). */
  public static final LEDPattern bounceRippleYellowPattern =
      LEDCustomPattern.bounceRipple(Color.kYellow);

  /** Bounce ripple pattern in green (spindexing, on target). */
  public static final LEDPattern bounceRippleGreenPattern =
      LEDCustomPattern.bounceRipple(Color.kGreen);

  /**
   * Pattern displaying the selected auto routine as counting blocks in alliance color. The number
   * of blocks corresponds to the auto option number.
   */
  public static LEDPattern autoSelectionPattern =
      LEDCustomPattern.countingBlocks(
          () -> Robot.autoSelector.get().get().getOptionNumber(),
          () -> Robot.autoSelector.get().get().getAllianceColor(),
          LEDConstants.kLEDsPerBlock,
          LEDConstants.kLEDsBetweenBlocks);

  /**
   * Pattern displaying a progress bar for the current match phase. Shows remaining time as a
   * filling bar in alliance color (red or blue depending on hub state). In the final 5 seconds,
   * resets to full and counts down while flashing.
   */
  public static LEDPattern hubCountdownPattern =
      LEDCustomPattern.urgentCountdown(
          // Remaining seconds
          () -> GameState.getCurrentPhase().remainingAt(GameState.getMatchTime()),
          // Total phase duration
          () -> GameState.getCurrentPhase().duration(),
          // Urgency threshold (5 seconds)
          10.0,
          // Fill color
          () -> {
            if (GameState.isMyHubActive()) {
              return GameState.getMyAlliance() == Alliance.Blue ? Color.kBlue : Color.kRed;
            }
            return GameState.getMyAlliance() == Alliance.Blue ? Color.kRed : Color.kBlue;
          },
          // Background color
          Color.kBlack,
          // Blink period (0.25s = 4Hz flash)
          0.25);

  /**
   * Displays the current auto selection on the LEDs. Shows counting blocks in alliance color
   * representing the auto option number. Blinks yellow if no auto is selected. Sets the last LED to
   * yellow if there is a mismatch between the driver station alliance and the selected alliance
   * color.
   */
  public void displayAutoSelection() {
    // In displayAutoSelection or somewhere you can test:
    // LEDSeries.Y_AXIS.applyPattern(LEDPattern.solid(Color.kRed));
    // LEDSeries.X_AXIS.applyPattern(LEDPattern.solid(Color.kBlue));

    Robot.autoSelector
        .get()
        .ifPresentOrElse(
            autoOption -> LEDSeries.AUTO_SELECTION.applyPattern(autoSelectionPattern),
            () -> LEDSeries.AUTO_SELECTION.applyPattern(solidYellowPattern.blink(Seconds.of(0.5))));

    // Display yellow warning pixel if alliance disagreement
    DriverStation.getAlliance()
        .ifPresent(
            alliance -> {
              if (alliance != Robot.allianceSelector.getAllianceColor()) {
                LEDSeries.WARNING_Y.applyPattern(LEDPattern.solid(Color.kYellow));
              }
            });

    // Display orange-red warning pixel if USB storage is low
    if (isUSBStorageLow()) {
      LEDSeries.WARNING_X.applyPattern(LEDPattern.solid(Color.kOrangeRed));
    }
  }

  /**
   * Displays a progress bar showing the remaining time in the current match phase. The bar fills in
   * alliance color based on hub state. When both hubs are active, shows our alliance color.
   *
   * <p>When FMS is connected, uses the full X-axis range (LEDs 12-35) to maximize visibility.
   * Otherwise uses X_AXIS_BODY (LEDs 14-35), leaving X_AXIS_WARNING free for other indicators.
   */
  public void displayHubCountdown() {
    if (DriverStation.isFMSAttached()) {
      LEDSeries.X_AXIS_FULL.applyPattern(hubCountdownPattern);
    } else {
      LEDSeries.X_AXIS_BODY.applyPattern(hubCountdownPattern);
    }
  }

  /**
   * Displays compressor state on the X_AXIS_WARNING pixel. Green when running, off when not.
   *
   * @param isRunning true when the compressor is actively running
   */
  public void displayCompressorState(boolean isRunning) {
    LEDSeries.WARNING_Y.applyPattern(isRunning ? solidGreenPattern : solidBlackPattern);
  }

  /**
   * Displays robot state on the Y_AXIS LEDs.
   *
   * <ul>
   *   <li><b>Color:</b> Yellow = not locked on target, Green = locked on target
   *   <li><b>Animation:</b> Solid = spindexer inactive, Ripple = spindexer active
   * </ul>
   *
   * @param isOnTarget supplies true when launcher is locked on target
   * @param isSpindexing supplies true when spindexer is active
   */
  public void displayRobotState(Supplier<Boolean> isOnTarget, Supplier<Boolean> isSpindexing) {
    boolean onTarget = isOnTarget.get();
    boolean spindexing = isSpindexing.get();

    LEDPattern pattern;
    if (onTarget) {
      pattern = spindexing ? bounceRippleGreenPattern : solidGreenPattern;
    } else {
      pattern = spindexing ? bounceRippleYellowPattern : solidYellowPattern;
    }
    LEDSeries.Y_AXIS.applyPattern(pattern);
  }

  /** Returns true when the USB stick at /U has less than 2 GB free. */
  private boolean isUSBStorageLow() {
    return Robot.getUSBStorageFreeSpace() < 2048L * 1024 * 1024;
  }

  /** Clears all LEDs by applying solid black. */
  public void clear() {
    clear(LEDSeries.ALL);
  }

  /**
   * Clears the specified LED series by applying solid black.
   *
   * @param series the first series to clear
   * @param more additional series to clear
   */
  public void clear(LEDSeries series, LEDSeries... more) {
    series.applyPattern(solidBlackPattern);
    for (var another : more) {
      another.applyPattern(solidBlackPattern);
    }
  }
}
