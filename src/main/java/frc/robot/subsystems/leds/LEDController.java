package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

/**
 * A subsystem to control the LEDs on the robot.
 *
 * <p>Physical strips are defined in {@link LEDStrip}, logical groups in {@link LEDGroup}, and
 * custom patterns in {@link LEDCustomPattern}.
 *
 * <p>Apply patterns directly to groups: {@code LEDGroup.ALL.applyPattern(pattern)}
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

  // ==================== COMMAND FACTORIES ====================

  /**
   * Creates a command that continuously applies a pattern to a group.
   *
   * @param pattern the pattern to apply
   * @param group the group to apply to
   * @return a command that runs the pattern
   */
  public Command runPattern(LEDPattern pattern, LEDGroup group) {
    return run(() -> group.applyPattern(pattern));
  }

  /**
   * Creates a command that continuously applies a pattern from a supplier.
   *
   * @param patternSupplier supplies the pattern to apply
   * @param group the group to apply to
   * @return a command that runs the pattern
   */
  public Command runPattern(Supplier<LEDPattern> patternSupplier, LEDGroup group) {
    return run(() -> group.applyPattern(patternSupplier.get()));
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
    var delta = targetPose.minus(currentPose);

    // Heading feedback on MIDDLE
    var theta = MathUtil.inputModulus(delta.getRotation().getDegrees(), -180, 180);
    Color headingColor =
        Math.abs(theta) < LEDConstants.kPoseSeekHeadingToleranceDegrees
            ? Color.kWhite
            : theta > 0 ? Color.kMagenta : Color.kCyan;
    LEDGroup.MIDDLE.applyPattern(LEDPattern.solid(headingColor));

    // X feedback on TOP
    var x = delta.getTranslation().getMeasureX().in(Centimeters);
    Color xColor =
        Math.abs(x) < LEDConstants.kPoseSeekXToleranceCm
            ? Color.kWhite
            : x > 0 ? Color.kGreen : Color.kRed;
    LEDGroup.TOP.applyPattern(LEDPattern.solid(xColor));

    // Y feedback on BOTTOM
    var y = delta.getTranslation().getMeasureY().in(Centimeters);
    if (Math.abs(y) < LEDConstants.kPoseSeekYToleranceCm) {
      LEDGroup.BOTTOM.applyPattern(LEDPattern.solid(Color.kWhite));
    } else {
      LEDGroup yGroup = y > 0 ? LEDGroup.LEFT_BOTTOM : LEDGroup.RIGHT_BOTTOM;
      LEDGroup otherGroup = y > 0 ? LEDGroup.RIGHT_BOTTOM : LEDGroup.LEFT_BOTTOM;
      yGroup.applyPattern(LEDPattern.solid(Color.kGreen));
      otherGroup.applyPattern(LEDPattern.solid(Color.kBlack));
    }
  }

  /**
   * Creates a command to display pose-seek feedback.
   *
   * @param currentPoseSupplier supplies the current pose
   * @param targetPose the target pose
   * @return a command that displays pose-seek feedback
   */
  public Command runPoseSeek(Supplier<Pose2d> currentPoseSupplier, Pose2d targetPose) {
    return run(() -> displayPoseSeek(currentPoseSupplier.get(), targetPose));
  }

  /**
   * Creates a command to display auto mode selection.
   *
   * @param autoNumberSupplier supplies the auto mode number (1-based)
   * @param allianceColorSupplier supplies the alliance color
   * @return a command that displays auto selection
   */
  public Command runAutoSelection(
      Supplier<Integer> autoNumberSupplier, Supplier<Color> allianceColorSupplier) {
    return run(
        () -> {
          LEDPattern pattern =
              LEDCustomPattern.countingBlocks(
                  autoNumberSupplier,
                  allianceColorSupplier.get(),
                  LEDConstants.kLEDsPerBlock,
                  LEDConstants.kLEDsBetweenBlocks);
          LEDGroup.ALL.applyPattern(pattern);
        });
  }
}
