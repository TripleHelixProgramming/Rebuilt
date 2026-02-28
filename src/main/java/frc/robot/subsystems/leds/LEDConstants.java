package frc.robot.subsystems.leds;

/**
 * Constants for LED animations. Physical strip and segment configuration is defined in the {@link
 * LEDStrip} and {@link LEDSeries} enums.
 */
public final class LEDConstants {

  private LEDConstants() {}

  // ==================== ANIMATION CONFIGURATION ====================

  public static final int kLEDsPerBlock = 2;
  public static final int kLEDsBetweenBlocks = 1;

  // ==================== POSE SEEK TOLERANCES ====================

  /** Heading tolerance in degrees for pose-seek feedback. */
  public static final double kPoseSeekHeadingToleranceDegrees = 3.0;

  /** X position tolerance in centimeters for pose-seek feedback. */
  public static final double kPoseSeekXToleranceCm = 5.0;

  /** Y position tolerance in centimeters for pose-seek feedback. */
  public static final double kPoseSeekYToleranceCm = 6.0;
}
