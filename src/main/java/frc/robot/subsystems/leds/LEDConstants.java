package frc.robot.subsystems.leds;

/**
 * Constants for LED animations. Physical strip and segment configuration is defined in the {@link
 * LEDStrip} and {@link LEDSeries} enums.
 */
public final class LEDConstants {

  private LEDConstants() {}

  // ==================== ANIMATION CONFIGURATION ====================

  public static final int LEDS_PER_BLOCK = 1;
  public static final int LEDS_BETWEEN_BLOCKS = 1;

  // ==================== POSE SEEK TOLERANCES ====================

  /** Heading tolerance in degrees for pose-seek feedback. */
  public static final double POSE_SEEK_HEADING_TOL_DEGREES = 3.0;

  /** X position tolerance in centimeters for pose-seek feedback. */
  public static final double POSE_SEEK_X_TOL_CM = 5.0;

  /** Y position tolerance in centimeters for pose-seek feedback. */
  public static final double POSE_SEEK_Y_TOL_CM = 6.0;
}
