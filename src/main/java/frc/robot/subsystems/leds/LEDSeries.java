package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;

/**
 * Defines logical LED series on the robot. Each series can span multiple physical strips, composed
 * of one or more portions. Series implement {@link LEDReader} and {@link LEDWriter} to present a
 * unified virtual buffer that makes all portions appear contiguous.
 *
 * <p>Modify this enum when the physical LED layout changes between seasons.
 */
public enum LEDSeries implements LEDReader, LEDWriter {
  // Full strip (all 36 LEDs)
  ALL(P.ALL),

  // The strip is an L at the front-right corner of the frame: 12 LEDs along the front edge, then
  // 24 back along the right side. Primary segments, with the portions inside them:
  // Y_AXIS:                 LEDs 0-11,  front edge, LED 11 at the corner (spindexer/target state)
  //   WARNING_STORAGE:        LEDs 0-1,   USB storage low warning (disabled)
  // X_AXIS_FULL:            LEDs 12-35, right side from the corner back (hub countdown with FMS)
  //   WARNING_COMPRESSOR:     LEDs 12-13, compressor running indicator
  //                           LED  14,    implicit gap
  //   X_AXIS_BODY:            LEDs 15-35, hub countdown range without FMS
  //     WARNING_ALLIANCE:       LEDs 22-23, alliance disagreement warning (disabled)
  //     AUTO_SELECTION:         LEDs 24-35, auto selection blocks (disabled)
  Y_AXIS(P.Y_AXIS),
  X_AXIS_FULL(P.X_AXIS_FULL),
  WARNING_ALLIANCE(P.WARNING_ALLIANCE),
  X_AXIS_BODY(P.X_AXIS_BODY),
  WARNING_STORAGE(P.WARNING_STORAGE),
  WARNING_COMPRESSOR(P.WARNING_COMPRESSOR),

  // Auto selection
  AUTO_SELECTION(P.AUTO_SELECTION),

  // Pose-seek segments, shown while disabled (see LEDController.displayPoseSeek)
  // POSE_Y:                 LEDs 3-6,   front edge
  // POSE_ROTATION_Y:        LEDs 8-11,  front edge, at the corner
  // POSE_ROTATION_X:        LEDs 12-15, right side, at the corner
  // POSE_X:                 LEDs 17-20, right side
  // Green means move that edge toward the corner, red means away from it (see doc/LED.md)
  POSE_Y(P.POSE_Y),
  POSE_ROTATION_X(P.POSE_ROTATION_X),
  POSE_ROTATION_Y(P.POSE_ROTATION_Y),
  POSE_X(P.POSE_X),
  POSE_ROTATION(P.POSE_ROTATION_X, P.POSE_ROTATION_Y);

  /** Portion definitions - single source of truth for physical layout. */
  private static final class P {
    // Full strip (all 36 LEDs)
    static final Portion ALL = new Portion(LEDStrip.MAIN, 0, 35, false);

    // Primary segments by robot axis
    static final Portion Y_AXIS = new Portion(LEDStrip.MAIN, 0, 11, false);
    static final Portion X_AXIS_FULL = new Portion(LEDStrip.MAIN, 12, 35, false);
    // LED 14 is an implicit gap between WARNING_COMPRESSOR and the body
    static final Portion X_AXIS_BODY = new Portion(LEDStrip.MAIN, 15, 35, false);
    // Warning portions overlap the axis segments above
    static final Portion WARNING_STORAGE = new Portion(LEDStrip.MAIN, 0, 1, false);
    static final Portion WARNING_COMPRESSOR = new Portion(LEDStrip.MAIN, 12, 13, false);
    static final Portion WARNING_ALLIANCE = new Portion(LEDStrip.MAIN, 22, 23, false);

    // Auto selection
    static final Portion AUTO_SELECTION = new Portion(LEDStrip.MAIN, 24, 35, false);

    // Pose-seek portions: POSE_Y and POSE_ROTATION_Y on Y_AXIS, the other two on X_AXIS_FULL
    static final Portion POSE_Y = new Portion(LEDStrip.MAIN, 3, 6, false);
    static final Portion POSE_ROTATION_Y = new Portion(LEDStrip.MAIN, 8, 11, false);
    static final Portion POSE_ROTATION_X = new Portion(LEDStrip.MAIN, 12, 15, false);
    static final Portion POSE_X = new Portion(LEDStrip.MAIN, 17, 20, false);
  }

  /** Defines a portion of a strip. */
  public record Portion(LEDStrip strip, int start, int end, boolean reversed) {
    public int length() {
      return end - start + 1;
    }

    public AddressableLEDBufferView createView() {
      return reversed ? strip.createReversedView(start, end) : strip.createView(start, end);
    }
  }

  private final Portion[] portions;
  private final int totalLength;
  private AddressableLEDBufferView[] bufferViews;

  LEDSeries(Portion... portions) {
    this.portions = portions;
    int len = 0;
    for (Portion p : portions) {
      len += p.length();
    }
    this.totalLength = len;
  }

  // ==================== LEDReader Implementation ====================

  @Override
  public int getLength() {
    return totalLength;
  }

  /** Returns the buffer view containing the given virtual index. */
  private AddressableLEDBufferView viewFor(int index) {
    int offset = 0;
    AddressableLEDBufferView[] views = getBufferViews();
    for (int i = 0; i < portions.length; i++) {
      int portionLen = portions[i].length();
      if (index < offset + portionLen) {
        return views[i];
      }
      offset += portionLen;
    }
    throw new IndexOutOfBoundsException(
        "Index " + index + " out of bounds for length " + totalLength);
  }

  /** Converts a virtual index to a local index within its portion. */
  private int localIndex(int index) {
    int offset = 0;
    for (Portion portion : portions) {
      int portionLen = portion.length();
      if (index < offset + portionLen) {
        return index - offset;
      }
      offset += portionLen;
    }
    throw new IndexOutOfBoundsException(
        "Index " + index + " out of bounds for length " + totalLength);
  }

  @Override
  public int getRed(int index) {
    return viewFor(index).getRed(localIndex(index));
  }

  @Override
  public int getGreen(int index) {
    return viewFor(index).getGreen(localIndex(index));
  }

  @Override
  public int getBlue(int index) {
    return viewFor(index).getBlue(localIndex(index));
  }

  // ==================== LEDWriter Implementation ====================

  @Override
  public void setRGB(int index, int r, int g, int b) {
    viewFor(index).setRGB(localIndex(index), r, g, b);
  }

  // ==================== Buffer View Access ====================

  /**
   * Gets the buffer views for all portions in this series, creating them if necessary.
   *
   * @return array of buffer views
   */
  public AddressableLEDBufferView[] getBufferViews() {
    if (bufferViews == null) {
      bufferViews = new AddressableLEDBufferView[portions.length];
      for (int i = 0; i < portions.length; i++) {
        bufferViews[i] = portions[i].createView();
      }
    }
    return bufferViews;
  }

  // ==================== Pattern Application ====================

  /**
   * Applies a pattern to this series as a unified buffer. The pattern sees this series as one
   * contiguous strip of LEDs.
   *
   * @param pattern the pattern to apply
   */
  public void applyPattern(LEDPattern pattern) {
    pattern.applyTo(this);
  }
}
