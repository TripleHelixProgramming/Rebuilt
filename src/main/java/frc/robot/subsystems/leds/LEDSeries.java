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
  // Full strip
  ALL(P.ALL),

  // Pose-seek segments (for displayPoseSeek feedback)
  // Layout: [Y_LEFT] [ROT_LEFT] [X_CENTER] [ROT_RIGHT] [Y_RIGHT]
  //         [0-1]    [2-3]      [4-7]      [8-9]       [10-11]
  POSE_Y_LEFT(P.POSE_Y_LEFT),
  POSE_Y_RIGHT(P.POSE_Y_RIGHT),
  POSE_ROTATION_LEFT(P.POSE_ROT_LEFT),
  POSE_ROTATION_RIGHT(P.POSE_ROT_RIGHT),
  POSE_ROTATION(P.POSE_ROT_LEFT, P.POSE_ROT_RIGHT),
  POSE_X_CENTER(P.POSE_X_CENTER);

  /** Portion definitions - single source of truth for physical layout. */
  private static final class P {
    // Full strip
    static final Portion ALL = new Portion(LEDStrip.MAIN, 0, 11, false);

    // Pose-seek portions: [Y_LEFT][ROT_LEFT][X_CENTER][ROT_RIGHT][Y_RIGHT]
    //                     [0-1]  [2-3]     [4-7]     [8-9]      [10-11]
    static final Portion POSE_Y_LEFT = new Portion(LEDStrip.MAIN, 0, 1, false);
    static final Portion POSE_ROT_LEFT = new Portion(LEDStrip.MAIN, 2, 3, false);
    static final Portion POSE_X_CENTER = new Portion(LEDStrip.MAIN, 4, 7, false);
    static final Portion POSE_ROT_RIGHT = new Portion(LEDStrip.MAIN, 8, 9, false);
    static final Portion POSE_Y_RIGHT = new Portion(LEDStrip.MAIN, 10, 11, false);
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
