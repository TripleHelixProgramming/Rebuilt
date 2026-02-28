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
  // Individual segments (for fine-grained control)
  // These define the physical layout - update these when hardware changes
  TOP_LEFT_LARBOARD(P.TOP_AFT_LARBOARD),
  // RIGHT_BOTTOM(P.RIGHT_BOTTOM),
  // RIGHT_MIDDLE(P.RIGHT_MIDDLE),
  // RIGHT_TOP(P.RIGHT_TOP),
  // LEFT_TOP(P.LEFT_TOP),
  // LEFT_MIDDLE(P.LEFT_MIDDLE),
  // LEFT_BOTTOM(P.LEFT_BOTTOM),

  // // Composite views (built from individual segment portions)
  // LEFT(P.LEFT_TOP, P.LEFT_MIDDLE, P.LEFT_BOTTOM),
  // RIGHT(P.RIGHT_BOTTOM, P.RIGHT_MIDDLE, P.RIGHT_TOP),
  // TOP(P.LEFT_TOP, P.RIGHT_TOP),
  // MIDDLE(P.LEFT_MIDDLE, P.RIGHT_MIDDLE),
  // BOTTOM(P.LEFT_BOTTOM, P.RIGHT_BOTTOM),
  // TOP_AND_BOTTOM(P.LEFT_TOP, P.RIGHT_TOP, P.LEFT_BOTTOM, P.RIGHT_BOTTOM),
  ALL(P.TOP_AFT_LARBOARD);

  /** Portion definitions - single source of truth for physical layout. */
  private static final class P {
    static final Portion TOP_AFT_LARBOARD = new Portion(LEDStrip.MAIN, 0, 11, false);
    // static final Portion RIGHT_BOTTOM = new Portion(LEDStrip.MAIN, 0, , true);
    // static final Portion RIGHT_MIDDLE = new Portion(LEDStrip.MAIN, 8, 11, true);
    // static final Portion RIGHT_TOP = new Portion(LEDStrip.MAIN, 12, 19, true);
    // static final Portion LEFT_TOP = new Portion(LEDStrip.MAIN, 20, 27, false);
    // static final Portion LEFT_MIDDLE = new Portion(LEDStrip.MAIN, 28, 31, false);
    // static final Portion LEFT_BOTTOM = new Portion(LEDStrip.MAIN, 32, 39, false);
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
