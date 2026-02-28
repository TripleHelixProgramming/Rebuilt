package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;

/**
 * Defines the physical LED strips on the robot. Each strip is connected to a PWM port and has a
 * fixed number of LEDs.
 *
 * <p>Modify this enum when the physical LED layout changes between seasons.
 */
public enum LEDStrip {
  MAIN(0, 12);
  // Add more strips here as needed, e.g.:
  // FRONT(8, 60),
  // BACK(7, 30);

  private final int port;
  private final int length;
  private AddressableLED led;
  private AddressableLEDBuffer buffer;
  private boolean started = false;

  LEDStrip(int port, int length) {
    this.port = port;
    this.length = length;
  }

  public int getLength() {
    return length;
  }

  /** Gets the LED buffer, creating it if necessary. */
  public AddressableLEDBuffer getBuffer() {
    if (buffer == null) {
      led = new AddressableLED(port);
      buffer = new AddressableLEDBuffer(length);
      led.setLength(length);
    }
    return buffer;
  }

  /** Creates a view of a portion of this strip's buffer. */
  public AddressableLEDBufferView createView(int start, int end) {
    return getBuffer().createView(start, end);
  }

  /** Creates a reversed view of a portion of this strip's buffer. */
  public AddressableLEDBufferView createReversedView(int start, int end) {
    return getBuffer().createView(start, end).reversed();
  }

  /** Starts this strip. Must be called before the strip will display anything. */
  public void start() {
    if (!started) {
      getBuffer(); // Ensure LED and buffer are created
      led.start();
      started = true;
    }
  }

  /** Pushes buffer data to the physical LED strip. */
  public void update() {
    if (led != null) {
      led.setData(buffer);
    }
  }

  /** Starts all physical LED strips. Call once during robot initialization. */
  public static void startAll() {
    for (LEDStrip strip : values()) {
      strip.start();
    }
  }

  /** Updates all physical LED strips with their current buffer data. */
  public static void updateAll() {
    for (LEDStrip strip : values()) {
      strip.update();
    }
  }
}
