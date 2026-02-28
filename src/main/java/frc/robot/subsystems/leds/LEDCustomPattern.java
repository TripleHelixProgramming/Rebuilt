package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Custom LED patterns for the robot. These patterns follow the WPILib LEDPattern design: stateless
 * lambdas that can be applied to any buffer and composed with modifiers like {@code
 * .scrollAtRelativeSpeed()}, {@code .blink()}, etc.
 */
public final class LEDCustomPattern {

  private LEDCustomPattern() {}

  /**
   * Creates a pattern of colored blocks with gaps between them.
   *
   * @param color the block color
   * @param blockSize number of LEDs per block
   * @param gapSize number of LEDs between blocks
   * @return the stacked blocks pattern
   */
  public static LEDPattern stackedBlocks(Color color, int blockSize, int gapSize) {
    return stackedBlocks(color, blockSize, gapSize, Color.kBlack);
  }

  /**
   * Creates a pattern of colored blocks with gaps of a specified color.
   *
   * @param color the block color
   * @param blockSize number of LEDs per block
   * @param gapSize number of LEDs between blocks
   * @param gapColor the color for gaps between blocks
   * @return the stacked blocks pattern
   */
  public static LEDPattern stackedBlocks(Color color, int blockSize, int gapSize, Color gapColor) {
    int period = blockSize + gapSize;
    return (reader, writer) -> {
      for (int i = 0; i < reader.getLength(); i++) {
        writer.setLED(i, (i % period) < blockSize ? color : gapColor);
      }
    };
  }

  /**
   * Creates a pattern that shows one color when a condition is true, another when false.
   *
   * @param condition the condition to evaluate
   * @param trueColor color when condition is true
   * @param falseColor color when condition is false
   * @return the conditional solid pattern
   */
  public static LEDPattern solidIf(BooleanSupplier condition, Color trueColor, Color falseColor) {
    return (reader, writer) -> {
      Color c = condition.getAsBoolean() ? trueColor : falseColor;
      LEDPattern.solid(c).applyTo(reader, writer);
    };
  }

  /**
   * Creates a scrolling blocks pattern for intake/outtake animations.
   *
   * @param color the block color
   * @return a scrolling stacked blocks pattern
   */
  public static LEDPattern scrollingBlocks(Color color) {
    return scrollingBlocks(color, 3, 2);
  }

  /**
   * Creates a scrolling blocks pattern with configurable sizes.
   *
   * @param color the block color
   * @param blockSize number of LEDs per block
   * @param gapSize number of LEDs between blocks
   * @return a scrolling stacked blocks pattern
   */
  public static LEDPattern scrollingBlocks(Color color, int blockSize, int gapSize) {
    return stackedBlocks(color, blockSize, gapSize)
        .scrollAtRelativeSpeed(edu.wpi.first.units.Units.Hertz.of(2));
  }

  /**
   * Creates a pattern showing a progress bar from 0 to the specified ratio.
   *
   * @param progressSupplier supplies progress from 0.0 to 1.0
   * @param color the fill color
   * @param backgroundColor the background color
   * @return the progress bar pattern
   */
  public static LEDPattern progressBar(
      Supplier<Double> progressSupplier, Color color, Color backgroundColor) {
    return (reader, writer) -> {
      int length = reader.getLength();
      int filledLeds = (int) (length * Math.max(0, Math.min(1, progressSupplier.get())));
      for (int i = 0; i < length; i++) {
        writer.setLED(i, i < filledLeds ? color : backgroundColor);
      }
    };
  }

  /**
   * Creates a pattern showing status as a color gradient based on a value.
   *
   * @param valueSupplier supplies a value from 0.0 to 1.0
   * @param lowColor color at 0.0
   * @param highColor color at 1.0
   * @return the status gradient pattern
   */
  public static LEDPattern statusGradient(
      Supplier<Double> valueSupplier, Color lowColor, Color highColor) {
    return (reader, writer) -> {
      double t = Math.max(0, Math.min(1, valueSupplier.get()));
      Color blended =
          new Color(
              lowColor.red + (highColor.red - lowColor.red) * t,
              lowColor.green + (highColor.green - lowColor.green) * t,
              lowColor.blue + (highColor.blue - lowColor.blue) * t);
      LEDPattern.solid(blended).applyTo(reader, writer);
    };
  }

  /**
   * Creates a pattern that displays N blocks to indicate a count (e.g., for auto selection).
   *
   * @param countSupplier supplies the number of blocks to display
   * @param color the block color
   * @param blockSize LEDs per block
   * @param gapSize LEDs between blocks
   * @return the counting blocks pattern
   */
  public static LEDPattern countingBlocks(
      Supplier<Integer> countSupplier, Supplier<Color> colorSupplier, int blockSize, int gapSize) {
    int period = blockSize + gapSize;
    return (reader, writer) -> {
      int count = countSupplier.get();
      var color = colorSupplier.get();
      int maxLed = count * period;
      for (int i = 0; i < reader.getLength(); i++) {
        if (i < maxLed && (i % period) < blockSize) {
          writer.setLED(i, color);
        } else {
          writer.setLED(i, Color.kBlack);
        }
      }
    };
  }

  /**
   * Creates a solid pattern showing the current alliance color.
   *
   * @param isRedAlliance supplies true for red alliance, false for blue
   * @return red when on red alliance, blue when on blue alliance
   */
  public static LEDPattern allianceColor(BooleanSupplier isRedAlliance) {
    return solidIf(isRedAlliance, Color.kRed, Color.kBlue);
  }
}
