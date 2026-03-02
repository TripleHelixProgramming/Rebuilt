package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
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
      Supplier<Double> progressSupplier, Supplier<Color> colorSupplier, Color backgroundColor) {
    return (reader, writer) -> {
      int length = reader.getLength();
      double progress = Math.max(0, Math.min(1, progressSupplier.get()));
      // Use ceil so any non-zero progress keeps at least one LED lit
      int filledLeds = (int) Math.ceil(length * progress);
      for (int i = 0; i < length; i++) {
        writer.setLED(i, i < filledLeds ? colorSupplier.get() : backgroundColor);
      }
    };
  }

  /**
   * Creates a countdown progress bar that enters "urgent" mode in the final seconds. In urgent
   * mode, the bar resets to full and counts down while flashing.
   *
   * @param remainingSecondsSupplier supplies remaining time in seconds
   * @param totalDurationSupplier supplies the total phase duration in seconds
   * @param urgencyThresholdSeconds seconds remaining to trigger urgent mode (e.g., 5.0)
   * @param colorSupplier the fill color
   * @param backgroundColor the background color
   * @param blinkPeriodSeconds blink interval in urgent mode (e.g., 0.25 for fast flash)
   * @return the urgent countdown pattern
   */
  public static LEDPattern urgentCountdown(
      Supplier<Double> remainingSecondsSupplier,
      Supplier<Double> totalDurationSupplier,
      double urgencyThresholdSeconds,
      Supplier<Color> colorSupplier,
      Color backgroundColor,
      double blinkPeriodSeconds) {
    return (reader, writer) -> {
      int length = reader.getLength();
      double remaining = Math.max(0, remainingSecondsSupplier.get());
      double totalDuration = totalDurationSupplier.get();

      double progress;
      boolean urgent = remaining <= urgencyThresholdSeconds && remaining > 0;

      if (urgent) {
        // Reset to full, count down over urgency period
        progress = remaining / urgencyThresholdSeconds;
      } else {
        progress = totalDuration > 0 ? remaining / totalDuration : 0;
      }

      // Blink in urgent mode
      boolean showFill =
          !urgent || ((int) (Timer.getFPGATimestamp() / blinkPeriodSeconds)) % 2 == 0;

      int filledLeds = (int) Math.ceil(length * Math.max(0, Math.min(1, progress)));
      Color fillColor = showFill ? colorSupplier.get() : backgroundColor;

      for (int i = 0; i < length; i++) {
        writer.setLED(i, i < filledLeds ? fillColor : backgroundColor);
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
   * Creates a bounce ripple pattern that expands from the center outward, then contracts back
   * inward, like a water drop ripple played forwards and backwards. Includes a comet-like tail that
   * trails behind the ripple.
   *
   * @param color the color to display
   * @param rippleWidth number of LEDs in the ripple band
   * @param tailLength number of LEDs in the trailing fade
   * @param cyclesPerSecond how many full expand-contract cycles per second
   * @return the bounce ripple pattern
   */
  public static LEDPattern bounceRipple(
      Color color, int rippleWidth, int tailLength, double cyclesPerSecond) {
    return (reader, writer) -> {
      int length = reader.getLength();
      int halfLength = length / 2;

      // Triangle wave for ping-pong motion (0 to 1 to 0)
      double time = Timer.getFPGATimestamp();
      double phase = time * cyclesPerSecond;
      double sawtooth = phase % 1.0; // 0 -> 1
      double triangleWave = Math.abs(sawtooth * 2 - 1); // 0 -> 1 -> 0
      boolean expanding = sawtooth < 0.5; // First half = expanding, second half = contracting

      // Calculate ripple position (distance from center)
      double rippleRadius = triangleWave * halfLength;

      for (int i = 0; i < length; i++) {
        double distanceFromCenter = Math.abs(i - (length - 1) / 2.0);
        double distanceFromRipple = Math.abs(distanceFromCenter - rippleRadius);

        double brightness = 0;

        // Main ripple band
        if (distanceFromRipple < rippleWidth / 2.0) {
          brightness = 1.0 - (distanceFromRipple / (rippleWidth / 2.0));
        }
        // Comet tail - trails behind the ripple
        else if (tailLength > 0) {
          // Tail is toward center when expanding, toward edges when contracting
          boolean inTailZone =
              expanding
                  ? distanceFromCenter < rippleRadius // Tail toward center
                  : distanceFromCenter > rippleRadius; // Tail toward edges

          if (inTailZone) {
            double tailDistance = distanceFromRipple - rippleWidth / 2.0;
            if (tailDistance < tailLength) {
              brightness = 0.6 * (1.0 - tailDistance / tailLength); // Fade from 60% to 0
            }
          }
        }

        if (brightness > 0) {
          writer.setLED(
              i,
              new Color(color.red * brightness, color.green * brightness, color.blue * brightness));
        } else {
          writer.setLED(i, Color.kBlack);
        }
      }
    };
  }

  /**
   * Creates a bounce ripple pattern with configurable ripple width.
   *
   * @param color the color to display
   * @param rippleWidth number of LEDs in the ripple band
   * @param cyclesPerSecond how many full expand-contract cycles per second
   * @return the bounce ripple pattern
   */
  public static LEDPattern bounceRipple(Color color, int rippleWidth, double cyclesPerSecond) {
    return bounceRipple(color, rippleWidth, 3, cyclesPerSecond); // Default 3 LED tail
  }

  /**
   * Creates a bounce ripple pattern with default settings (3 LED width, 1.5 cycles per second).
   *
   * @param color the color to display
   * @return the bounce ripple pattern
   */
  public static LEDPattern bounceRipple(Color color) {
    return bounceRipple(color, 3, 1.5);
  }

  /**
   * Creates a solid pattern showing the current alliance color.
   *
   * @param isRedAlliance supplies true for red alliance, false for blue
   * @return red when on red alliance, blue when on blue alliance
   */
  public static LEDPattern allianceColor() {
    if (allianceColorPattern == null) {
      allianceColorPattern =
          solidIf(() -> Robot.getAlliance() == Alliance.Blue, Color.kBlue, Color.kRed);
    }
    return allianceColorPattern;
  }

  private static LEDPattern allianceColorPattern;
}
