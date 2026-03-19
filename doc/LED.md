# LED System

This document explains the LED subsystem architecture and how to use it.

## Overview

The LED system provides a flexible way to control addressable LED strips on the robot. It separates concerns into distinct layers:

| Class | Purpose |
|-------|---------|
| `LEDStrip` | Physical hardware (PWM ports, buffers) |
| `LEDSeries` | Logical groupings (segments, composite series) |
| `LEDCustomPattern` | Custom pattern factories |
| `LEDController` | Singleton subsystem with display methods |
| `LEDConstants` | Animation and tolerance constants |

### Design Rationale

**Why enums for strips and series?**
Enums provide compile-time safety with no string parsing or map lookups at runtime. When you write `LEDSeries.ALL`, the compiler validates it exists.

**Why separate LEDStrip from LEDSeries?**
Physical layout (which PWM port, how many LEDs) changes rarely. Logical groupings (what we call "all" or "X_AXIS") are how code interacts with LEDs. Separating them means you can reorganize logical series without rewiring, or rewire without changing application code.

**Why implement LEDReader/LEDWriter?**
Series can span multiple physical portions. By implementing WPILib's `LEDReader` and `LEDWriter` interfaces, series present a unified virtual buffer. Patterns see one contiguous strip of LEDs, even if the series spans multiple physical segments.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      LEDController                          │
│  - Singleton subsystem                                      │
│  - Calls LEDStrip.updateAll() every periodic cycle          │
│  - Provides display methods (displayAutoSelection, etc.)    │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                        LEDSeries                            │
│  - Enum of logical series (Y_AXIS, X_AXIS_FULL, ALL, etc.)  │
│  - Implements LEDReader + LEDWriter for unified buffer      │
│  - applyPattern() applies any LEDPattern                    │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                        LEDStrip                             │
│  - Enum of physical strips (MAIN, etc.)                     │
│  - Manages AddressableLED and AddressableLEDBuffer          │
│  - startAll() / updateAll() for lifecycle                   │
└─────────────────────────────────────────────────────────────┘
```

## Current Physical Layout

The robot currently has one LED strip:

| Strip | PWM Port | LED Count |
|-------|----------|-----------|
| MAIN  | 0        | 36        |

### Series Configuration

The strip is divided into two primary segments by robot axis:

| Series | LEDs | Description |
|--------|------|-------------|
| `ALL` | 0-35 | All LEDs as one series |
| `Y_AXIS` | 0-11 | Along robot Y axis (pose-seek feedback, robot state) |
| `X_AXIS_FULL` | 12-35 | Full X-axis range (hub countdown when FMS attached) |
| `X_AXIS_BODY` | 15-35 | Main X-axis display area (hub countdown when FMS not attached) |
| `AUTO_SELECTION` | 24-35 | Auto selection counting blocks |

#### Warning Indicators

Reserved LEDs for status warnings:

| Series | LEDs | Description |
|--------|------|-------------|
| `WARNING_STORAGE` | 0-1 | USB storage low warning (orange-red when <2GB free) |
| `WARNING_COMPRESSOR` | 12-13 | Compressor state (green when running) |
| `WARNING_ALLIANCE` | 22-23 | Alliance mismatch warning (yellow when DS/selected disagree) |

#### Pose-Seek Segments (within Y_AXIS)

The Y_AXIS segment includes pose-seek feedback for manual alignment:

| Series | LEDs | Description |
|--------|------|-------------|
| `POSE_Y` | 3-6 | Y translation feedback |
| `POSE_ROTATION_Y` | 8-11 | Rotation feedback (Y axis side) |
| `POSE_ROTATION_X` | 12-15 | Rotation feedback (X axis side) |
| `POSE_X` | 17-20 | X translation feedback |
| `POSE_ROTATION` | 8-11, 12-15 | Combined rotation (composite series) |

## Display Functions

The `LEDController` provides high-level display methods for common robot states:

### displayAutoSelection()

Displays the currently selected autonomous routine during disabled mode.

- Shows counting blocks in alliance color (red or blue)
- Number of blocks corresponds to the auto option number (1, 2, 3, etc.)
- Blinks yellow if no auto is selected
- Shows yellow warning on `WARNING_ALLIANCE` if there's a mismatch between driver station alliance and selected alliance
- Shows orange-red warning on `WARNING_STORAGE` if USB storage is below 2GB free

**Used on:** `AUTO_SELECTION` (LEDs 24-35)

### displayPoseSeek(currentPose, targetPose)

Provides visual feedback for manually aligning the robot to a target pose. Useful during disabled mode to help drivers position the robot for autonomous. Coordinates are transformed to robot-relative.

| Segment | Meaning |
|---------|---------|
| **X (POSE_X)** | Green = drive forward, Red = drive backward, White = correct |
| **Rotation (POSE_ROTATION_X/Y)** | Green on one side, Red on other side depending on rotation direction; White = correct |
| **Y (POSE_Y)** | Red = move left needed, Green = move right needed, White = correct |

Tolerances are defined in `LEDConstants`:
- X: 5 cm
- Y: 6 cm
- Heading: 3°

**Used on:** `POSE_X`, `POSE_Y`, `POSE_ROTATION_X`, `POSE_ROTATION_Y` segments

### displayRobotState(isOnTarget, isSpindexing)

Shows launcher and spindexer state during teleop and autonomous.

| State | Color | Animation |
|-------|-------|-----------|
| Not on target, spindexer inactive | Yellow | Solid |
| Not on target, spindexer active | Yellow | Bounce ripple |
| On target, spindexer inactive | Green | Solid |
| On target, spindexer active | Green | Bounce ripple |

**Used on:** `Y_AXIS` (LEDs 0-11)

### displayHubCountdown()

Shows remaining time in the current match phase as an "urgent countdown" progress bar.

- Bar fills in alliance color (red or blue based on hub state)
- Empties as time runs out
- In the final 10 seconds, resets to full and counts down while flashing (4Hz)
- When FMS is attached, uses full X-axis range for maximum visibility

**Used on:** `X_AXIS_FULL` (LEDs 12-35) when FMS attached, `X_AXIS_BODY` (LEDs 15-35) otherwise

### displayCompressorState(isRunning)

Shows compressor state on the warning indicator.

- Green when compressor is actively running
- Off (black) when compressor is idle

**Used on:** `WARNING_COMPRESSOR` (LEDs 12-13)

### clear() / clear(series, ...)

Turns off all LEDs or specific series by applying solid black.

## Quick Start

### Basic Usage

```java
// Apply a solid color to all LEDs
LEDSeries.ALL.applyPattern(LEDPattern.solid(Color.kGreen));

// Apply a built-in WPILib pattern
LEDSeries.ALL.applyPattern(LEDPattern.rainbow(255, 255));

// Apply a custom pattern
LEDSeries.X_AXIS.applyPattern(LEDCustomPattern.scrollingBlocks(Color.kOrange));
```

### Using LEDController

The `LEDController` provides pre-built display methods:

```java
LEDController leds = LEDController.getInstance();

// Display auto selection (counting blocks in alliance color)
// Also shows warning indicators for alliance mismatch and low USB storage
leds.displayAutoSelection();

// Display pose-seek feedback
leds.displayPoseSeek(drive.getPose(), targetPose);

// Display robot state (on Y_AXIS)
leds.displayRobotState(() -> launcher.isOnTarget(), () -> feeder.isSpinning());

// Display hub countdown (urgent countdown bar on X_AXIS)
leds.displayHubCountdown();

// Display compressor state
leds.displayCompressorState(compressor.isEnabled());

// Clear all LEDs
leds.clear();
```

## Physical Layout Configuration

### Defining Strips

In `LEDStrip.java`, define each physical LED strip:

```java
public enum LEDStrip {
  MAIN(0, 36);      // PWM port 0, 36 LEDs
  // FRONT(8, 60),  // Uncomment to add more strips
  // BACK(7, 30);
  ...
}
```

### Defining Portions

In `LEDSeries.java`, the `P` class defines portions - the single source of truth for physical layout:

```java
private static final class P {
  // (strip, startIndex, endIndex, reversed)
  static final Portion ALL = new Portion(LEDStrip.MAIN, 0, 35, false);

  // Primary segments
  static final Portion Y_AXIS = new Portion(LEDStrip.MAIN, 0, 11, false);
  static final Portion X_AXIS_FULL = new Portion(LEDStrip.MAIN, 12, 35, false);
  static final Portion X_AXIS_BODY = new Portion(LEDStrip.MAIN, 15, 35, false);

  // Warning indicators
  static final Portion WARNING_STORAGE = new Portion(LEDStrip.MAIN, 0, 1, false);
  static final Portion WARNING_COMPRESSOR = new Portion(LEDStrip.MAIN, 12, 13, false);
  static final Portion WARNING_ALLIANCE = new Portion(LEDStrip.MAIN, 22, 23, false);

  // Auto selection
  static final Portion AUTO_SELECTION = new Portion(LEDStrip.MAIN, 24, 35, false);

  // Pose-seek portions
  static final Portion POSE_Y = new Portion(LEDStrip.MAIN, 3, 6, false);
  static final Portion POSE_ROTATION_Y = new Portion(LEDStrip.MAIN, 8, 11, false);
  static final Portion POSE_ROTATION_X = new Portion(LEDStrip.MAIN, 12, 15, false);
  static final Portion POSE_X = new Portion(LEDStrip.MAIN, 17, 20, false);
}
```

The `reversed` flag controls animation direction. Set `true` if LEDs are wired in the opposite direction from your logical "forward."

### Defining Series

Series reference portions from `P`:

```java
public enum LEDSeries implements LEDReader, LEDWriter {
  ALL(P.ALL),
  Y_AXIS(P.Y_AXIS),
  X_AXIS_FULL(P.X_AXIS_FULL),
  X_AXIS_BODY(P.X_AXIS_BODY),
  WARNING_ALLIANCE(P.WARNING_ALLIANCE),
  WARNING_STORAGE(P.WARNING_STORAGE),
  WARNING_COMPRESSOR(P.WARNING_COMPRESSOR),
  AUTO_SELECTION(P.AUTO_SELECTION),
  POSE_Y(P.POSE_Y),
  POSE_ROTATION(P.POSE_ROTATION_X, P.POSE_ROTATION_Y),  // Composite series
  ...
}
```

## Creating Custom Patterns

Patterns are lambdas that implement `LEDPattern`:

```java
// In LEDCustomPattern.java
public static LEDPattern myPattern(Color color) {
  return (reader, writer) -> {
    for (int i = 0; i < reader.getLength(); i++) {
      // Your pattern logic here
      writer.setLED(i, color);
    }
  };
}
```

### Using WPILib Modifiers

Patterns can be composed with modifiers:

```java
// Make any pattern scroll
LEDPattern scrolling = LEDCustomPattern.stackedBlocks(Color.kGreen, 3, 2)
    .scrollAtRelativeSpeed(Units.Hertz.of(2));

// Make any pattern blink
LEDPattern blinking = LEDPattern.solid(Color.kRed)
    .blink(Units.Seconds.of(0.5));

// Combine modifiers
LEDPattern scrollingAndBreathing = myPattern
    .scrollAtRelativeSpeed(Units.Hertz.of(1))
    .breathe(Units.Seconds.of(2));
```

### Dynamic Patterns

For patterns that change based on robot state, use suppliers:

```java
public static LEDPattern progressBar(
    Supplier<Double> progressSupplier,
    Supplier<Color> colorSupplier,
    Color backgroundColor) {
  return (reader, writer) -> {
    int length = reader.getLength();
    int filled = (int) (length * progressSupplier.get());
    for (int i = 0; i < length; i++) {
      writer.setLED(i, i < filled ? colorSupplier.get() : backgroundColor);
    }
  };
}
```

### Urgent Countdown Pattern

The `urgentCountdown` pattern is designed for match timers with an "urgent" final phase:

```java
public static LEDPattern urgentCountdown(
    Supplier<Double> remainingSecondsSupplier,
    Supplier<Double> totalDurationSupplier,
    double urgencyThresholdSeconds,  // e.g., 10.0
    Supplier<Color> colorSupplier,
    Color backgroundColor,
    double blinkPeriodSeconds) {     // e.g., 0.25 for 4Hz flash
  ...
}
```

## Available Patterns

### WPILib Built-in Patterns

```java
LEDPattern.solid(Color.kGreen)           // Solid color
LEDPattern.rainbow(255, 255)             // Rainbow effect
LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kBlue)
```

### Custom Patterns (LEDCustomPattern class)

| Pattern | Description |
|---------|-------------|
| `stackedBlocks(color, blockSize, gapSize)` | Colored blocks with gaps |
| `stackedBlocks(color, blockSize, gapSize, gapColor)` | Blocks with colored gaps |
| `scrollingBlocks(color)` | Animated scrolling blocks (default 3 LED blocks, 2 LED gaps) |
| `scrollingBlocks(color, blockSize, gapSize)` | Configurable scrolling blocks |
| `solidIf(condition, trueColor, falseColor)` | Conditional solid color |
| `progressBar(progress, colorSupplier, bgColor)` | Fill bar based on 0.0-1.0 value |
| `urgentCountdown(remaining, total, threshold, color, bg, blinkPeriod)` | Countdown bar with urgent flashing mode in final seconds |
| `statusGradient(value, lowColor, highColor)` | Blend between colors |
| `countingBlocks(countSupplier, colorSupplier, blockSize, gapSize)` | Display N blocks |
| `bounceRipple(color)` | Expanding/contracting ripple with comet tail |
| `bounceRipple(color, rippleWidth, cyclesPerSecond)` | Configurable bounce ripple |
| `bounceRipple(color, rippleWidth, tailLength, cyclesPerSecond)` | Full control |
| `allianceColor()` | Red or blue based on current alliance |

### Pre-allocated Patterns in LEDController

| Pattern | Description |
|---------|-------------|
| `solidBlackPattern` | Solid black (off) |
| `solidYellowPattern` | Solid yellow |
| `solidRedPattern` | Solid red |
| `solidGreenPattern` | Solid green |
| `solidWhitePattern` | Solid white |
| `solidOrangeRedPattern` | Solid orange-red (for storage warning) |
| `blinkingYellowPattern` | Blinking yellow (0.5s period, for missing auto) |
| `bounceRippleYellowPattern` | Yellow bounce ripple (spindexing, not on target) |
| `bounceRippleGreenPattern` | Green bounce ripple (spindexing, on target) |
| `autoSelectionPattern` | Counting blocks in alliance color |
| `hubCountdownPattern` | Urgent countdown bar for match phase (flashes in final 10s) |

## Simulation

When running in simulation mode, LED state can be visualized using the **WPILib Simulation GUI**. The GUI automatically displays `AddressableLED` objects.

To view LEDs in simulation:
1. Run robot code in simulation mode
2. Open the Simulation GUI window
3. Look for the "Addressable LEDs" widget under Hardware menu
4. LED colors update in real-time as patterns are applied

No code changes are required - the built-in simulation support works automatically with the existing `AddressableLED` implementation in `LEDStrip`.

## Troubleshooting

### LEDs not lighting up
1. Check PWM port number in `LEDStrip`
2. Verify `LEDController.getInstance()` is called during robot init
3. Check LED strip power connections

### Animation going wrong direction
Adjust the `reversed` flag in the `Portion` definition. Set `true` if the physical wiring runs opposite to your logical direction.

### Pattern only appears on part of the strip
Check the start/end indices in your `Portion` definitions. Indices are inclusive.

### Colors look wrong
WPILib uses RGB order. Some LED strips expect GRB. Check your strip's data format.

### Same LEDs lighting up for different series
Some series intentionally overlap (e.g., `WARNING_STORAGE` overlaps with `Y_AXIS`). This is by design - warning indicators share space with other displays. Make sure your code doesn't apply conflicting patterns to overlapping series simultaneously.

### Warning indicators not showing
Warning indicators (`WARNING_ALLIANCE`, `WARNING_STORAGE`, `WARNING_COMPRESSOR`) are controlled by `displayAutoSelection()` and `displayCompressorState()`. Ensure these methods are being called in the appropriate robot modes.
