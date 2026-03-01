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
│  - Enum of logical series (Y_AXIS, X_AXIS, ALL, etc.)       │
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
| MAIN  | 0        | 24        |

### Series Configuration

The strip is divided into two primary segments by robot axis:

| Series | LEDs | Description |
|--------|------|-------------|
| `ALL` | 0-23 | All LEDs as one series |
| `Y_AXIS` | 0-11 | Along robot Y axis (pose-seek, hub countdown) |
| `X_AXIS` | 12-23 | Along robot X axis (auto selection, robot state) |

#### Pose-Seek Segments (within Y_AXIS)

The Y_AXIS segment is further divided for pose-seek feedback:

```
[Y_LEFT] [ROT_LEFT] [X_CENTER] [ROT_RIGHT] [Y_RIGHT]
[0-1]    [2-3]      [4-7]      [8-9]       [10-11]
```

| Series | LEDs | Description |
|--------|------|-------------|
| `POSE_Y_LEFT` | 0-1 | Left side Y translation feedback |
| `POSE_ROTATION_LEFT` | 2-3 | Left rotation feedback |
| `POSE_X_CENTER` | 4-7 | X translation feedback (center) |
| `POSE_ROTATION_RIGHT` | 8-9 | Right rotation feedback |
| `POSE_Y_RIGHT` | 10-11 | Right side Y translation feedback |
| `POSE_ROTATION` | 2-3, 8-9 | Combined rotation (both sides) |

## Display Functions

The `LEDController` provides high-level display methods for common robot states:

### displayAutoSelection()

Displays the currently selected autonomous routine during disabled mode.

- Shows counting blocks in alliance color (red or blue)
- Number of blocks corresponds to the auto option number (1, 2, 3, etc.)
- Blinks yellow if no auto is selected
- Sets the last LED to yellow if there's a mismatch between driver station alliance and selected alliance

**Used on:** `X_AXIS` (LEDs 12-23)

### displayPoseSeek(currentPose, targetPose)

Provides visual feedback for manually aligning the robot to a target pose. Useful during disabled mode to help drivers position the robot for autonomous.

**Layout:** `[Y_LEFT][ROT_LEFT][X_CENTER][ROT_RIGHT][Y_RIGHT]`

| Segment | Meaning |
|---------|---------|
| **X (center)** | Green = drive forward, Red = drive backward, White = correct |
| **Rotation (inner)** | Magenta = rotate CCW, Cyan = rotate CW, White = correct |
| **Y (ends)** | Green on side to strafe toward, Red on opposite side, White = correct |

Tolerances are defined in `LEDConstants`:
- X: 5 cm
- Y: 6 cm
- Heading: 3°

**Used on:** `Y_AXIS` segments (LEDs 0-11)

### displayRobotState(isOnTarget, isSpindexing)

Shows launcher and spindexer state during teleop and autonomous.

| State | Color | Animation |
|-------|-------|-----------|
| Not on target, spindexer inactive | Yellow | Solid |
| Not on target, spindexer active | Yellow | Bounce ripple |
| On target, spindexer inactive | Green | Solid |
| On target, spindexer active | Green | Bounce ripple |

**Used on:** `X_AXIS` (LEDs 12-23)

### displayHubCountdown()

Shows remaining time in the current match phase as a progress bar.

- Bar fills in alliance color (red or blue based on hub state)
- Empties as time runs out

**Used on:** `Y_AXIS` (LEDs 0-11)

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
leds.displayAutoSelection();

// Display pose-seek feedback
leds.displayPoseSeek(drive.getPose(), targetPose);

// Display robot state
leds.displayRobotState(() -> launcher.isOnTarget(), () -> feeder.isSpinning());

// Display hub countdown (progress bar)
leds.displayHubCountdown();

// Clear all LEDs
leds.clear();
```

## Physical Layout Configuration

### Defining Strips

In `LEDStrip.java`, define each physical LED strip:

```java
public enum LEDStrip {
  MAIN(0, 24);      // PWM port 0, 24 LEDs
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
  static final Portion ALL = new Portion(LEDStrip.MAIN, 0, 23, false);
  static final Portion Y_AXIS = new Portion(LEDStrip.MAIN, 0, 11, false);
  static final Portion X_AXIS = new Portion(LEDStrip.MAIN, 12, 23, false);
  // Pose-seek portions
  static final Portion POSE_Y_LEFT = new Portion(LEDStrip.MAIN, 0, 1, false);
  static final Portion POSE_ROT_LEFT = new Portion(LEDStrip.MAIN, 2, 3, false);
  static final Portion POSE_X_CENTER = new Portion(LEDStrip.MAIN, 4, 7, false);
  static final Portion POSE_ROT_RIGHT = new Portion(LEDStrip.MAIN, 8, 9, false);
  static final Portion POSE_Y_RIGHT = new Portion(LEDStrip.MAIN, 10, 11, false);
}
```

The `reversed` flag controls animation direction. Set `true` if LEDs are wired in the opposite direction from your logical "forward."

### Defining Series

Series reference portions from `P`:

```java
public enum LEDSeries implements LEDReader, LEDWriter {
  ALL(P.ALL),
  Y_AXIS(P.Y_AXIS),
  X_AXIS(P.X_AXIS),
  POSE_Y_LEFT(P.POSE_Y_LEFT),
  POSE_ROTATION(P.POSE_ROT_LEFT, P.POSE_ROT_RIGHT),  // Composite series
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
| `solidGreenPattern` | Solid green |
| `bounceRippleYellowPattern` | Yellow bounce ripple (spindexing, not on target) |
| `bounceRippleGreenPattern` | Green bounce ripple (spindexing, on target) |
| `autoSelectionPattern` | Counting blocks in alliance color |
| `hubCountdownPattern` | Progress bar for match phase |

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
Verify that portion definitions in the `P` class have non-overlapping indices for the segments you're using independently.
