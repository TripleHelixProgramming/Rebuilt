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
Physical layout (which PWM port, how many LEDs) changes rarely. Logical groupings (what we call "all" or "X_AXIS_FULL") are how code interacts with LEDs. Separating them means you can reorganize logical series without rewiring, or rewire without changing application code.

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

The strip is mounted as an L at the front-right corner of the frame. LEDs 0-11 run along the front edge toward the corner, so LED 11 is at the corner. LEDs 12-35 turn the corner and run back along the right side, so LED 35 is the farthest back.

```
Top view, front of the robot at the top:

   LED 0 ───── front edge ───── LED 11 ┐ front-right corner
                                       │ LED 12
                                       │
                                       │ right side
                                       │
                                       │ LED 35
```

### Series Configuration

The strip is divided into two primary segments by robot axis:

| Series | LEDs | Where | Description |
|--------|------|-------|-------------|
| `ALL` | 0-35 | Whole strip | All LEDs as one series |
| `Y_AXIS` | 0-11 | Front edge | Robot state (launcher and spindexer) while enabled |
| `X_AXIS_FULL` | 12-35 | Right side | Hub countdown when FMS attached |
| `X_AXIS_BODY` | 15-35 | Right side, behind the compressor LEDs | Hub countdown when FMS not attached |
| `AUTO_SELECTION` | 24-35 | Rear half of the right side | Auto selection counting blocks |

#### Warning Indicators

Reserved LEDs for status warnings. Each one overlaps a segment above:

| Series | LEDs | Where | Description |
|--------|------|-------|-------------|
| `WARNING_STORAGE` | 0-1 | Front edge, end farthest from the corner | USB storage low warning (orange-red when <2GB free) |
| `WARNING_COMPRESSOR` | 12-13 | Right side, at the corner | Compressor state (green when running) |
| `WARNING_ALLIANCE` | 22-23 | Right side, just ahead of the auto selection blocks | Alliance mismatch warning (yellow when DS/selected disagree) |

#### Pose-Seek Segments

The pose-seek groups sit on both sides of the corner:

| Series | LEDs | Where | Description |
|--------|------|-------|-------------|
| `POSE_Y` | 3-6 | Front edge | Left/right translation feedback |
| `POSE_ROTATION_Y` | 8-11 | Front edge, at the corner | Rotation feedback |
| `POSE_ROTATION_X` | 12-15 | Right side, at the corner | Rotation feedback |
| `POSE_X` | 17-20 | Right side | Forward/back translation feedback |
| `POSE_ROTATION` | 8-11, 12-15 | Both sides of the corner | Combined rotation (composite series) |

## Placing the Robot for Auto

While the robot is disabled with an auto selected, the LEDs show how to move the robot onto that auto's start pose. When auto begins, the routine resets the robot's pose to the start pose (`resetOdometry()`), so any placement error carries straight into the auto.

### Before placing

1. Set the alliance switch. Yellow on LEDs 22-23 means the switch disagrees with the driver station's alliance.
2. Turn the auto selector switch to the routine. LEDs 24-35, on the rear half of the right side, show the option number in the alliance color: one lit LED per number with a dark LED between, counting back from LED 24. Auto 3 lights LEDs 24, 26 and 28. Blinking yellow means no routine is set for that switch position. The pose lights stop updating while it blinks, so ignore them until a routine is selected.
3. Make sure the cameras have seen an AprilTag since the robot booted. The first accepted camera reading sets the robot's pose. Before that, the pose lights are computed from (0, 0) and mean nothing. If a dashboard shows the robot on the field, check that it is in the right place.
4. Orange-red on LEDs 0-1 means the USB log drive has less than 2 GB free.

### Reading the pose lights

Each group gives one direction along the edge it sits on. Green means move that edge toward the front-right corner, red means move it away from the corner, and white means that direction is within tolerance.

| Group | Where | Green | Red | White |
|-------|-------|-------|-----|-------|
| Left/right (LEDs 3-6) | Front edge | Move right | Move left | Within 6 cm |
| Forward/back (LEDs 17-20) | Right side | Move forward | Move backward | Within 5 cm |

Rotation uses two groups at the corner, LEDs 8-11 on the front edge and LEDs 12-15 on the right side. When the heading is off they show opposite colors:

- Side group green, front group red: turn counterclockwise, so the front swings to the robot's left. The right side moves forward, toward the corner, and the front edge moves left, away from it.
- Side group red, front group green: turn clockwise.
- Both white: within 3°.

Directions are the robot's own. "Forward" is toward the robot's front and "left" is the robot's left, whichever way it faces on the field.

### Procedure

1. Set the robot roughly on its start mark, facing roughly the right way.
2. Turn it until both rotation groups are white.
3. Move it until the left/right and forward/back groups are white.
4. Check that rotation is still white. When every group is white, the robot is on the start pose.

Notes:

- The lights are only as accurate as the robot's camera-based pose. A group that flickers between white and a color is at the edge of its tolerance.
- The lights only work if a camera can see a tag from the start position.
- If the auto selection changes after the robot is placed, check the lights again. Each routine has its own start pose.

## Display Functions

The `LEDController` provides high-level display methods for common robot states:

### displayAutoSelection()

Displays the currently selected autonomous routine during disabled mode.

- Shows counting blocks in alliance color (red or blue): one LED per block with a dark LED between blocks, starting at LED 24
- Number of blocks corresponds to the auto option number (1, 2, 3, etc.)
- Blinks yellow if no auto is selected
- Shows yellow warning on `WARNING_ALLIANCE` if there's a mismatch between driver station alliance and selected alliance
- Shows orange-red warning on `WARNING_STORAGE` if USB storage is below 2GB free

**Used on:** `AUTO_SELECTION` (LEDs 24-35)

### displayPoseSeek(currentPose, targetPose)

Shows how to move the robot onto a target pose. `Robot.disabledPeriodic()` calls it with the selected auto's start pose; [Placing the Robot for Auto](#placing-the-robot-for-auto) explains how the drive team reads it. Directions are robot-relative: `targetPose.minus(currentPose)` already expresses the target in the robot's frame, so the code must not rotate it again.

| Segment | Meaning |
|---------|---------|
| **X (POSE_X)** | Green = drive forward, Red = drive backward, White = correct |
| **Rotation (POSE_ROTATION_X/Y)** | POSE_ROTATION_X green and POSE_ROTATION_Y red = rotate CCW; POSE_ROTATION_X red and POSE_ROTATION_Y green = rotate CW; White = correct |
| **Y (POSE_Y)** | Red = move left, Green = move right, White = correct |

With the strip mounted at the front-right corner, every green means "move this edge toward the corner" and every red means "move it away". If the layout changes, keep the colors consistent with that rule. `LEDControllerTest` checks the colors at several headings.

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
LEDSeries.X_AXIS_BODY.applyPattern(LEDCustomPattern.scrollingBlocks(Color.kOrange));
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
