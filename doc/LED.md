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
| `LEDConstants` | Constants |

### Design Rationale

**Why enums for strips and series?**
Enums provide compile-time safety with no string parsing or map lookups at runtime. When you write `LEDSeries.ALL`, the compiler validates it exists.

**Why separate LEDStrip from LEDSeries?**
Physical layout (which PWM port, how many LEDs) changes rarely. Logical groupings (what we call "all" or "top left") are how code interacts with LEDs. Separating them means you can reorganize logical series without rewiring, or rewire without changing application code.

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
│  - Enum of logical series (TOP_LEFT_LARBOARD, ALL, etc.)    │
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
| MAIN  | 0        | 12        |

### Series Configuration

| Series | Portions | Description |
|--------|----------|-------------|
| `TOP_LEFT_LARBOARD` | LEDs 0-11 | Individual segment |
| `ALL` | LEDs 0-11 | All LEDs as one series |

## Quick Start

### Basic Usage

```java
// Apply a solid color to all LEDs
LEDSeries.ALL.applyPattern(LEDPattern.solid(Color.kGreen));

// Apply a built-in WPILib pattern
LEDSeries.ALL.applyPattern(LEDPattern.rainbow(255, 255));

// Apply a custom pattern
LEDSeries.ALL.applyPattern(LEDCustomPattern.scrollingBlocks(Color.kOrange));
```

### Using LEDController

The `LEDController` provides pre-built display methods:

```java
LEDController leds = LEDController.getInstance();

// Display auto selection (counting blocks in alliance color)
leds.displayAutoSelection();

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
  MAIN(0, 12);      // PWM port 0, 12 LEDs
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
  static final Portion TOP_AFT_LARBOARD = new Portion(LEDStrip.MAIN, 0, 11, false);
}
```

The `reversed` flag controls animation direction. Set `true` if LEDs are wired in the opposite direction from your logical "forward."

### Defining Series

Series reference portions from `P`:

```java
public enum LEDSeries implements LEDReader, LEDWriter {
  // Individual segments
  TOP_LEFT_LARBOARD(P.TOP_AFT_LARBOARD),

  // Composite series (could combine multiple portions)
  ALL(P.TOP_AFT_LARBOARD);
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
| `allianceColor()` | Red or blue based on current alliance |

### Pre-built Patterns in LEDController

| Pattern | Description |
|---------|-------------|
| `autoSelectionPattern` | Shows auto number as counting blocks in alliance color |
| `hubCountdownPattern` | Progress bar showing match phase remaining time |
| `solidBlackPattern` | Solid black (off) |
| `solidYellowPattern` | Solid yellow |

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
