# LED System

This document explains the LED subsystem architecture and how to use it.

## Overview

The LED system provides a flexible way to control addressable LED strips on the robot. It separates concerns into distinct layers:

| Class | Purpose |
|-------|---------|
| `LEDStrip` | Physical hardware (PWM ports, buffers) |
| `LEDGroup` | Logical groupings (segments, composite groups) |
| `LEDCustomPattern` | Custom pattern factories |
| `LEDController` | Subsystem with command factories |
| `LEDConstants` | Tunable constants |

### Design Rationale

**Why enums for strips and groups?**
Enums provide compile-time safety with no string parsing or map lookups at runtime. When you write `LEDGroup.LEFT_TOP`, the compiler validates it exists.

**Why separate LEDStrip from LEDGroup?**
Physical layout (which PWM port, how many LEDs) changes rarely. Logical groupings (what we call "left side" or "top") are how code interacts with LEDs. Separating them means you can reorganize logical groups without rewiring, or rewire without changing application code.

**Why implement LEDReader/LEDWriter?**
Groups can span multiple physical portions (e.g., `ALL` spans 6 portions). By implementing WPILib's `LEDReader` and `LEDWriter` interfaces, groups present a unified virtual buffer. Patterns see one contiguous strip of LEDs, even if the group spans multiple physical segments.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      LEDController                          │
│  - Singleton subsystem                                      │
│  - Calls LEDStrip.updateAll() every periodic cycle          │
│  - Provides command factories                               │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                        LEDGroup                             │
│  - Enum of logical groups (LEFT, RIGHT, TOP, ALL, etc.)     │
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

## Quick Start

### Basic Usage

```java
// Apply a solid color to a group
LEDGroup.ALL.applyPattern(LEDPattern.solid(Color.kGreen));

// Apply a built-in WPILib pattern
LEDGroup.LEFT.applyPattern(LEDPattern.rainbow(255, 255));

// Apply a custom pattern
LEDGroup.RIGHT.applyPattern(LEDCustomPattern.scrollingBlocks(Color.kOrange));
```

### Creating Commands

Use `LEDController` to create commands that run patterns:

```java
LEDController leds = LEDController.getInstance();

// Static pattern
Command solidGreen = leds.runPattern(LEDPattern.solid(Color.kGreen), LEDGroup.ALL);

// Dynamic pattern (re-evaluated each cycle)
Command allianceColor = leds.runPattern(
    () -> LEDPattern.solid(isRedAlliance() ? Color.kRed : Color.kBlue),
    LEDGroup.ALL
);
```

### Binding to Triggers

```java
// Show green while a button is held
joystick.button(1).whileTrue(
    leds.runPattern(LEDPattern.solid(Color.kGreen), LEDGroup.ALL)
);

// Set default pattern
leds.setDefaultCommand(
    leds.runPattern(LEDCustomPattern.allianceColor(this::isRedAlliance), LEDGroup.ALL)
);
```

## Physical Layout Configuration

### Defining Strips

In `LEDStrip.java`, define each physical LED strip:

```java
public enum LEDStrip {
  MAIN(9, 40);      // PWM port 9, 40 LEDs
  // FRONT(8, 60),  // Uncomment to add more strips
  // BACK(7, 30);
  ...
}
```

### Defining Portions

In `LEDGroup.java`, the `P` class defines portions - the single source of truth for physical layout:

```java
private static final class P {
  // (strip, startIndex, endIndex, reversed)
  static final Portion RIGHT_BOTTOM = new Portion(LEDStrip.MAIN, 0, 7, true);
  static final Portion RIGHT_MIDDLE = new Portion(LEDStrip.MAIN, 8, 11, true);
  static final Portion RIGHT_TOP = new Portion(LEDStrip.MAIN, 12, 19, true);
  static final Portion LEFT_TOP = new Portion(LEDStrip.MAIN, 20, 27, false);
  static final Portion LEFT_MIDDLE = new Portion(LEDStrip.MAIN, 28, 31, false);
  static final Portion LEFT_BOTTOM = new Portion(LEDStrip.MAIN, 32, 39, false);
}
```

The `reversed` flag controls animation direction. Set `true` if LEDs are wired in the opposite direction from your logical "forward."

### Defining Groups

Groups reference portions from `P`:

```java
public enum LEDGroup implements LEDReader, LEDWriter {
  // Individual segments
  RIGHT_BOTTOM(P.RIGHT_BOTTOM),
  LEFT_TOP(P.LEFT_TOP),

  // Composite groups (multiple portions)
  LEFT(P.LEFT_TOP, P.LEFT_MIDDLE, P.LEFT_BOTTOM),
  RIGHT(P.RIGHT_BOTTOM, P.RIGHT_MIDDLE, P.RIGHT_TOP),
  TOP(P.LEFT_TOP, P.RIGHT_TOP),
  ALL(P.RIGHT_BOTTOM, P.RIGHT_MIDDLE, P.RIGHT_TOP, P.LEFT_TOP, P.LEFT_MIDDLE, P.LEFT_BOTTOM);
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
    Color color,
    Color backgroundColor) {
  return (reader, writer) -> {
    int length = reader.getLength();
    int filled = (int) (length * progressSupplier.get());
    for (int i = 0; i < length; i++) {
      writer.setLED(i, i < filled ? color : backgroundColor);
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
| `scrollingBlocks(color)` | Animated scrolling blocks |
| `solidIf(condition, trueColor, falseColor)` | Conditional solid color |
| `progressBar(progress, color, bgColor)` | Fill bar based on 0.0-1.0 value |
| `statusGradient(value, lowColor, highColor)` | Blend between colors |
| `countingBlocks(count, color, blockSize, gapSize)` | Display N blocks |
| `allianceColor(isRedAlliance)` | Red or blue based on alliance |

## Typical Use Cases

### Intake/Outtake Animation

```java
// Scroll blocks inward during intake
Command intakeLEDs = leds.runPattern(
    LEDCustomPattern.scrollingBlocks(Color.kOrange),
    LEDGroup.ALL
);

// Scroll blocks outward during outtake (reverse the pattern)
Command outtakeLEDs = leds.runPattern(
    LEDCustomPattern.scrollingBlocks(Color.kOrange).reversed(),
    LEDGroup.ALL
);
```

### Showing Robot State

```java
// Show arm position as progress bar
Command armProgress = leds.runPattern(
    LEDCustomPattern.progressBar(arm::getPositionRatio, Color.kGreen, Color.kBlack),
    LEDGroup.LEFT
);

// Flash when at setpoint
Command atSetpoint = leds.runPattern(
    LEDPattern.solid(Color.kGreen).blink(Units.Seconds.of(0.1)),
    LEDGroup.ALL
);
```

### Auto Selection Display

```java
// Show auto number as counted blocks in alliance color
Command autoDisplay = leds.runAutoSelection(
    autoChooser::getSelectedAutoNumber,
    () -> isRedAlliance() ? Color.kRed : Color.kBlue
);
```

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
