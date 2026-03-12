# Defensive Guards: Design Philosophy and Implementation

This document explains the defensive guard changes made to the codebase to prevent NaN values, division by zero errors, and other numerical instabilities.

## Overview

Numerical errors like NaN (Not a Number) and Infinity can silently propagate through calculations, causing erratic robot behavior. This document describes the guards added or refined in the recent code review.

## Changes Summary

### New Guards Added

These guards did not exist before and were added to protect against runtime edge cases:

| File | Guard | Risk Mitigated |
|------|-------|----------------|
| DriveCommands.java | FF characterization denominator | Division by zero when all velocity samples identical |
| DriveCommands.java | Wheel radius wheelDelta check | Division by zero if wheels don't rotate |
| DriveCommands.java | Wheel radius gyroDelta check | Division by zero if gyro doesn't register rotation |
| PathCommands.java | Zero-length vector check | NaN from `getAngle()` on zero vector |
| VisionIOPhotonVision.java | Target count check | Division by zero with empty targets list |
| VisionIOPhotonVisionSim.java | Null pose check | NPE during subsystem initialization |
| Launcher.java | v_0r < 1e-6 check | Division by zero in subsequent calculation |

### Existing Guards Refined

These guards already existed but were refined with better thresholds and documentation:

| File | Before | After | Why Changed |
|------|--------|-------|-------------|
| Launcher.java | `denominator <= 0` | `denominator < 1e-6` | Also catches near-zero values causing numerical instability |
| Launcher.java | `discriminant < 0` | `discriminant < 1e-6` | Safety margin for floating-point edge cases |

---

## Guard Philosophy

### When to Add Guards

Guards are appropriate for **legitimate runtime conditions**:

1. **User-collected data** - Characterization samples may have insufficient variation
2. **Geometric edge cases** - Robot at exact target position, zero-length vectors
3. **External inputs** - Vision data may have zero targets
4. **Timing edge cases** - Subsystems may not be ready during initialization

### When NOT to Add Guards

Guards should NOT hide bugs or impossible conditions:

1. **Mathematical impossibilities** - `getNorm()` cannot return negative
2. **Configuration errors** - Should fail immediately during testing, not silently return defaults
3. **Invalid internal state** - Should be fixed at source, not masked downstream

**Example**: The `flywheelSetpointfromBallistics()` function takes input from `getNorm()` which mathematically cannot be negative. Adding a guard for `ballistics < 0` would be protecting against an impossible condition and was not added.

---

## Detailed Changes

### DriveCommands.java - FF Characterization (NEW)

**Before:**
```java
double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
```

**After:**
```java
double denominator = n * sumX2 - sumX * sumX;
if (Math.abs(denominator) < 1e-6) {
    System.out.println("ERROR: Insufficient velocity variation in samples.");
    return;
}
double kS = (sumY * sumX2 - sumX * sumXY) / denominator;
double kV = (n * sumXY - sumX * sumY) / denominator;
```

**Impact:** Prevents NaN when user collects samples at constant velocity.

### DriveCommands.java - Wheel Radius (NEW)

**Before:**
```java
double wheelRadius = (state.gyroDelta * driveBaseRadius.in(Meters)) / wheelDelta;
```

**After:**
```java
if (wheelDelta < 1e-6) {
    System.out.println("ERROR: Wheels did not rotate sufficiently.");
    return;
}
if (state.gyroDelta < 1e-6) {
    System.out.println("ERROR: Robot did not rotate sufficiently.");
    return;
}
double wheelRadius = (state.gyroDelta * driveBaseRadius.in(Meters)) / wheelDelta;
```

**Impact:** Prevents Infinity/NaN when characterization routine fails to move robot.

### PathCommands.java - Zero Vector (NEW)

**Before:**
```java
var heading = targetPoint.minus(initialPose.getTranslation()).getAngle();
```

**After:**
```java
Translation2d delta = targetPoint.minus(initialPose.getTranslation());
Rotation2d heading = delta.getNorm() < 1e-6 ? initialPose.getRotation() : delta.getAngle();
```

**Impact:** Prevents NaN when robot is already at target position.

### VisionIOPhotonVision.java - Division Guard (NEW)

**Before:**
```java
totalTagDistance / result.targets.size(), // Average tag distance
```

**After:**
```java
int targetCount = result.targets.size();
double avgTagDistance = targetCount > 0 ? totalTagDistance / targetCount : 0.0;
// ...
avgTagDistance, // Average tag distance
```

**Impact:** Prevents Infinity if targets list is unexpectedly empty.

### VisionIOPhotonVisionSim.java - Null Check (NEW)

**Before:**
```java
visionSim.update(poseSupplier.get());
```

**After:**
```java
Pose2d robotPose = poseSupplier.get();
if (robotPose != null) {
    visionSim.update(robotPose);
}
```

**Impact:** Prevents NPE during early initialization before drive subsystem is ready.

### Launcher.java - Ballistics Denominator (REFINED)

**Before:**
```java
if (denominator <= 0) {
    Logger.recordOutput("Launcher/" + key + "/Reachable", false);
    // log(d, v0, key);
    return v0nominalLast;
}
```

**After:**
```java
// Guard: denominator <= 0 means target is unreachable with this impact angle (would require
// negative velocity or infinite speed). Using < 1e-6 threshold also catches near-zero values
// that would cause numerical instability in sqrt(g / denominator).
if (denominator < 1e-6) {
    Logger.recordOutput("Launcher/" + key + "/Reachable", false);
    // log(d, v0, key);
    return v0nominalLast;
}
```

**Change:** Threshold changed from `<= 0` to `< 1e-6` and explanatory comment added above the guard.

### Launcher.java - v_0r Guard (NEW)

**After the denominator check:**
```java
double v_0r = dr * Math.sqrt(g / denominator);
if (v_0r < 1e-6) {
    Logger.recordOutput("Launcher/" + key + "/Reachable", false);
    return v0nominalLast;
}
```

**Impact:** Prevents division by near-zero v_0r in subsequent calculation.

### Launcher.java - Discriminant (REFINED)

**Before:**
```java
if (discriminant < 0) {
    // Unreachable target at this speed
    Logger.recordOutput("Launcher/" + key + "/Reachable", false);
    // log(d, v0, key);
    return v0replannedLast;
}
```

**After:**
```java
// Guard: discriminant < 0 means target is beyond maximum range for current flywheel speed.
// Using < 1e-6 threshold adds safety margin against sqrt of tiny negative values from
// floating-point errors at the edge of reachable range.
if (discriminant < 1e-6) {
    // Unreachable target at this speed
    Logger.recordOutput("Launcher/" + key + "/Reachable", false);
    // log(d, v0, key);
    return v0replannedLast;
}
```

**Change:** Threshold changed from `< 0` to `< 1e-6` and explanatory comment added above the guard.

---

## Epsilon Value: 1e-6

The value `1e-6` is used consistently throughout for:
- **Practical safety margin** - Large enough to catch floating-point rounding errors
- **Educational clarity** - Explicit value is easier to understand than an abstraction
- **Consistency** - Same threshold everywhere simplifies reasoning about interactions

---

## Log Analysis

To check for issues in match logs, open `.wpilog` files in AdvantageScope and look for:
- `Launcher/*/Reachable` = false entries (ballistics guard activated)
- Any NaN or Infinity values in motor setpoints
- Discontinuities in flywheel velocity commands
