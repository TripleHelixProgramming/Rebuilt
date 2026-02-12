# Performance Tweaks

This document describes all changes made on the `performance-tweaks` branch. These optimizations reduce main loop latency by moving blocking I/O operations to background threads and eliminating redundant allocations/computations.

## Table of Contents

1. [SparkOdometryThread](#1-sparkodometrythread---background-thread-for-rev-spark-motor-controllers)
2. [CanandgyroThread](#2-canandgyrothread---background-thread-for-redux-canandgyro)
3. [VisionThread](#3-visionthread---background-thread-for-photonvision)
4. [Batched Phoenix 6 Signal Refreshes](#4-batched-phoenix-6-signal-refreshes-in-drive)
5. [Pre-computed Constants in Vision](#5-pre-computed-constants-in-vision)
6. [Re-use Collections in VisionIOPhotonVision](#6-re-use-collections-in-visioniophotonvision)
7. [Pre-computed wheelRadiusMeters](#7-pre-computed-wheelradiusmeters-in-driveconstants)
8. [IntakeRollerIOTalonFX Fix](#8-intakerolleriotalonfx-fix)
9. [SmartDashboard.putData Optimization](#9-smartdashboardputdata-moved-to-robot-constructor)

---

## 1. SparkOdometryThread - Background Thread for REV Spark Motor Controllers

**New file:** `src/main/java/frc/robot/util/SparkOdometryThread.java`

### How It Works

This creates a singleton background thread using WPILib's `Notifier` that runs at 100Hz (10ms period). REV Spark motor controllers (SparkMax/SparkFlex) use blocking CAN calls - when you call `encoder.getPosition()`, the code blocks until the CAN frame returns. This can add several milliseconds of latency per device.

The thread maintains a list of registered `SparkInputs` objects. Each `SparkInputs` wraps a SparkBase device and caches:

- Position (from encoder)
- Velocity (from encoder)
- Applied voltage (computed as `getAppliedOutput() * getBusVoltage()`)
- Output current
- Connection status (based on `REVLibError.kOk` checks)
- Timestamp

The `update()` method reads all values from the device, checks for errors after each read, and only updates the cached values if all reads succeeded. This ensures atomic updates - you never see partially updated data.

**Thread safety:** All cached fields are `volatile`, ensuring visibility across threads. The `ReentrantLock` protects the registration list from concurrent modification.

### Changes to Subsystems

**HoodIOSpark.java:**

Before:
```java
ifOk(hoodSpark, hoodEncoder::getPosition, (value) -> inputs.position = new Rotation2d(value));
ifOk(hoodSpark, hoodEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
ifOk(hoodSpark, new DoubleSupplier[] {hoodSpark::getAppliedOutput, hoodSpark::getBusVoltage},
    (values) -> inputs.appliedVolts = values[0] * values[1]);
```

After:
```java
inputs.position = new Rotation2d(sparkInputs.getPosition());
inputs.velocityRadPerSec = sparkInputs.getVelocity();
inputs.appliedVolts = sparkInputs.getAppliedVolts();
inputs.currentAmps = sparkInputs.getOutputCurrent();
inputs.connected = connectedDebounce.calculate(sparkInputs.isConnected());
```

**Why behavior is unchanged:** The values read are identical - position, velocity, applied volts, current, and connection status. The only difference is *when* the CAN read happens (background thread vs main loop). The same `ifOk` pattern that validated `sparkStickyFault` is replaced by `SparkInputs.isConnected()`, which performs the same error checking inside `update()`.

**TurretIOSpark.java:** Same pattern as HoodIOSpark. Note the absolute encoder (DIO) is still read directly since DIO reads are already fast.

---

## 2. CanandgyroThread - Background Thread for Redux Canandgyro

**New file:** `src/main/java/frc/robot/util/CanandgyroThread.java`

### How It Works

Same pattern as SparkOdometryThread, but for the Redux Canandgyro. Runs at 100Hz and caches:

- `isConnected()`
- `isCalibrating()`
- `getYaw()`
- `getAngularVelocityYaw()`

### Changes to GyroIOBoron

**GyroIOBoron.java:**

Before:
```java
inputs.connected = canandgyro.isConnected();
inputs.calibrated = !canandgyro.isCalibrating();
inputs.yawPosition = Rotation2d.fromRotations(canandgyro.getYaw());
inputs.yawVelocityRadPerSec = Units.rotationsToRadians(canandgyro.getAngularVelocityYaw());
```

After:
```java
inputs.connected = gyroInputs.isConnected();
inputs.calibrated = !gyroInputs.isCalibrating();
inputs.yawPosition = Rotation2d.fromRotations(gyroInputs.getYaw());
inputs.yawVelocityRadPerSec = Units.rotationsToRadians(gyroInputs.getAngularVelocityYaw());
```

**Why behavior is unchanged:** The exact same values are read and converted. The odometry queue registration with `PhoenixOdometryThread` is untouched - high-frequency odometry samples still flow through that existing mechanism.

---

## 3. VisionThread - Background Thread for PhotonVision

**New file:** `src/main/java/frc/robot/util/VisionThread.java`

### How It Works

Runs at 50Hz (20ms period). PhotonVision uses NetworkTables, which can have variable latency. Each registered `VisionIO` is polled on the background thread.

Key design: Uses an **immutable snapshot** pattern. The background thread creates a new `VisionIOInputsSnapshot` object each update, containing cloned arrays. The main thread reads a volatile reference to this snapshot. This avoids any possibility of reading partially-updated data.

```java
snapshot = new VisionIOInputsSnapshot(
    workingInputs.connected,
    workingInputs.latestTargetObservation,
    workingInputs.poseObservations.clone(),  // Clone to ensure immutability
    workingInputs.tagIds.clone());
```

### Changes to Vision Subsystem

**Vision.java:**

Before:
```java
for (int i = 0; i < io.length; i++) {
  io[i].updateInputs(inputs[i]);
  Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
}
```

After:
```java
for (int i = 0; i < io.length; i++) {
  visionInputs[i].getSnapshot().copyTo(inputs[i]);
  Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
}
```

**Why behavior is unchanged:** The `copyTo()` method copies all four fields (`connected`, `latestTargetObservation`, `poseObservations`, `tagIds`) into the AdvantageKit input object. All downstream processing (pose estimation, scoring, etc.) operates on these inputs identically.

---

## 4. Batched Phoenix 6 Signal Refreshes in Drive

### How It Works

Phoenix 6 (CTRE TalonFX) provides `BaseStatusSignal.refreshAll()` which can batch multiple signal reads into fewer CAN frames. Previously, each module called `refreshSignals()` individually.

**ModuleIOTalonFX.java:**

Added a cached array of all 9 signals per module:
```java
allSignals = new BaseStatusSignal[] {
    drivePosition, driveVelocity, driveAppliedVolts, driveCurrent,
    turnPosition, turnVelocity, turnAppliedVolts, turnCurrent,
    turnAbsolutePosition
};
```

Added `getStatusSignals()` method to expose these for batching.

**Drive.java:**

Before:
```java
for (var module : modules) {
  module.refreshSignals();
}
```

After:
```java
// Collect signals from all 4 modules into one array (done in constructor)
if (allModuleSignals.length > 0) {
  BaseStatusSignal.refreshAll(allModuleSignals);  // Single batched CAN call
}
```

**Why behavior is unchanged:** The same signals are refreshed with the same data. The only difference is that 4 separate `refreshAll()` calls (one per module, each with 9 signals = 36 individual signal refreshes) become 1 `refreshAll()` call with all 36 signals. The Phoenix 6 library optimizes this internally.

---

## 5. Pre-computed Constants in Vision

### How It Works

WPILib's Units API (`Distance`, `Angle`) calls `.in(Units.X)` to convert to raw doubles. This involves virtual method calls and some overhead. For constants used every loop iteration, we pre-compute the double value once.

**VisionConstants.java:**

```java
public static Distance tagDistanceTolerance = Meters.of(4.0);
public static final double tagDistanceToleranceMeters = tagDistanceTolerance.in(Meters);  // NEW

public static Distance elevationTolerance = Meters.of(0.25);
public static final double elevationToleranceMeters = elevationTolerance.in(Meters);  // NEW
// ... same for rollToleranceRadians, pitchToleranceRadians, field dimensions
```

**Vision.java:**

The `VisionTest` enum methods now use pre-computed values:

Before:
```java
normalizedSigmoid(Math.abs(observation.pose().getRotation().getY()), pitchTolerance.in(Radians), 1.0);
```

After:
```java
normalizedSigmoid(Math.abs(observation.pose().getRotation().getY()), pitchToleranceRadians, 1.0);
```

**Arena boundary optimization:**

Before (called per observation):
```java
var cornerA = new Translation2d(minRobotWidth.div(2.0), minRobotWidth.div(2.0));
var cornerB = new Translation2d(Field.field_x_len, Field.field_y_len).minus(cornerA);
var arena = new Rectangle2d(cornerA, cornerB);
boolean pass = arena.contains(observation.pose().toPose2d().getTranslation());
```

After (computed once in static initializer):
```java
private static final Rectangle2d arenaRectangle;
static {
  double halfWidth = minRobotWidthHalfMeters;
  Translation2d cornerA = new Translation2d(halfWidth, halfWidth);
  Translation2d cornerB = new Translation2d(fieldXLenMeters - halfWidth, fieldYLenMeters - halfWidth);
  arenaRectangle = new Rectangle2d(cornerA, cornerB);
}
// In test method:
boolean pass = arenaRectangle.contains(observation.pose().toPose2d().getTranslation());
```

**Why behavior is unchanged:** `pitchTolerance.in(Radians)` returns the same double as `pitchToleranceRadians` - it's just computed at class load time instead of every call. The rectangle dimensions are identical.

---

## 6. Re-use Collections in VisionIOPhotonVision

**VisionIOPhotonVision.java:**

Before (allocates new collections every call):
```java
Set<Short> tagIds = new HashSet<>();
List<PoseObservation> poseObservations = new LinkedList<>();
```

After (reuses instance fields):
```java
// Instance fields:
private final Set<Short> tagIds = new HashSet<>();
private final List<PoseObservation> poseObservations = new ArrayList<>();

// In updateInputs():
tagIds.clear();
poseObservations.clear();
```

**Why behavior is unchanged:** The collections are cleared at the start of each call, then populated identically. The logic is unchanged; only the allocation strategy differs. Also changed `LinkedList` to `ArrayList` which has better cache locality for iteration.

---

## 7. Pre-computed wheelRadiusMeters in DriveConstants

**DriveConstants.java:**
```java
public static final Distance wheelRadius = Inches.of(2);
public static final double wheelRadiusMeters = wheelRadius.in(Meters);  // NEW
```

**Module.java:**

Before:
```java
double positionMeters = inputs.odometryDrivePositionsRad[i] * wheelRadius.in(Meters);
```

After:
```java
double positionMeters = inputs.odometryDrivePositionsRad[i] * wheelRadiusMeters;
```

**Why behavior is unchanged:** The same conversion factor is used; it's simply pre-computed at class load time.

---

## 8. IntakeRollerIOTalonFX Fix

**IntakeRollerIOTalonFX.java:**

Before (incorrect - `setUpdateFrequencyForAll` changes frequency, doesn't refresh):
```java
BaseStatusSignal.setUpdateFrequencyForAll(50.0, intakeVelocity, intakeAppliedVolts, intakeCurrent).isOK();
inputs.appliedVolts = intakeAppliedVolts.getValueAsDouble();
```

After (correct - actually refreshes the signals):
```java
var status = BaseStatusSignal.refreshAll(intakeVelocity, intakeAppliedVolts, intakeCurrent);
if (!status.isOK()) {
  return;  // Don't update inputs with stale data
}
inputs.appliedVolts = intakeAppliedVolts.getValueAsDouble();
```

**Why behavior is unchanged (or improved):** The original code was calling `setUpdateFrequencyForAll` every loop, which is wasteful (frequency should only be set once in the constructor). It then read signal values that may not have been refreshed. The new code properly refreshes signals before reading them. This is a bug fix that makes behavior *more* correct.

---

## 9. SmartDashboard.putData Moved to Robot Constructor

**Robot.java:**

`SmartDashboard.putData()` calls were moved from `robotPeriodic()` to the constructor. These calls register objects with SmartDashboard for Shuffleboard/Glass - they only need to be called once.

**Why behavior is unchanged:** The same subsystems are registered. Calling `putData` repeatedly with the same key just overwrites the same entry.

---

## Summary

All changes fall into these categories:

| Category | Changes | Benefit |
|----------|---------|---------|
| **Background threading** | SparkOdometryThread, CanandgyroThread, VisionThread | Move blocking I/O to background threads. Main loop reads cached values instead of blocking. |
| **Batching** | Drive signal refresh | Combine multiple CAN operations into one call that the library can optimize. |
| **Allocation avoidance** | Vision collections, arena Rectangle2d | Reuse objects instead of allocating new ones every loop. Reduces GC pressure. |
| **Pre-computation** | wheelRadiusMeters, tolerance constants | Compute unit conversions once at startup instead of every call. |
| **Bug fix** | IntakeRollerIOTalonFX | Replace incorrect `setUpdateFrequencyForAll` with correct `refreshAll`. |

None of these changes alter the logical behavior of the robot code - they all produce identical values through more efficient means.
