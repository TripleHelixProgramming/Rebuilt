# Performance Tweaks 3

This document describes the performance optimization work done on the `performance-tweaks-3` branch, building on the foundation established in the `performance-tweaks-2` branch.

## Executive Summary

This branch ports the profiling infrastructure and CAN optimization changes from `performance-tweaks-2` to the current `main` branch. The goal is to identify and reduce loop overruns caused by blocking I/O operations.

---

## Changes Implemented

### 1. Profiling Infrastructure

Timing instrumentation has been added throughout the codebase to identify bottlenecks empirically. Each subsystem reports timing breakdowns when thresholds are exceeded:

| File | Method | Threshold | Metrics |
|------|--------|-----------|---------|
| `Robot.java` | `robotPeriodic()` | >20ms | scheduler, gameState |
| `Drive.java` | `periodic()` | >5ms | lock, gyroUpdate, gyroLog, modules, disabled, odometry |
| `Module.java` | `periodic()` | >2ms | updateInputs, log, rest |
| `Vision.java` | `periodic()` | >5ms | snapshot, processInputs, cameraLoop, consumer, summaryLog |
| `Intake.java` | `periodic()` | >2ms | update, log |
| `Feeder.java` | `periodic()` | >2ms | spindexer, kicker, spindexerLog, kickerLog |
| `Launcher.java` | `periodic()` | >3ms | update, log, aimLog, ballistics |
| `Launcher.java` | `aim()` | >500μs | v0nom, baseSpeeds, v0replan, setPos, rest |
| `Hopper.java` | `periodic()` | >2ms | update, log |

**Example profiling output:**
```
[Drive] lock=0ms gyroUpdate=2ms gyroLog=1ms modules=105ms disabled=69ms odometry=144ms total=322ms
[Robot] scheduler=350ms gameState=5ms total=355ms
```

### 2. Removed Blocking CAN Refreshes

The codebase was calling `BaseStatusSignal.refreshAll()` synchronously in the main loop, which blocks until CAN responses are received. This has been replaced with relying on Phoenix 6's automatic signal updates.

**Files Changed:**
- `Drive.java` - Removed batched refresh of all module signals
- `Module.java` - Removed `refreshSignals()` and `getStatusSignals()` methods
- `ModuleIO.java` - Removed interface methods for signal refresh
- `ModuleIOTalonFX.java` - Removed `allSignals` array and refresh methods
- `IntakeRollerIOTalonFX.java` - Changed from `refreshAll()` to checking `.getStatus().isOK()`
- `FlywheelIOTalonFX.java` - Changed from `refreshAll()` to checking `.getStatus().isOK()`

**Why this works:**
- Phoenix 6 automatically updates status signals at the configured frequency (50Hz for telemetry, 250Hz for odometry)
- Position signals for odometry are handled by `PhoenixOdometryThread` at 250Hz in the background
- The main loop reads cached values (memory read in nanoseconds) instead of blocking on CAN (milliseconds)

---

## Background Threading (Pre-existing)

The codebase already implements background threading for sensor reads:

- **PhoenixOdometryThread** - Reads TalonFX position signals at 250Hz
- **SparkOdometryThread** - Reads SparkMax telemetry in background
- **VisionThread** - Processes camera data asynchronously
- **CanandgyroThread** - Reads gyro data in background

### 3. Deferred Logging in Launcher

The `aim()` method is called frequently and was experiencing significant overhead from `Logger.recordOutput()` calls. The fix is to cache values during `aim()` and log them in `periodic()` instead.

**Implementation:**
- Added cached fields: `cachedBaseSpeeds`, `cachedFlywheelVelocityTooLow`, `cachedActualD`, `cachedActualV`
- `hasCachedAimData` flag indicates when cached data is available
- `aim()` populates the cache instead of calling Logger directly
- `periodic()` calls `logCachedAimData()` to flush the cache

**Why this helps:**
- `Logger.recordOutput()` can take 11-28ms per call due to serialization overhead
- Moving logging out of `aim()` keeps the hot path fast
- Logging happens once per robot loop instead of during time-critical aiming calculations

### 4. Zero-Velocity Guard

Added protection against `Translation2d.getAngle()` errors when velocity vectors are near zero.

**In `log()` method:**
- Check if `v_r < 1e-6` before calling `.getAngle()` on velocity vectors
- Log 0.0 for angles and travel time when velocity is too low
- Prevents `ArithmeticException` or `NaN` propagation

---

## Files Modified

| File | Changes |
|------|---------|
| `Robot.java` | Added profiling to `robotPeriodic()` |
| `Drive.java` | Removed blocking CAN refresh, added granular profiling |
| `Module.java` | Added profiling, removed signal methods |
| `ModuleIO.java` | Removed `refreshSignals()` and `getStatusSignals()` interface methods |
| `ModuleIOTalonFX.java` | Removed signal caching and refresh code |
| `Vision.java` | Added profiling, throttled logging |
| `VisionConstants.java` | Added `kLoggingDivisor` constant |
| `Intake.java` | Added profiling to `periodic()` |
| `Feeder.java` | Added profiling to `periodic()` |
| `IntakeRollerIOTalonFX.java` | Changed to non-blocking status checks, moved Logger calls to inputs struct |
| `IntakeRollerIO.java` | Added follower telemetry fields to inputs |
| `FlywheelIOTalonFX.java` | Changed to non-blocking status checks |
| `Launcher.java` | Added profiling, deferred logging, zero-velocity guard |
| `Hopper.java` | Added profiling to `periodic()` |

---

## Phase 3: Analysis of New Subsystems

Subsystems added since `performance-tweaks-2` was branched were analyzed for blocking calls:

| Subsystem | IO Implementation | Status |
|-----------|-------------------|--------|
| `Hopper` | `HopperIOReal` | Uses DoubleSolenoid (pneumatics, not CAN) - no blocking issues. Added profiling. |
| `Feeder/Spindexer` | `SpindexerIOSpark` | Uses `SparkOdometryThread` for non-blocking CAN reads. |
| `Feeder/Kicker` | `KickerIOSpark` | Uses `SparkOdometryThread` for non-blocking CAN reads. |
| `Launcher/Hood` | `HoodIOSpark` | Uses `SparkOdometryThread` for non-blocking CAN reads. |
| `Launcher/Turret` | `TurretIOSpark` | Uses `SparkOdometryThread` for non-blocking CAN reads. |
| `Intake/Arm` | `IntakeArmIOReal` | Uses DoubleSolenoid (pneumatics, not CAN) - no blocking issues. |
| `Drive/Gyro` | `GyroIOBoron` | Uses `CanandgyroThread` for non-blocking CAN reads. |

**Conclusion:** All motor controller IO implementations properly use background threading (`SparkOdometryThread` or `CanandgyroThread`) for CAN reads. Pneumatic subsystems use WPILib's DoubleSolenoid which doesn't have blocking CAN concerns.

---

## Phase 4: Throttled Vision Logging

Profiling revealed that `Logger.processInputs()` serialization was a major source of loop overruns, not CAN I/O. Vision logging was particularly expensive due to serializing `PoseObservation` arrays for multiple cameras.

### Changes

**VisionConstants.java:**
- Added `kLoggingDivisor` constant (default: 1)
- Set to 2+ to log every Nth cycle, reducing CPU load

**Vision.java:**
- Added `loopCounter` to track cycles
- Throttled `Logger.processInputs()` calls based on `kLoggingDivisor`
- Throttled summary pose logging similarly
- Split profiling to separate `snapshot` (volatile read) from `processInputs` (serialization)

**Trade-offs:**
- `kLoggingDivisor = 1`: Full data for replay, higher CPU load
- `kLoggingDivisor = 2`: Half the vision log data, ~50% reduction in vision logging overhead
- Higher values lose more replay granularity but further reduce load

**Profiling output now shows:**
```
[Vision] snapshot=0ms processInputs=12ms cameraLoop=3ms consumer=0ms summaryLog=2ms total=17ms
```

---

## Phase 5: Intake Logger Fix

Profiling revealed `[Intake] update=23ms` spikes. Investigation found `Logger.recordOutput()` calls inside `IntakeRollerIOTalonFX.updateInputs()`, which was measured as "update" time rather than "log" time.

### Root Cause

```java
// In IntakeRollerIOTalonFX.updateInputs() - BEFORE
Logger.recordOutput("Intake/Follower/Current", followerCurrent.getValue());
Logger.recordOutput("Intake/Follower/Volts", followerAppliedVolts.getValue());
```

Each `Logger.recordOutput()` call can take 10-30ms due to serialization overhead, causing the 23ms "update" time.

### Fix

Moved follower data into the `IntakeRollerIOInputs` struct so it gets logged via `Logger.processInputs()` instead:

**IntakeRollerIO.java:**
- Added `followerAppliedVolts` and `followerCurrentAmps` fields to inputs struct

**IntakeRollerIOTalonFX.java:**
- Removed `Logger.recordOutput()` calls from `updateInputs()`
- Populate `inputs.followerAppliedVolts` and `inputs.followerCurrentAmps` instead
- Removed unused Logger import

This follows the same pattern as the Launcher deferred logging fix - keep IO methods fast by avoiding Logger calls, and let the subsystem's `periodic()` handle logging via `Logger.processInputs()`.

---

## References

- [Chief Delphi: Let's talk about CAN delays](https://www.chiefdelphi.com/t/lets-talk-about-can-delays-and-how-to-address-them/396566)
- [Phoenix 6 Status Signals Documentation](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html)
- [AdvantageKit TalonFX Swerve Template](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template)
