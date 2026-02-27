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
| `Vision.java` | `periodic()` | >5ms | copyInputs, cameraLoop, consumer, summaryLog |
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
| `Vision.java` | Added profiling to `periodic()` |
| `Intake.java` | Added profiling to `periodic()` |
| `Feeder.java` | Added profiling to `periodic()` |
| `IntakeRollerIOTalonFX.java` | Changed to non-blocking status checks |
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

## References

- [Chief Delphi: Let's talk about CAN delays](https://www.chiefdelphi.com/t/lets-talk-about-can-delays-and-how-to-address-them/396566)
- [Phoenix 6 Status Signals Documentation](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html)
- [AdvantageKit TalonFX Swerve Template](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template)
