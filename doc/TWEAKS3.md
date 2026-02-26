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

---

## Future Work (Phase 2)

The following changes from `performance-tweaks-2` still need to be adapted for the current `main` branch:

1. **Deferred Logging in Launcher** - Cache values during `aim()`, log them in `periodic()`
2. **Zero-Velocity Guard** - Prevent `Translation2d.getAngle()` errors when velocity is zero
3. **Launcher Profiling** - Add timing instrumentation to `periodic()` and `aim()`

These changes require more careful adaptation due to structural differences in `Launcher.java` between the branches.

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

---

## References

- [Chief Delphi: Let's talk about CAN delays](https://www.chiefdelphi.com/t/lets-talk-about-can-delays-and-how-to-address-them/396566)
- [Phoenix 6 Status Signals Documentation](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html)
- [AdvantageKit TalonFX Swerve Template](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template)
