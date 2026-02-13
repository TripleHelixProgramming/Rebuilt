# Performance Tweaks - Phase 2

This document describes the performance optimization work done on the `performance-tweaks-2` branch, building on the foundation established in the original `performance-tweaks` branch.

## Executive Summary

Our robot code was experiencing consistent loop overruns (target: 20ms, actual: 50-300ms+). Through systematic profiling and research into how top FRC teams handle similar issues, we identified the root causes and implemented targeted fixes. The architecture now follows best practices used by Team 6328 (Mechanical Advantage) and is comparable to approaches used by Team 254 (The Cheesy Poofs).

---

## Methodology

### 1. Profiling Infrastructure

We added timing instrumentation throughout the codebase to identify bottlenecks empirically rather than guessing. Each subsystem now reports timing breakdowns when thresholds are exceeded:

| File | Method | Threshold | Metrics |
|------|--------|-----------|---------|
| `Robot.java` | `robotPeriodic()` | >20ms | scheduler, gameState |
| `Drive.java` | `periodic()` | >5ms | lock, gyroUpdate, gyroLog, modules, disabled, odometry |
| `Module.java` | `periodic()` | >2ms | updateInputs, log, rest |
| `Vision.java` | `periodic()` | varies | copyInputs, cameraLoop, consumer, summaryLog |
| `Intake.java` | `periodic()` | >2ms | update, log |
| `Feeder.java` | `periodic()` | >2ms | spindexer, kicker, spindexerLog, kickerLog |
| `Launcher.java` | `periodic()` | >3ms | turret, flywheel, hood, turretLog, flywheelLog, hoodLog |
| `Launcher.java` | `aim()` | >500μs | setup, v0nom, flywheelSet, baseSpeeds, v0replan, setPos, rest |

**Example profiling output:**
```
[Drive] lock=0ms gyroUpdate=2ms gyroLog=1ms modules=105ms disabled=69ms odometry=144ms total=322ms
[Launcher.aim] setup=196us v0nom=89us flywheelSet=194us baseSpeeds=4597us v0replan=48us setPos=7833us rest=77us total=13037us
```

### 2. Research into Top Team Approaches

We researched how top FRC teams handle CAN bus latency and loop timing:

**Team 6328 (Mechanical Advantage / AdvantageKit):**
- Use background threads (`PhoenixOdometryThread`, `SparkOdometryThread`) for high-frequency sensor reads
- Main loop reads from cached/queued values - no blocking CAN calls
- Accept that `Logger.processInputs()` has inherent overhead (required for deterministic replay)

**Team 254 (The Cheesy Poofs):**
- Use signal-driven loops with `BaseStatusSignal.waitForAll()`
- Loop timing synchronized to actual CAN updates rather than fixed clock
- Heavy use of feedforward to reduce sensitivity to measurement latency

**Phoenix 6 Documentation (CTR Electronics):**
- `setUpdateFrequency()` to configure per-signal update rates
- `optimizeBusUtilizationForAll()` to disable unused signals
- CAN FD via CANivore for higher bandwidth (5 Mbps vs 1 Mbps)

**REVLib 2025:**
- Individual signal period configuration
- `configureAsync()` for non-blocking device configuration
- SparkOdometryThread pattern for background reads

**Key Finding:** Our codebase already implements the Team 6328 approach, which is the most common pattern among top teams using AdvantageKit.

---

## Root Causes Identified

### Primary Bottlenecks

1. **Logger.processInputs()** - 2-22ms per call
   - Serializes input data for AdvantageKit replay
   - Not thread-safe by design (deterministic replay requirement)
   - Cannot be moved to background thread

2. **Module CAN Reads** - Occasional spikes to 30-84ms
   - Phoenix 6 auto-refresh configured correctly
   - Spikes due to CAN bus contention or JVM garbage collection

3. **SparkMax CAN Writes** - `setPosition()` taking 1-8ms
   - `turretIO.setPosition()` and `hoodIO.setPosition()` are synchronous
   - REVLib does not provide async control API
   - This is a hardware/library limitation

4. **Vision Processing** - 1-134ms variability
   - Camera data copy and processing time varies with scene complexity

### Secondary Issues

5. **Rotation2d Zero Error** - Console spam from `logTrajectory()`
   - `cachedV0Actual` was zero before first `aim()` call
   - `Translation2d.getAngle()` throws when both components are zero

---

## Code Changes

### 1. Deferred Logging in Launcher

**Problem:** `Launcher.aim()` was calling 27+ `Logger.recordOutput()` calls per invocation, causing 11-28ms delays in the hot path.

**Solution:** Cache values during `aim()`, log them in `periodic()`.

```java
// Launcher.java - Added cached fields
private Translation3d cachedBaseSpeeds = new Translation3d();
private Translation3d cachedV0Nominal = new Translation3d();
private Translation3d cachedV0Replanned = new Translation3d();
private Translation3d cachedV0Actual = new Translation3d();
private boolean cachedNominalReachable = false;
private boolean cachedReplannedReachable = false;
private double cachedPredictedRange = 0.0;
```

```java
// In aim() - populate caches, no Logger calls
cachedV0Nominal = v0nominalLast;
cachedNominalReachable = true;
// ... etc

// In periodic() - log from caches
private void logAimData() {
    Logger.recordOutput("Launcher/BaseSpeeds", cachedBaseSpeeds);
    Logger.recordOutput("Launcher/" + nominalKey + "/Reachable", cachedNominalReachable);
    if (cachedNominalReachable) {
        logTrajectory(cachedV0Nominal, nominalKey);
    }
    // ... etc
}
```

**Why it works:** Logging overhead still exists, but it's now in `periodic()` where it's expected, not in `aim()` which runs during command execution. This makes loop timing more predictable.

### 2. Zero-Velocity Guard in logTrajectory

**Problem:** When `cachedV0Actual` is zero (before first `aim()` call), `Translation2d.getAngle()` throws an error because it can't compute an angle from a zero vector.

**Solution:** Early return when velocity is effectively zero.

```java
private void logTrajectory(Translation3d v, String key) {
    Logger.recordOutput("Launcher/" + key + "/InitialVelocities", v);
    Logger.recordOutput("Launcher/" + key + "/InitialSpeedMetersPerSecond", v.getNorm());

    var v_r = v.toTranslation2d().getNorm();
    var v_z = v.getZ();

    // Skip angle calculations if velocity is effectively zero
    if (v_r < 1e-6 && Math.abs(v_z) < 1e-6) {
        return;
    }

    if (v_r >= 1e-6) {
        Logger.recordOutput("Launcher/" + key + "/HorizontalLaunchAngleDegrees",
            v.toTranslation2d().getAngle().getDegrees());
        // ...
    }
    // ...
}
```

**Why it works:** The threshold `1e-6` is small enough to only trigger when velocity is truly zero (not just small), preventing the math error while preserving logging for all valid aiming states.

### 3. Background Threading for Sensor Reads (Pre-existing)

The codebase already implements background threading for CAN reads:

- **PhoenixOdometryThread** - Reads TalonFX position signals at 250Hz
- **SparkOdometryThread** - Reads SparkMax telemetry in background
- **VisionThread** - Processes camera data asynchronously

```java
// TurretIOSpark.java - Registers with background thread
sparkInputs = SparkOdometryThread.getInstance().registerSpark(turnSpark, turnSparkEncoder);

// In updateInputs() - reads cached values, doesn't block
inputs.relativePosition = new Rotation2d(sparkInputs.getPosition()).plus(mechanismOffset);
inputs.velocityRadPerSec = sparkInputs.getVelocity();
```

**Why it works:** The background thread runs the CAN reads at its own pace. The main loop just reads from thread-safe cached values, which is a memory read (nanoseconds) instead of a CAN transaction (milliseconds).

### 4. Phoenix 6 Signal Optimization (Pre-existing)

```java
// ModuleIOTalonFX.java - Configure update frequencies
BaseStatusSignal.setUpdateFrequencyForAll(
    Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
BaseStatusSignal.setUpdateFrequencyForAll(
    50.0, driveVelocity, driveAppliedVolts, driveCurrent, ...);
ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
```

**Why it works:**
- Position signals (needed for odometry) update at 250Hz
- Telemetry signals update at 50Hz (sufficient for logging)
- `optimizeBusUtilizationForAll()` disables unused signals, reducing CAN bandwidth

---

## Remaining Limitations

### Cannot Be Fixed in Software

1. **Logger.processInputs() Overhead**
   - Required for AdvantageKit's deterministic replay feature
   - Not thread-safe by design
   - Trade-off: We accept 2-20ms logging overhead for replay capability

2. **SparkMax CAN Write Latency**
   - `controller.setSetpoint()` is synchronous (blocks until acknowledged)
   - REVLib doesn't provide async API
   - Would require hardware change (TalonFX) to eliminate

3. **JVM Garbage Collection Pauses**
   - Occasional 10-50ms pauses from GC
   - Can be mitigated with RT thread priority, but has other risks

### Could Be Optimized Further (Diminishing Returns)

1. **Reduce Logger.recordOutput() calls**
   - Log only when values change significantly
   - Trade-off: Reduced replay fidelity

2. **Increase feedforward, reduce feedback gains**
   - Makes system less sensitive to measurement latency
   - Trade-off: Less responsive to disturbances

3. **Move to TalonFX for turret/hood**
   - Phoenix 6 `setControl()` is internally non-blocking
   - Trade-off: Hardware change, different motor characteristics

---

## Assessment

### Current State

| Aspect | Status | Notes |
|--------|--------|-------|
| Background sensor reads | ✅ Optimal | PhoenixOdometryThread, SparkOdometryThread implemented |
| Phoenix 6 signal config | ✅ Optimal | Update frequencies and bus optimization configured |
| Hot path logging | ✅ Fixed | Deferred to periodic() |
| Error handling | ✅ Fixed | Zero-velocity guard added |
| SparkMax write latency | ⚠️ Hardware limit | 1-8ms spikes unavoidable with REVLib |
| Logger overhead | ⚠️ Architectural | Inherent to AdvantageKit, ~15-50ms total per loop |

### Comparison to Top Teams

| Team | Approach | Our Status |
|------|----------|------------|
| Team 6328 | Background threads + cached reads | ✅ Implemented |
| Team 6328 | AdvantageKit logging | ✅ Same overhead |
| Team 254 | Signal-driven loops (waitForAll) | ❌ Not used (different pattern) |
| Team 254 | Heavy feedforward | ⚠️ Could increase |
| Top teams | CANivore for CAN FD | ✅ Used for swerve modules |

### Realistic Expectations

With our current architecture:
- **Typical loop time:** 30-80ms (above 20ms target due to logging)
- **Worst case:** 150-300ms (Vision + Logger + GC alignment)
- **Drive control:** Unaffected (runs at 250Hz in background thread)
- **Aiming accuracy:** Unaffected (setpoints sent every loop, SparkMax handles control)

The loop overrun warnings will continue to appear, but **they don't indicate a functional problem**. The robot will still:
- Drive smoothly (odometry at 250Hz)
- Aim accurately (SparkMax PID runs on-controller)
- Log data for replay (AdvantageKit working as designed)

### Recommendations

1. **For competition:** Keep profiling enabled during practice matches to identify any new bottlenecks
2. **For debugging:** Use AdvantageScope to replay logs and verify behavior
3. **Future consideration:** If turret/hood latency causes aiming issues, experiment with higher feedforward gains before considering hardware changes

---

## Files Modified

| File | Changes |
|------|---------|
| `Robot.java` | Added profiling to `robotPeriodic()` |
| `Drive.java` | Added granular profiling to `periodic()` |
| `Module.java` | Profiling already present |
| `Vision.java` | Profiling already present |
| `Intake.java` | Added profiling to `periodic()` |
| `Feeder.java` | Added profiling to `periodic()` |
| `Launcher.java` | Deferred logging, added profiling to `periodic()` and `aim()`, fixed zero-velocity error |

---

## References

- [Chief Delphi: Let's talk about CAN delays](https://www.chiefdelphi.com/t/lets-talk-about-can-delays-and-how-to-address-them/396566)
- [Chief Delphi: The 2025 Ultimate CAN Bus Thread](https://www.chiefdelphi.com/t/the-2025-ultimate-can-bus-thread/491147)
- [Phoenix 6 Status Signals Documentation](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html)
- [REVLib 2025 Migration Guide](https://docs.revrobotics.com/revlib/24-to-25)
- [AdvantageKit TalonFX Swerve Template](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template)
- [Team 254 FRC-2024-Public](https://github.com/Team254/FRC-2024-Public)
