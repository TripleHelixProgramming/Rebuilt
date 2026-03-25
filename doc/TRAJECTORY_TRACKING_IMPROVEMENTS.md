# Improving Trajectory Tracking

This document outlines strategies for reducing trajectory tracking error on a swerve drive robot, roughly ordered from easiest to hardest. The current system uses Choreo trajectories with P-only feedback controllers (Kp=8.01) on X, Y, and heading, plus Choreo's velocity feedforward. At VACHE, the best-case tracking error was about 0.2 m at trajectory end, with peaks of 1.0-1.4 m mid-trajectory.

The goal is to understand *where the error comes from* and *what we can do about it* — not to implement everything at once, but to have a menu of options ranked by effort and impact.

## Where does tracking error come from?

Tracking error has four sources. Fixing any one of them improves the result, but they compound — a 5% improvement in each of four areas is better than a 20% improvement in one.

### 1. Feedforward model inaccuracy

Choreo pre-computes a trajectory: at each timestep, the robot should be at position (x, y, θ) with velocity (vx, vy, ω). The velocity feedforward is the open-loop command that *should* produce the desired motion if the robot's physics model is perfect. When the model is wrong — the robot is heavier than expected, the wheels slip more, the carpet has more friction — the feedforward under- or over-shoots, and the PID has to clean up the difference.

**Why it matters**: feedforward does most of the work. If it's 90% accurate, the PID only needs to correct 10%. If it's 70% accurate, the PID is fighting a 30% error on every cycle.

### 2. Feedback controller limitations

The current controller is P-only (proportional). It applies a correction proportional to the error: if you're 0.5 m off, it pushes at 8.01 × 0.5 = 4.0 m/s toward the target. This has two inherent limitations:

- **No steady-state elimination**: A P controller always needs *some* error to produce *some* output. It can get close to zero error but never truly reaches it. This is called steady-state error, and it's why the trajectory end error is 0.2 m rather than 0.0 m.
- **No damping**: Without a D term, the controller has no way to sense that it's approaching the target too fast. It can overshoot and oscillate, especially during sharp trajectory changes.

### 3. Pose estimation error

The path follower can only correct what it can see. If the pose estimator says the robot is at (4.3, 0.6) but it's actually at (4.5, 0.8), the controller is correcting a phantom error while ignoring the real one. The error sources in pose estimation are:

- **Odometry drift**: wheel slip, wheel radius error, carpet compression variations
- **Vision latency**: observations are 50-80 ms old by the time they're fused, so the robot has moved since the image was captured
- **Vision noise**: even good observations have some scatter
- **Kalman filter tuning**: the standard deviations that control how much the filter trusts odometry vs vision

### 4. Mechanical inconsistencies

- **Module azimuth error**: if a module's zero angle is off by 1°, every velocity command is slightly misdirected
- **Wheel radius variation**: if the actual wheel radius differs from the configured value, odometry accumulates a proportional position error over time
- **Coupling ratio**: on MK4 modules, steering the azimuth back-drives the drive motor slightly. If the coupling ratio isn't compensated, it creates phantom drive encoder counts

## Improvement strategies

### Tier 1: Low effort, high impact

#### Fix the Choreo model mismatch

The Choreo configuration (`Rho.chor`) and the robot code (`DriveConstants.java`) disagree on several parameters:

| Parameter | Choreo (`Rho.chor`) | Code (`DriveConstants`) | Impact |
|-----------|---------------------|------------------------|--------|
| **Mass** | 130 lbs | 150 lbs | Choreo generates trajectories for a lighter robot. The real robot accelerates more slowly, so the path follower constantly lags the trajectory. |
| **COF** | 1.0 | 1.2 | Choreo is more conservative about traction limits than PathPlanner. Inconsistency means different behavior depending on which planner generated the path. |
| **Motor torque (tmax)** | 0.25 N*m | — | This seems very low for a Kraken X60 (stall torque ~7 N*m). If this limits Choreo's trajectory generation, the trajectories may be unnecessarily slow. |

**Fix**: weigh the robot at competition weight (bumpers, battery, game pieces), update both `Rho.chor` and `DriveConstants` with the same value, and verify `tmax` in Choreo's documentation.

**Expected impact**: this is probably the single highest-impact code change. A 15% mass error (130 vs 150 lbs) means the feedforward is systematically 15% wrong during acceleration phases.

#### Characterize wheel radius precisely

The wheel radius appears in two critical places: odometry (how far the robot thinks it moved per encoder tick) and feedforward (how fast the wheels need to spin for a desired velocity). A 2% error in wheel radius means 2% error in both.

Your current value is 2 inches (0.0508 m). The `wheelRadiusCharacterization` command measures the actual value by spinning in place and comparing gyro rotation to wheel encoder rotation. Run this on the actual competition carpet — carpet compression changes the effective radius. Do it with the robot at competition weight. On worn treads, the effective radius can drop to 1.85-1.9 inches, which causes 5-7% distance tracking error.

Update the measured value in both `DriveConstants.wheelRadius` and `Rho.chor`.

**Expected impact**: if the current value is off by 2-3%, fixing it could reduce tracking error proportionally. Small in absolute terms but it's free accuracy.

#### Fix cameras 2 and 3

At VACHE, cameras 2 and 3 detected zero tags in every single match before auto. This meant no multi-camera fusion, no score-boosted observations, and marginal observation quality. Fixing this alone would likely be the single biggest improvement to pre-auto localization.

Possible causes: disconnected cables, camera orientation pointing away from tags at starting positions, PhotonVision pipeline misconfiguration, or coprocessor not running.

**Expected impact**: multi-camera fusion produces observation scores of 0.9-1.0 instead of 0.65-0.68. The pose estimator would converge faster and more accurately before auto even starts.

### Tier 2: Moderate effort, moderate impact

#### Add a D term to the trajectory controllers

Change the X, Y, and heading controllers from P to PD:

```java
private final PIDController xController = new PIDController(8.01, 0.0, 0.5);
private final PIDController yController = new PIDController(8.01, 0.0, 0.5);
```

The D term resists rapid changes in error — it's a braking force that reduces overshoot. Start with a small value (0.3-0.5) and increase until you see improved settling without oscillation. Tune on carpet, not on smooth floors. Be cautious: D amplifies sensor noise, so too much will cause jitter.

Also consider tuning heading separately from translation — rotational inertia is different from translational inertia, so the same gains aren't necessarily optimal for both. Many teams use P=3-5 for heading vs P=8 for translation.

**Expected impact**: faster convergence to the trajectory, less overshoot during direction changes. Won't help with steady-state error but reduces peak error.

#### Add an I term (carefully) for steady-state elimination

An integral term accumulates error over time and eventually pushes the output high enough to eliminate steady-state offset. But it's dangerous: if the error is large for a sustained period (like the start of auto), the integral winds up and causes massive overshoot when the error finally decreases.

Use WPILib's `PIDController.setIntegratorRange()` to clamp the integral accumulator:

```java
xController.setIntegratorRange(-0.5, 0.5);
```

**Expected impact**: could reduce the 0.2 m steady-state tracking error toward zero. Requires careful tuning.

#### Tune Kalman filter standard deviations

The `SwerveDrivePoseEstimator` uses standard deviations to balance odometry trust vs vision trust. The current vision standard deviations are:

```
linearStdDev = 0.02 * (3.0 for single-cam, 1.0 for multi-cam) / score
```

For single-camera observations at a typical score of 0.67, this gives ~0.09 m. For multi-camera, ~0.03 m. These seem reasonable, but experiment with the odometry standard deviations — making them larger (less trust in odometry) would make the filter respond faster to vision corrections, which helps during the initial seconds of auto.

The gyro is very accurate for short-term heading tracking, so you could tighten the heading odometry stddev relative to the position stddevs.

**Expected impact**: tighter pose estimates, especially during the 2-3 seconds after auto start when the pose is converging.

#### Add trajectory error logging

Add logging inside `followTrajectory()`:

```java
Logger.recordOutput("Trajectory/ErrorX", pose.getX() - sample.x);
Logger.recordOutput("Trajectory/ErrorY", pose.getY() - sample.y);
Logger.recordOutput("Trajectory/ErrorHeading",
    pose.getRotation().getRadians() - sample.heading);
```

This gives you real-time visibility into whether the robot is consistently lagging (feedforward too weak), oscillating (P too high or D too low), or drifting (odometry issue). You can't improve what you can't measure.

**Expected impact**: no direct tracking improvement, but it makes every other tuning effort more effective because you can see exactly what's happening.

#### Reduce vision processing interval during auto

The vision system batches observations for 5 robot loops (~100 ms) before processing them. During fast auto movement, this adds 100 ms of latency before the pose estimator gets a correction. Consider reducing `processingIntervalLoops` to 2-3 during auto for faster vision correction, at the cost of less multi-camera temporal correlation.

**Expected impact**: faster pose convergence during the first seconds of auto.

### Tier 3: Higher effort, significant impact

#### Use a full-state feedback controller (LQR)

WPILib provides `LinearQuadraticRegulator` which optimally balances position accuracy vs control effort. Unlike PID where you tune three gains by feel, LQR lets you specify "I care about position error this much and control effort this much" and it computes the optimal gains mathematically.

For trajectory tracking, you'd model the system as a double integrator (acceleration → velocity → position) for each axis, then let LQR compute the feedback gains. This naturally provides both proportional and derivative action with optimal tuning.

That said, most top FRC teams — including 254, 1678, and 2910 — use PID + feedforward rather than LQR for trajectory following. The consensus is that with accurate feedforward, the feedback controller barely has to do any work, and the added complexity of LQR isn't worth it for the 15-second auto period. LQR becomes more valuable when the feedforward model is poor or the disturbances are large.

**Expected impact**: better transient response with less manual tuning. Most impactful when feedforward accuracy is limited.

#### Implement velocity feedback

Currently, `followTrajectory` uses only position feedback. The trajectory provides velocity setpoints (sample.vx, sample.vy) as feedforward, but there's no feedback on whether the robot is actually achieving those velocities.

Adding velocity feedback catches problems like wheel slip immediately rather than waiting for the position error to grow:

```java
ChassisSpeeds measured = getRobotRelativeChassisSpeeds();
double vxError = measured.vxMetersPerSecond - sample.vx;
double vyError = measured.vyMetersPerSecond - sample.vy;
```

**Expected impact**: faster response to disturbances (hitting game pieces, carpet transitions, collisions). Reduces peak tracking error during high-speed segments.

#### Improve vision during motion

The current system gets very few accepted vision observations during the first seconds of auto because the robot is moving fast and the observations are marginal quality:

- **Reduce camera exposure time**: faster shutter = less motion blur = better PnP solutions at speed. Trade-off is darker images.
- **Use multi-tag PnP**: if a camera can see multiple tags simultaneously, the PnP solution is much more constrained. PhotonVision supports this.
- **Optimize camera placement**: point cameras toward areas of the field with closely-spaced tags.
- **Add speed-based vision stddev scaling**: when the robot is moving at 4+ m/s, AprilTag detections have more motion blur and should be trusted less. Scale the vision standard deviations proportionally to robot speed.

**Expected impact**: faster and more reliable pose convergence during auto.

### Tier 4: Advanced

#### Predictive feedforward with acceleration compensation

Since the robot can't change velocity instantaneously, the feedforward should command slightly *ahead* of the current trajectory point to account for response delay. This is called "feedforward with phase lead" and is especially helpful during high-acceleration segments.

#### Time-optimal trajectory replanning

Instead of following a pre-computed trajectory and correcting errors with PID, replan the trajectory on the fly from the robot's current state. PathPlanner supports this through its replanning feature. This eliminates the concept of "tracking error" — the trajectory is always correct for where the robot actually is.

**Expected impact**: eliminates large transient errors from initial pose offset or mid-auto disturbances. Higher CPU cost.

## Where to start

For your current system, the highest-impact improvements are:

1. **Fix the Choreo mass mismatch** (130 lbs vs 150 lbs) and verify tmax — this is causing systematic feedforward error in every trajectory
2. **Fix cameras 2 and 3** — hardware/configuration issue that would dramatically improve localization
3. **Characterize wheel radius on competition carpet** — free accuracy, update both code and Choreo config
4. **Add trajectory error logging** — can't improve what you can't measure
5. **Add D term to trajectory controllers** — small code change, reduces overshoot

These five changes could plausibly reduce your 0.2-1.4 m tracking range to 0.05-0.5 m with modest effort. The more advanced strategies (LQR, velocity feedback, replanning) are worth exploring for a future season where scoring tolerances demand it.

## References

- **6328 (Mechanical Advantage)**: your codebase is based on their AdvantageKit template. Their 2024 code is at `github.com/Mechanical-Advantage/RobotCode2024`.
- **Choreo config guide**: `choreo.autos/usage/estimating-config/` — how to estimate mass, MOI, and motor parameters for accurate trajectory generation.
- **WPILib pose estimator docs**: `docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html`
- **Chief Delphi**: search for "swerve trajectory tracking tips" and "choreo tuning" for community experience.
