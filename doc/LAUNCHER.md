# Launcher Subsystem

This document explains the launcher (shooting) system, covering the mechanical design, ballistics calculations, and motor control.

## Table of Contents

1. [Overview](#overview)
2. [Mechanical Components](#mechanical-components)
3. [Ballistics Physics](#ballistics-physics)
4. [Moving Robot Compensation](#moving-robot-compensation)
5. [Motor Control](#motor-control)
6. [Code Structure](#code-structure)
7. [Tuning Guide](#tuning-guide)

---

## Overview

The launcher shoots game pieces (fuel) into an elevated target (the hub). It consists of three coordinated mechanisms:

```
                    Target (Hub)
                        ●
                       /
                      /  ← Ball trajectory
                     /
    ┌───────────────●───────────────┐
    │             Hood              │  ← Controls vertical angle
    │         ┌─────────┐           │
    │         │ Flywheel│           │  ← Provides launch velocity
    │         └─────────┘           │
    │              │                │
    │         ┌────┴────┐           │
    │         │  Turret │           │  ← Controls horizontal angle
    │         └─────────┘           │
    │            Robot              │
    └───────────────────────────────┘
```

The system must:
1. Calculate where to aim based on target position
2. Spin the flywheel to the right speed
3. Adjust hood angle for the correct trajectory
4. Rotate turret to face the target
5. Compensate for robot motion while driving

---

## Mechanical Components

### Turret

**Purpose**: Rotates the entire shooting mechanism horizontally to face the target.

```
         Top View
    ┌─────────────────┐
    │     ┌───┐       │
    │     │ T │──────▶│  Turret can rotate
    │     └───┘       │  ±90° from forward
    │                 │
    └─────────────────┘
```

**Hardware**:
- **Motor**: NEO 550 (SparkMax controller)
- **Gear Ratio**: 54:1 (9:1 × 72:12)
- **Encoder**: Absolute encoder on DIO for startup position
- **Range**: 180° total (±90° from forward)

**Why a Turret?**
- Faster than rotating the entire robot
- Can track target while robot drives elsewhere
- Allows continuous shooting while moving

### Flywheel

**Purpose**: Accelerates the game piece to launch velocity.

```
    Side View
         ○ ← Ball enters
        ╱
    ═══●═══  ← Flywheel (spinning)
        ╲
         ○ → Ball exits at high speed
```

**Hardware**:
- **Motors**: 2× Kraken X60 (TalonFX) in follower configuration
- **Gear Ratio**: 1:1 (direct drive for max speed)
- **Wheel Radius**: 1.5 inches

**How It Works**:
1. Ball enters the flywheel mechanism
2. Spinning wheels grip and accelerate the ball
3. Ball exits at a fraction of wheel surface speed

**Ball-to-Flywheel Relationship**: The ball doesn't exit at full wheel speed due to:
- Compression of the ball
- Slip between ball and wheel
- Energy lost to ball deformation

The relationship is nonlinear. To achieve a desired ball velocity, the flywheel setpoint is:
`flywheel_speed = 0.335 × ball_velocity^1.8`

### Hood

**Purpose**: Adjusts the vertical launch angle.

```
    Side View at Different Angles

    Low Hood (long shot)     High Hood (close shot)
          ╱                        │
         ╱                         │
    ════●════                 ════●════
         ╲                        ╲
          → Shallow angle          → Steep angle
```

**Hardware**:
- **Motor**: NEO 550 (SparkMax controller)
- **Gear Ratio**: 80:1 (high reduction for precision)
- **Range**: 60° to 80° from horizontal

**Why Variable Hood?**
- Close shots need steep angles to clear the target lip
- Far shots need shallow angles to reach the target
- Fixed angle would limit effective range

---

## Ballistics Physics

### The Problem

Given:
- Target position (x, y, z) in field coordinates
- Current robot position and heading

Calculate:
- Flywheel speed
- Hood angle
- Turret angle

### Projectile Motion Review

In a vacuum (no air resistance), a projectile follows:

```
Horizontal: x(t) = x₀ + vₓ·t
Vertical:   z(t) = z₀ + vᵤ·t - ½·g·t²

Where:
- (x₀, z₀) = initial position
- vₓ = horizontal velocity
- vᵤ = vertical velocity
- g = 9.81 m/s² (gravity)
- t = time
```

### Impact Angle Approach

Rather than just hitting the target, we want to hit it at a specific **impact angle**. A steeper impact angle:
- Increases the effective target area
- Makes shots less sensitive to distance errors
- Helps balls "fall into" the target

```
              ┌───┐
              │   │ Target
    ─ ─ ─ ─ ─▶│   │  ← Shallow impact (risky)
              │   │
         ╲    │   │
          ╲   │   │
           ╲  │   │  ← Steep impact (better)
            ╲▼└───┘
```

### The Math

Given target at (dᵣ, dᵤ) from launcher (horizontal distance, height difference), and desired impact angle θᵢ:

**Step 1**: Calculate required initial velocity magnitude

```
v₀ᵣ = dᵣ × √(g / (2 × (dᵤ + dᵣ × tan(θᵢ))))
```

**Step 2**: Calculate vertical component

```
v₀ᵤ = (g × dᵣ) / v₀ᵣ - v₀ᵣ × tan(θᵢ)
```

**Step 3**: Convert to mechanism angles

```
hood_angle = atan2(v₀ᵤ, v₀ᵣ)
turret_angle = atan2(target_y, target_x)  // in field frame
flywheel_speed = √(v₀ᵣ² + v₀ᵤ²) × ball_to_flywheel_factor
```

### Distance-Based Impact Angle

We use different impact angles based on distance:

| Distance | Impact Angle | Reason |
|----------|--------------|--------|
| ≤2m | 55° | Steep for large target window |
| ≥6m | 40° | Shallow to reach target |
| Between | Interpolated | Smooth transition |

```java
// Linear interpolation between close and far angles
double t = (distance - 2.0) / (6.0 - 2.0);  // 0 to 1
t = clamp(t, 0, 1);
impactAngle = 55° + t × (40° - 55°);
```

### When Targets Are Unreachable

Sometimes the math has no solution (discriminant < 0):
- Target is too far for available flywheel speed
- Target is above the maximum trajectory height

In these cases, we keep the last valid solution to avoid erratic behavior.

---

## Moving Robot Compensation

### The Problem

When the robot moves, the ball inherits that motion. If we aim directly at a stationary target, we'll miss.

```
    Robot moving right →

    Without compensation:        With compensation:
         ○                            ○
        ╱                            ╱
       ╱  Ball curves right         ╱  Ball appears to curve
      ╱                            ╱   but hits target
     ●───→                        ●───→
    Robot                        Robot

    Target                       Target
      X  Miss!                     ● Hit!
```

### The Solution: Vector Addition

The ball's velocity relative to the field is:

```
v_ball_field = v_ball_launcher + v_launcher_field

Where:
- v_ball_launcher = velocity we give the ball
- v_launcher_field = velocity of the turret due to robot motion
```

To hit the target, we need:

```
v_ball_launcher = v_required - v_launcher_field
```

### Implementation

1. Calculate required ball velocity (from ballistics)
2. Get turret velocity from chassis speeds:
   ```
   v_turret = v_chassis + ω × r_turret
   ```
3. Subtract to get required flywheel-relative velocity
4. Aim turret along this adjusted vector

### Replanning with Actual Speed

The flywheel takes time to spin up. Rather than waiting, we:

1. Calculate nominal speed for stationary robot
2. Set flywheel to that speed
3. Read actual current flywheel speed
4. Replan trajectory using actual speed
5. Adjust turret and hood angles

This means we can shoot while the flywheel is still accelerating, at the cost of slightly different trajectories.

---

## Motor Control

### Flywheel Control

**Control Mode**: Velocity control with feedforward

```
Target: 3000 RPM (50 rev/s)
Measured: 2900 RPM

PID Output = kP × error + kV × target
           = 0.11 × 100 + 0.12 × 3000
           = 11 + 360
           = 371 (arbitrary units)
```

**Key Parameters**:
- kP = 0.11 (proportional gain)
- kV = 0.12 (velocity feedforward)
- kS = 0.1 (static friction compensation)

**Motion Magic**: We use CTRE's Motion Magic for smooth acceleration:
- Max acceleration: 4000 RPM/s
- Max jerk: 40000 RPM/s²

### Hood Control

**Control Mode**: Position control

The hood must hold precise angles against gravity and ball impacts.

**Key Parameters**:
- kP = 0.35 (position gain)
- Soft limits: 60° to 80°
- Allowable error: 0.5°

### Turret Control

**Control Mode**: Position control with velocity feedforward

The turret must track a moving target smoothly.

```
position_output = kP × angle_error
velocity_feedforward = -robot_angular_velocity × 2.0
total_output = position_output + velocity_feedforward
```

The velocity feedforward compensates for robot rotation - if the robot rotates clockwise, the turret must rotate counter-clockwise to maintain aim.

---

## Code Structure

### Files

```
subsystems/launcher/
├── Launcher.java           # Main subsystem, ballistics
├── LauncherConstants.java  # All configuration
├── TurretIO.java          # Turret hardware interface
├── TurretIOSparkMax.java  # Real turret hardware
├── FlywheelIO.java        # Flywheel hardware interface
├── FlywheelIOTalonFX.java # Real flywheel hardware
├── HoodIO.java            # Hood hardware interface
└── HoodIOSparkMax.java    # Real hood hardware
```

### Key Methods in Launcher.java

#### `aim(Translation3d target)`

Main aiming method, called every loop:

```java
public void aim(Translation3d target) {
    // 1. Get vector from turret to target
    vectorTurretBaseToTarget = target.minus(turretBasePose);

    // 2. Calculate distance-based impact angle
    double distance = hypot(vector.getX(), vector.getY());
    Rotation2d impactAngle = getImpactAngle(distance);

    // 3. Calculate nominal velocity (stationary robot)
    var v0_nominal = getV0Nominal(vector, impactAngle);
    flywheelIO.setVelocity(v0_nominal.getNorm());

    // 4. Get turret base velocity from robot motion
    var v_base = getTurretBaseSpeeds(chassisSpeeds);

    // 5. Replan with actual flywheel speed
    var v0_total = getV0Replanned(vector, actualFlywheelSpeed);

    // 6. Calculate required flywheel-relative velocity
    var v0_flywheel = v0_total.minus(v_base);

    // 7. Command mechanisms
    turretIO.setPosition(v0_flywheel.getAngle());
    hoodIO.setPosition(v0_flywheel.getElevation());
}
```

#### `getV0Nominal(Translation3d d, Rotation2d impactAngle)`

Calculates required velocity for stationary robot:

```java
private Translation3d getV0Nominal(Translation3d d, Rotation2d impactAngle) {
    double dr = hypot(d.getX(), d.getY());  // horizontal distance
    double dz = d.getZ();                    // height difference

    // Check if target is reachable
    double denominator = 2 * (dz + dr * impactAngle.getTan());
    if (denominator <= 0) return lastValidSolution;

    // Calculate velocity components
    double v_0r = dr * sqrt(g / denominator);
    double v_0z = (g * dr) / v_0r - v_0r * impactAngle.getTan();

    return new Translation3d(v_0r * cos(azimuth), v_0r * sin(azimuth), v_0z);
}
```

#### `getV0Replanned(Translation3d d, double speed)`

Adjusts trajectory for actual flywheel speed:

```java
private Translation3d getV0Replanned(Translation3d d, double v_flywheel) {
    double dr = hypot(d.getX(), d.getY());
    double dz = d.getZ();

    // Solve projectile equation for angle given speed
    double discriminant = v⁴ - g(g·dr² + 2·dz·v²);
    if (discriminant < 0) return lastValidSolution;

    // Use high-arc solution for better accuracy
    double tanTheta = (v² + sqrt(discriminant)) / (g * dr);

    // Convert back to velocity components
    double v_r = v_flywheel / sqrt(1 + tanTheta²);
    double v_z = v_r * tanTheta;

    return new Translation3d(v_r * cos(azimuth), v_r * sin(azimuth), v_z);
}
```

---

## Tuning Guide

### Flywheel Characterization

1. **Measure ball exit velocity** at various flywheel speeds
2. **Fit a curve** to find the ball-to-flywheel relationship
3. **Update constants** in LauncherConstants.java

Current relationship: `flywheel = 0.335 × ball^1.8`

### Hood Calibration

1. Move hood to physical stop
2. Reset encoder to known angle
3. Verify soft limits prevent over-travel

### Turret Calibration

1. Point turret straight forward manually
2. Read absolute encoder value
3. Set offset so reported angle is 0°

### Ballistics Tuning

If shots consistently miss:

| Symptom | Adjustment |
|---------|------------|
| Shots fall short | Increase ball-to-flywheel factor |
| Shots go long | Decrease ball-to-flywheel factor |
| Shots go left/right | Check turret offset |
| Close shots miss | Adjust impactAngleClose |
| Far shots miss | Adjust impactAngleFar |

### Testing Procedure

1. **Static shots**: Robot stationary, various distances
2. **Moving shots**: Robot translating, stationary target
3. **Rotating shots**: Robot rotating in place
4. **Full field**: Combine all motions

---

## Related Documentation

- [OVERVIEW.md](OVERVIEW.md) - Project architecture
- [INTAKE_FEEDER.md](INTAKE_FEEDER.md) - How fuel gets to the launcher
- [VISION.md](VISION.md) - How we find the target
