# Drive Subsystem

This document explains the swerve drive system, covering both the mechanical concepts and the software implementation.

## Table of Contents

1. [What is Swerve Drive?](#what-is-swerve-drive)
2. [Hardware Components](#hardware-components)
3. [Coordinate Systems](#coordinate-systems)
4. [How Swerve Math Works](#how-swerve-math-works)
5. [Motor Control Basics](#motor-control-basics)
6. [Odometry and Pose Estimation](#odometry-and-pose-estimation)
7. [Code Structure](#code-structure)
8. [Tuning and Calibration](#tuning-and-calibration)

---

## What is Swerve Drive?

Swerve drive is a drivetrain where each wheel can both spin (drive) and rotate (steer) independently. This gives the robot **omnidirectional movement** - it can drive in any direction while facing any direction.

### Comparison to Other Drivetrains

| Drivetrain | Movement | Complexity | Cost |
|------------|----------|------------|------|
| **Tank** | Forward/back, rotate in place | Simple | Low |
| **Mecanum** | Omnidirectional (slides) | Medium | Medium |
| **Swerve** | Omnidirectional (true) | High | High |

### Advantages of Swerve

- **Agility**: Strafe while shooting, dodge defenders
- **Speed**: All wheels always push in the direction of travel
- **Traction**: No wheel slip during lateral movement (unlike mecanum)

### Disadvantages

- **Complexity**: 8 motors, 4 encoders, complex math
- **Cost**: ~$2000+ for modules
- **Weight**: Heavier than tank drive

---

## Hardware Components

### Per Module (×4)

Each swerve module contains:

```
┌─────────────────────────────────────┐
│           SWERVE MODULE             │
│                                     │
│  ┌─────────────┐  ┌─────────────┐  │
│  │ Drive Motor │  │ Turn Motor  │  │
│  │ (TalonFX)   │  │ (TalonFX)   │  │
│  │ Kraken X60  │  │ Kraken X60  │  │
│  └──────┬──────┘  └──────┬──────┘  │
│         │                │         │
│         ▼                ▼         │
│  ┌─────────────┐  ┌─────────────┐  │
│  │ Drive Gear  │  │ Turn Gear   │  │
│  │ Reduction   │  │ Reduction   │  │
│  │ 6.75:1 L2   │  │ 12.8:1      │  │
│  └──────┬──────┘  └──────┬──────┘  │
│         │                │         │
│         ▼                ▼         │
│  ┌─────────────────────────────┐   │
│  │         Wheel               │   │
│  │    ┌─────────────┐          │   │
│  │    │ CANcoder    │          │   │
│  │    │ (absolute)  │          │   │
│  │    └─────────────┘          │   │
│  └─────────────────────────────┘   │
└─────────────────────────────────────┘
```

#### Drive Motor
- **Purpose**: Spins the wheel to move the robot
- **Type**: Kraken X60 (TalonFX controller)
- **Gear Ratio**: 6.75:1 (SDS MK4 L2)
- **Free Speed**: ~16 ft/s theoretical, ~14 ft/s actual

#### Turn Motor (Azimuth)
- **Purpose**: Rotates the wheel to change direction
- **Type**: Kraken X60 (TalonFX controller)
- **Gear Ratio**: 12.8:1
- **Range**: Unlimited (can spin continuously)

#### CANcoder
- **Purpose**: Knows the absolute wheel angle at startup
- **Type**: CTRE CANcoder (magnetic encoder)
- **Resolution**: 4096 counts per revolution
- **Why Needed**: Motor encoders only know relative position; CANcoder provides absolute reference

### Robot-Level Hardware

#### Gyroscope (IMU)
- **Type**: Redux Canandgyro
- **Purpose**: Measures robot rotation (heading)
- **Why Important**: Enables field-relative driving

### Physical Dimensions

```
          Front
    ┌───────────────┐
    │ FL         FR │   Wheelbase: 22.5"
    │               │   Track Width: 19.5"
    │       ↑       │   Wheel Radius: 2"
    │       │       │
    │    (robot)    │   Drive Base Radius: 14.9"
    │               │   (center to module)
    │ BL         BR │
    └───────────────┘
          Back
```

---

## Coordinate Systems

Understanding coordinate systems is crucial for swerve drive.

### Robot-Relative

- **+X**: Forward (front of robot)
- **+Y**: Left (driver's perspective when behind robot)
- **+Rotation**: Counter-clockwise

```
        +X (forward)
           ↑
           │
    +Y ←───┼───→ -Y
           │
           ↓
        -X (backward)
```

### Field-Relative

- **+X**: Toward the red alliance wall
- **+Y**: Toward the left (from blue alliance perspective)
- **+Rotation**: Counter-clockwise

```
    Blue Alliance                    Red Alliance
    ┌─────────────────────────────────────────┐
    │                                         │
    │  ←─── +Y                                │
    │   │                                     │
    │   ▼                                     │
    │  +X                              Goal   │
    │                                         │
    └─────────────────────────────────────────┘
```

### Why Field-Relative Matters

With field-relative driving:
- Push joystick forward → robot moves toward red alliance
- This is true **regardless of robot heading**

The gyro tells us which way the robot is facing, allowing us to convert field-relative commands to robot-relative commands.

---

## How Swerve Math Works

### The Problem

Given desired robot motion (vx, vy, omega), calculate what each wheel should do.

### Inverse Kinematics

For each module at position (x, y) from robot center:

```
v_module = v_robot + omega × r_module

Where:
- v_robot = (vx, vy) desired translation velocity
- omega = desired rotation rate
- r_module = position vector from center to module
```

Expanded:
```
v_x_module = vx - omega * y_module
v_y_module = vy + omega * x_module
```

Then convert to wheel speed and angle:
```
wheel_speed = sqrt(v_x_module² + v_y_module²)
wheel_angle = atan2(v_y_module, v_x_module)
```

### Wheel Optimization

Two optimizations make swerve smoother:

#### 1. Shortest Path Rotation
Instead of rotating 270° clockwise, rotate 90° counter-clockwise.

#### 2. Reverse Drive
If the target angle is more than 90° from current, flip the target by 180° and reverse the drive direction. This prevents unnecessary spinning.

```
Current: 0°, Target: 170°

Option A: Rotate to 170°, drive forward (170° rotation)
Option B: Rotate to -10°, drive backward (10° rotation) ← Better!
```

### Desaturation

If any wheel would exceed max speed, scale ALL wheels proportionally:

```
if max_wheel_speed > max_allowed:
    scale = max_allowed / max_wheel_speed
    for each wheel:
        wheel_speed *= scale
```

This preserves the robot's intended motion direction.

---

## Motor Control Basics

### Open Loop vs Closed Loop

**Open Loop**: Send voltage, hope for the best
- Simple but inaccurate
- Speed varies with battery voltage and load

**Closed Loop**: Use feedback to hit target
- More complex but precise
- PID controller adjusts output based on error

### PID Control

```
output = kP * error + kI * integral(error) + kD * derivative(error)

Where:
- kP: Proportional - bigger error = bigger correction
- kI: Integral - accumulated error over time
- kD: Derivative - rate of change of error
```

#### Drive Motor PID
- Controls wheel surface speed (m/s)
- Uses velocity feedback from motor encoder
- Gains: kP=10, kV=0.124 (feedforward for velocity)

#### Turn Motor PID
- Controls wheel angle (radians)
- Uses position feedback from CANcoder
- Gains: kP=300, kD=1.5, kS=0.1, kV=1.91

### Feedforward

Feedforward predicts the output needed without waiting for error:

```
output = kS * sign(velocity) + kV * velocity + kA * acceleration

Where:
- kS: Static friction overcome voltage
- kV: Velocity feedforward (voltage per unit velocity)
- kA: Acceleration feedforward
```

This gets you "in the ballpark" immediately; PID handles the fine-tuning.

### TorqueCurrentFOC

Our motors use **Field Oriented Control** (FOC) with torque (current) control:
- More precise than voltage control
- Better efficiency
- Requires good motor characterization

---

## Odometry and Pose Estimation

### What is Odometry?

Odometry tracks robot position by integrating wheel movements:

```
For each time step:
    1. Read wheel angles and distances traveled
    2. Calculate robot velocity using inverse kinematics
    3. Integrate velocity to update position

    position += velocity * dt
    heading += angular_velocity * dt
```

### Odometry Drift

Odometry accumulates error over time due to:
- Wheel slip
- Measurement noise
- Timing inaccuracies

After a full match, odometry might be off by several feet.

### Pose Estimation with Vision

We fuse odometry with vision to correct drift:

```
┌─────────────────┐     ┌─────────────┐
│    Odometry     │────▶│   Kalman    │────▶ Best Estimate
│ (fast, noisy)   │     │   Filter    │      of Position
└─────────────────┘     └─────────────┘
                              ▲
┌─────────────────┐           │
│     Vision      │───────────┘
│ (slow, accurate)│
└─────────────────┘
```

The Kalman filter weighs each source by its uncertainty:
- Odometry: Updated every 20ms, moderate trust
- Vision: Updated every 100ms, high trust (when tags visible)

### Standard Deviations

We configure how much to trust each source:

```java
// Odometry: trust position to ~10cm, heading to ~5°
odometryStdDevs = {0.1, 0.1, 0.1}  // meters, meters, radians

// Vision: trust position to ~50cm, heading to ~10°
visionStdDevs = {0.5, 0.5, 0.2}  // varies with distance
```

---

## Code Structure

### Key Files

```
subsystems/drive/
├── Drive.java              # Main subsystem class
├── DriveConstants.java     # All configuration
├── Module.java             # Per-module control
├── ModuleIO.java           # Hardware abstraction interface
├── ModuleIOTalonFX.java    # Real hardware implementation
├── ModuleIOSim.java        # Simulation implementation
├── GyroIO.java             # Gyro interface
└── GyroIOCanandgyro.java   # Real gyro implementation
```

### Drive.java Responsibilities

1. **Coordinate Transformation**: Field-relative → robot-relative
2. **Kinematics**: Robot velocity → per-module states
3. **Pose Estimation**: Fuse odometry + vision
4. **Logging**: Record all states for replay

### Module.java Responsibilities

1. **Setpoint Tracking**: Receive target speed/angle
2. **Optimization**: Calculate shortest rotation path
3. **Motor Control**: Command drive and turn motors
4. **Feedback**: Report actual speed/angle

### IO Layer Pattern

```java
// Interface defines what we need
public interface ModuleIO {
    void updateInputs(ModuleIOInputs inputs);
    void setDriveVelocity(double metersPerSec);
    void setTurnPosition(Rotation2d angle);
}

// Real implementation talks to hardware
public class ModuleIOTalonFX implements ModuleIO {
    // Uses Phoenix 6 API
}

// Sim implementation uses physics
public class ModuleIOSim implements ModuleIO {
    // Uses WPILib simulation
}
```

This separation allows:
- Running the same logic in simulation
- Replaying logged matches
- Swapping hardware without changing control code

---

## Tuning and Calibration

### CANcoder Offsets

Each module's CANcoder has an offset to define "zero":

1. Point all wheels forward manually
2. Read CANcoder values
3. Set offsets so reported angle is 0°

### Drive PID Tuning

1. Start with kV from motor characterization
2. Add kP until response is snappy but not oscillating
3. Add kD if there's overshoot

### Turn PID Tuning

1. Set kP high enough to reach setpoint quickly
2. Add kD to reduce overshoot
3. Ensure kS overcomes static friction

### SysId Characterization

WPILib's SysId tool can automatically find feedforward constants:

1. Run quasistatic test (slow acceleration)
2. Run dynamic test (fast acceleration)
3. Analyze data to extract kS, kV, kA

### Common Issues

| Symptom | Likely Cause |
|---------|--------------|
| Wheels oscillate | kP too high or kD too low |
| Slow response | kP too low |
| Drift to one side | Wheel offsets incorrect |
| Jerky motion | Feedforward values wrong |
| Wheels fight each other | Kinematics constants wrong |

---

## Related Documentation

- [OVERVIEW.md](OVERVIEW.md) - Project architecture
- [VISION.md](VISION.md) - How vision updates pose estimation
- [PathPlanner Docs](https://pathplanner.dev/) - Autonomous path following
