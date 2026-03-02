# Project Overview

This document explains how the different parts of the robot code work together, aimed at helping newcomers understand both the software architecture and how it relates to the physical robot.

## Table of Contents

1. [High-Level Architecture](#high-level-architecture)
2. [The Robot Loop](#the-robot-loop)
3. [Subsystems and Commands](#subsystems-and-commands)
4. [How Data Flows](#how-data-flows)
5. [Match Lifecycle](#match-lifecycle)
6. [Key Files to Know](#key-files-to-know)

---

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                           Robot.java                                │
│                    (Main entry point, runs at 50Hz)                 │
└─────────────────────────────────────────────────────────────────────┘
                                   │
                    ┌──────────────┴──────────────┐
                    ▼                              ▼
         ┌──────────────────┐           ┌──────────────────┐
         │ Command Scheduler│           │   Subsystems     │
         │ (WPILib)         │           │                  │
         └──────────────────┘           └──────────────────┘
                    │                              │
      ┌─────────────┼─────────────┐               │
      ▼             ▼             ▼               ▼
 ┌─────────┐  ┌─────────┐  ┌─────────┐    ┌─────────────┐
 │ Drive   │  │ Launcher│  │ Intake  │    │ IO Layers   │
 │ Commands│  │ Commands│  │ Commands│    │ (Hardware)  │
 └─────────┘  └─────────┘  └─────────┘    └─────────────┘
                                                  │
                                    ┌─────────────┴─────────────┐
                                    ▼                           ▼
                             ┌─────────────┐            ┌─────────────┐
                             │   Motors    │            │   Sensors   │
                             │ (CAN Bus)   │            │ (DIO, CAN)  │
                             └─────────────┘            └─────────────┘
```

### The Three Layers

1. **Robot/Commands Layer** - High-level logic: "score in the hub", "follow this path"
2. **Subsystem Layer** - Mechanism control: "spin flywheel at 3000 RPM", "set hood angle"
3. **IO Layer** - Hardware abstraction: "send this voltage to motor controller #5"

This separation lets us:
- Swap hardware without changing game logic
- Run the same code in simulation
- Replay logged matches for debugging

---

## The Robot Loop

The robot runs a **periodic loop at 50Hz** (every 20ms). Each cycle:

```
1. READ all sensor inputs (encoders, gyro, cameras)
2. RUN the command scheduler (executes active commands)
3. WRITE outputs to motors and actuators
4. LOG everything for later analysis
```

### Why 50Hz?

- Fast enough for responsive control
- Slow enough for the RoboRIO to handle all subsystems
- Matches the driver station update rate

### Critical Timing

Each loop iteration must complete in under 20ms. If it takes longer ("loop overrun"), the robot becomes less responsive. That's why we:
- Run sensor reads in background threads
- Cache values instead of blocking on CAN bus reads
- Avoid complex calculations in the main loop

---

## Subsystems and Commands

### What is a Subsystem?

A **subsystem** represents a physical mechanism on the robot. Each subsystem:
- Owns specific hardware (motors, sensors)
- Has a `periodic()` method called every loop
- Can only run ONE command at a time

Our subsystems:

| Subsystem | Purpose | Key Hardware |
|-----------|---------|--------------|
| **Drive** | Move the robot | 4 swerve modules, gyro |
| **Launcher** | Shoot game pieces | Turret, flywheel, hood |
| **Intake** | Collect game pieces | Rollers, pneumatic arm |
| **Feeder** | Move pieces to launcher | Spindexer, kicker motors |
| **Vision** | See AprilTags | 4 cameras |
| **LEDs** | Driver feedback | Addressable LED strip |

### What is a Command?

A **command** is an action that uses one or more subsystems. Commands:
- Have `initialize()`, `execute()`, `end()`, `isFinished()` methods
- "Require" subsystems (prevents conflicts)
- Can be composed into sequences

Example: "Score in hub" might be:
```
1. AimAtHub command (requires Launcher)
2. Wait until on target
3. Feed command (requires Feeder)
4. Wait until shot
```

### Default Commands

When no other command is using a subsystem, its **default command** runs:
- **Drive**: Joystick control
- **Launcher**: Idle/stop
- **Feeder**: Stop

---

## How Data Flows

### Controller Input → Robot Motion

```
Xbox Controller
      │
      ▼
┌─────────────────┐
│ Joystick values │  (X, Y, rotation: -1.0 to 1.0)
│ with deadband   │
└─────────────────┘
      │
      ▼
┌─────────────────┐
│ DriveCommands   │  Converts to field-relative speeds
│ .joystickDrive()│  using gyro heading
└─────────────────┘
      │
      ▼
┌─────────────────┐
│ Drive subsystem │  Calculates per-module speeds
│ .runVelocity()  │  using swerve kinematics
└─────────────────┘
      │
      ▼
┌─────────────────┐
│ Module          │  Sets motor voltages via PID
│ .runSetpoint()  │  for drive and turn motors
└─────────────────┘
      │
      ▼
┌─────────────────┐
│ TalonFX motors  │  Physical wheel motion
└─────────────────┘
```

### Vision → Pose Estimation

```
AprilTag on field
      │
      ▼
┌─────────────────┐
│ PhotonVision    │  Detects tag, estimates camera pose
│ (coprocessor)   │
└─────────────────┘
      │
      ▼
┌─────────────────┐
│ VisionIO        │  Reads pose observations from NT
│ (background)    │  at 50Hz in separate thread
└─────────────────┘
      │
      ▼
┌─────────────────┐
│ Vision subsystem│  Filters observations, calculates
│                 │  standard deviations
└─────────────────┘
      │
      ▼
┌─────────────────┐
│ Drive subsystem │  Fuses vision with odometry
│ pose estimator  │  using Kalman filter
└─────────────────┘
```

### Launcher Ballistics

```
Target position (hub)
      │
      ▼
┌─────────────────┐
│ Launcher.aim()  │  Calculates initial velocity needed
│                 │  using projectile motion equations
└─────────────────┘
      │
      ├──────────────────────────────────┐
      ▼                                  ▼
┌─────────────────┐            ┌─────────────────┐
│ Flywheel speed  │            │ Hood angle      │
│ setpoint        │            │ setpoint        │
└─────────────────┘            └─────────────────┘
      │                                  │
      ▼                                  ▼
┌─────────────────┐            ┌─────────────────┐
│ FlywheelIO      │            │ HoodIO          │
│ PID control     │            │ PID control     │
└─────────────────┘            └─────────────────┘
```

---

## Match Lifecycle

### Pre-Match (Disabled)

```
Robot.disabledPeriodic() runs continuously:
  - Update alliance selection
  - Update auto mode selection
  - Display selection on LEDs
  - Run vision to update pose
  - Allow driver to position robot (pose-seek LEDs)
```

### Autonomous (20 seconds)

```
Robot.autonomousInit():
  - Get selected auto routine
  - Reset pose to starting position
  - Schedule auto command sequence

Robot.autonomousPeriodic():
  - Command scheduler runs auto routine
  - Typically: drive paths + intake + shoot
```

### Teleoperated (2:20)

```
Robot.teleopInit():
  - Cancel any running commands
  - Set default commands

Robot.teleopPeriodic():
  - Default commands run (joystick drive)
  - Button presses trigger other commands
  - Game state updates (hub countdown)
```

### Test Mode

Used for individual mechanism testing and calibration.

---

## Key Files to Know

### Entry Points

| File | Purpose |
|------|---------|
| `Main.java` | JVM entry point, starts Robot |
| `Robot.java` | Robot lifecycle, subsystem creation |
| `Constants.java` | All hardware constants and port mappings |

### Subsystems

| File | Purpose |
|------|---------|
| `Drive.java` | Swerve drive control and pose estimation |
| `Launcher.java` | Turret, flywheel, hood coordination |
| `Vision.java` | Camera management and pose updates |

### Commands

| File | Purpose |
|------|---------|
| `DriveCommands.java` | All drive-related commands |
| `PathCommands.java` | Autonomous path following |

### Auto Routines

| File | Purpose |
|------|---------|
| `AutoMode.java` | Base class for all autos |
| `B_*.java` / `R_*.java` | Alliance-specific auto routines |

### Configuration

| File | Purpose |
|------|---------|
| `DriveConstants.java` | Swerve geometry, PID gains |
| `LauncherConstants.java` | Ballistics parameters |
| `VisionConstants.java` | Camera positions |

---

## Next Steps

For deeper understanding, see the subsystem-specific documentation:

- [DRIVE.md](DRIVE.md) - Swerve drive and odometry
- [LAUNCHER.md](LAUNCHER.md) - Ballistics and shooting
- [INTAKE_FEEDER.md](INTAKE_FEEDER.md) - Game piece handling
- [VISION.md](VISION.md) - AprilTag detection and pose estimation
- [LED.md](LED.md) - LED patterns and driver feedback
