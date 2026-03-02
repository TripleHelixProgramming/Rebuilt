# FRC Team 2363 Robot Code (2026 Season)

This repository contains the robot code for FRC Team 2363's 2026 competition robot.

## Quick Start

### Prerequisites

- [WPILib 2026](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
- Java 17 (included with WPILib)
- Git

### Build

```bash
./gradlew build
```

### Deploy to Robot

```bash
./gradlew deploy
```

### Run Simulation

```bash
./gradlew simulateJava
```

### Format Code

```bash
./gradlew spotlessApply
```

## Project Structure

```
src/main/java/frc/
├── robot/
│   ├── Robot.java          # Main robot class, subsystem instantiation
│   ├── Constants.java      # Hardware port mappings, motor constants
│   ├── auto/               # Autonomous routines (Choreo-based)
│   ├── commands/           # Drive and path commands
│   ├── subsystems/
│   │   ├── drive/          # Swerve drive (4 modules, gyro, odometry)
│   │   ├── launcher/       # Turret, flywheel, hood with ballistics
│   │   ├── feeder/         # Spindexer and kicker
│   │   ├── intake/         # Rollers and pneumatic arm
│   │   ├── vision/         # 4-camera AprilTag pose estimation
│   │   └── leds/           # Addressable LED feedback
│   └── util/               # Background threads, utilities
├── lib/                    # Controller abstraction, auto selection
└── game/                   # Field geometry, game state tracking
```

## Subsystems

| Subsystem | Purpose | Key Hardware |
|-----------|---------|--------------|
| **Drive** | Swerve locomotion | 4x TalonFX modules, CANcoders, Canandgyro |
| **Launcher** | Projectile scoring | Turret (SparkMax), Flywheel (TalonFX), Hood (SparkMax) |
| **Intake** | Game piece collection | Dual TalonFX rollers, pneumatic arm |
| **Feeder** | Game piece indexing | Spindexer + Kicker (SparkMax) |
| **Vision** | Field localization | 4x PhotonVision cameras |
| **LEDs** | Driver feedback | Addressable LED strips |

## Architecture

This codebase uses the **AdvantageKit** logging framework with an IO abstraction pattern:

- **REAL mode**: Runs on physical robot hardware
- **SIM mode**: Physics simulation for development
- **REPLAY mode**: Log file playback for debugging

Each subsystem has:
- An interface (e.g., `FlywheelIO`)
- Real implementation (e.g., `FlywheelIOTalonFX`)
- Simulation implementation (e.g., `FlywheelIOSimTalonFX`)

This allows the same high-level code to run on hardware, in simulation, or replaying logs.

## Hardware Configuration

### CAN Bus Layout

**RoboRIO CAN (CAN2):**
| ID | Device |
|----|--------|
| 0 | Gyroscope (Canandgyro) |
| 12-13 | Turret, Hood (SparkMax) |
| 14-15 | Flywheel Leader/Follower (TalonFX) |
| 16-17 | Spindexer, Kicker (SparkMax) |
| 22-23 | Intake Roller Leader/Follower (TalonFX) |

**CANivore (CANHD):**
| ID | Device |
|----|--------|
| 10, 18, 20, 28 | Drive Motors (TalonFX) |
| 11, 19, 21, 29 | Turn Motors (TalonFX) |
| 31, 33, 43, 45 | Turn Encoders (CANcoder) |

### Digital IO

| Port | Function |
|------|----------|
| 0-2 | Autonomous mode selector (3-bit) |
| 3 | Alliance color selector |
| 4 | Turret absolute encoder |

### Controller Ports

| Port | Controller |
|------|------------|
| 0 | Driver (Zorro or Xbox) |
| 1 | Operator (Xbox) |

## Autonomous Modes

Eight autonomous routines available via DIO selector:

| Selector | Blue Alliance | Red Alliance |
|----------|---------------|--------------|
| 1 | Left Trench | Left Trench |
| 2 | Right Trench | Right Trench |
| 3 | Depot | Depot |
| 4 | Outpost | Outpost |

Trajectories are generated using [Choreo](https://sleipnirgroup.github.io/Choreo/) and stored in `src/main/deploy/choreo/`.

## Controls

### Driver (Zorro Controller)

| Input | Action |
|-------|--------|
| Right Stick | Translation (X/Y) |
| Left Stick X | Rotation |
| Switch E | Field-relative toggle |
| Button G | Reset gyro heading |
| Button H | Deploy intake |
| Button A | Desaturate turret + advance feeder |
| Left Dial > 0.5 | Enable launcher aiming |

### Driver (Xbox Fallback)

| Input | Action |
|-------|--------|
| Left Stick | Translation (X/Y) |
| Right Stick X | Rotation |
| Left Bumper (hold) | Robot-relative mode |
| B Button | Reset gyro heading |
| X Button | Stop wheels (X pattern) |

### Operator (Xbox)

| Input | Action |
|-------|--------|
| B Button | Deploy intake |
| Y Button | Reverse intake |
| A Button | Advance feeder |
| X Button | Reverse feeder |
| Right Bumper | Desaturate turret + advance |

## Logging

Logs are saved to:
- **USB drive**: `/U/logs` (on robot)
- **NetworkTables**: Real-time via NT4Publisher

View logs using [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope).

### Replay Mode

To replay a log file:

```bash
./gradlew replayWatch
```

Or set `Constants.simMode = Mode.REPLAY` and run simulation.

## Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| WPILib | 2026 | Core FRC framework |
| AdvantageKit | 26.x | Logging and replay |
| Phoenix 6 | 26.x | TalonFX, CANcoder |
| REVLib | 2026.x | SparkMax controllers |
| PhotonVision | 2026.x | Vision processing |
| PathPlanner | 2026.x | Path planning |
| ChoreoLib | 2026.x | Trajectory following |

## Documentation

Detailed subsystem documentation is available in the `/doc` directory:

- [OVERVIEW.md](doc/OVERVIEW.md) - High-level architecture
- [DRIVE.md](doc/DRIVE.md) - Swerve drive system
- [LAUNCHER.md](doc/LAUNCHER.md) - Ballistics and shooting
- [INTAKE_FEEDER.md](doc/INTAKE_FEEDER.md) - Game piece handling
- [VISION.md](doc/VISION.md) - AprilTag pose estimation
- [LED.md](doc/LED.md) - LED patterns
- [CODE_REVIEW.md](doc/CODE_REVIEW.md) - Code review notes

## Development

### Code Style

This project uses [Spotless](https://github.com/diffplug/spotless) with Google Java Format. Formatting is automatically applied on build.

To manually format:

```bash
./gradlew spotlessApply
```

### Event Branches

When deploying from a branch starting with `event`, changes are automatically committed with a timestamp. This helps track code versions deployed at competitions.

### VisualVM Profiling

JMX is enabled on the robot for remote profiling:
- Host: `10.23.63.2`
- Port: `1198`

## License

This project incorporates code from:
- [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit) - See [AdvantageKit-License.md](AdvantageKit-License.md)
- [WPILib](https://github.com/wpilibsuite/allwpilib) - See [WPILib-License.md](WPILib-License.md)
