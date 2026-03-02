# Intake and Feeder Subsystems

This document explains the game piece handling systems that collect fuel from the field and deliver it to the launcher.

## Table of Contents

1. [System Overview](#system-overview)
2. [Intake Subsystem](#intake-subsystem)
3. [Feeder Subsystem](#feeder-subsystem)
4. [Pneumatics Basics](#pneumatics-basics)
5. [Code Structure](#code-structure)
6. [Common Issues](#common-issues)

---

## System Overview

The fuel handling system forms a pipeline from field to launcher:

```
    Field          Intake         Feeder         Launcher
      ○    →    ┌─────────┐    ┌─────────┐    ┌─────────┐
    (fuel)      │ Rollers │ →  │Spindexer│ →  │ Flywheel│ → Target
                │  + Arm  │    │ + Kicker│    └─────────┘
                └─────────┘    └─────────┘
```

### Flow Sequence

1. **Intake deploys** (pneumatic arm extends)
2. **Intake rollers spin** (pull fuel into robot)
3. **Spindexer receives fuel** (rotates to stage it)
4. **Kicker feeds launcher** (pushes fuel into flywheel)
5. **Launcher shoots** (fuel exits robot)

---

## Intake Subsystem

### Purpose

Collect game pieces (fuel) from the field and transfer them into the robot's internal storage.

### Mechanical Design

```
    Side View - Stowed              Side View - Deployed

    ┌────────────────┐              ┌────────────────┐
    │     Robot      │              │     Robot      │
    │                │              │                │
    │    ┌──────┐    │              │    ┌──────┐    │
    │    │Intake│    │              │    │      │    │
    │    │(up)  │    │              │    │      │    │
    │    └──────┘    │              │    │      │╲   │
    │                │              │    └──────┘ ╲  │
    └────────────────┘              └───────────○─╲─┘
                                              Rollers
                                              (on ground)
```

### Components

#### Intake Arm (Pneumatic)

**Purpose**: Moves intake rollers between stowed and deployed positions.

**Hardware**:
- **Actuator**: Pneumatic cylinder
- **Control**: Double solenoid (REV Pneumatic Hub)
- **Positions**: Extended (deployed) or retracted (stowed)

**Why Pneumatic?**
- Binary positions (no precise control needed)
- High force for fast deployment
- Simple control (on/off)
- Holds position without power

#### Intake Rollers

**Purpose**: Grab fuel and pull it into the robot.

**Hardware**:
- **Motors**: 2× Kraken X60 (TalonFX)
- **Configuration**: Running in parallel
- **Surface**: Compliant wheels (grippy)

**Operation**:
- **Forward**: Pull fuel in (intake)
- **Reverse**: Push fuel out (eject/outtake)
- **Speed**: Typically 50-80% power

### Control Strategy

The intake is typically controlled with a state machine:

```
        ┌─────────┐
        │  IDLE   │
        └────┬────┘
             │ Driver presses intake button
             ▼
        ┌─────────┐
        │ DEPLOY  │ ← Extend arm
        └────┬────┘
             │ Arm extended
             ▼
        ┌─────────┐
        │ RUNNING │ ← Spin rollers
        └────┬────┘
             │ Driver releases button
             ▼
        ┌─────────┐
        │ RETRACT │ ← Retract arm
        └────┬────┘
             │ Arm retracted
             ▼
        ┌─────────┐
        │  IDLE   │
        └─────────┘
```

---

## Feeder Subsystem

### Purpose

Stage collected fuel and deliver it to the launcher on demand.

### Mechanical Design

```
    Top View of Feeder

         ┌───────────────┐
         │   To Launcher │
         │       ↑       │
         │   ┌───────┐   │
         │   │Kicker │   │
         │   └───────┘   │
         │       ↑       │
    ┌────┴───────────────┴────┐
    │                         │
    │    ┌───────────────┐    │
    │    │  Spindexer    │    │
    │    │  (rotating)   │    │
    │    └───────────────┘    │
    │                         │
    └────┬───────────────┬────┘
         │  From Intake  │
         └───────────────┘
```

### Components

#### Spindexer

**Purpose**: A rotating hopper that stages multiple fuel and indexes them toward the kicker.

**Hardware**:
- **Motor**: NEO Vortex (SparkFlex controller)
- **Mechanism**: Rotating drum with dividers

**Operation**:
- Rotates to receive fuel from intake
- Indexes fuel around to the kicker position
- Can hold multiple fuel simultaneously

**Why "Spindexer"?**
It's a portmanteau of "spinning" and "indexer" - it spins to index fuel into position.

#### Kicker

**Purpose**: The final stage that pushes fuel into the launcher flywheel.

**Hardware**:
- **Motor**: NEO Vortex (SparkFlex controller)
- **Mechanism**: Small roller or wheel

**Operation**:
- Activates when ready to shoot
- Pushes one fuel at a time into flywheel
- Must synchronize with flywheel speed

### Control Strategy

Coordinated operation with the launcher:

```
    Shooter Ready?──No──▶ Spindexer: STAGE (slow rotation)
         │                Kicker: STOP
         │
        Yes
         │
         ▼
    Fuel in Position?──No──▶ Spindexer: INDEX (rotate to position)
         │                   Kicker: STOP
         │
        Yes
         │
         ▼
    Spindexer: STOP
    Kicker: FEED (push into flywheel)
```

### Timing Considerations

The kicker must be carefully timed:
- Too early: Flywheel not at speed, weak shot
- Too late: Wastes time, slower cycle
- Continuous: Can cause jams if fuel backs up

---

## Pneumatics Basics

### How Pneumatics Work

Pneumatic systems use compressed air to create linear motion:

```
    Compressor → Tank → Regulator → Solenoid → Cylinder → Motion
                  │
            Pressure Sensor
```

#### Components

**Compressor**: Electric pump that fills the tank with air
- Runs automatically when pressure drops
- Limited runtime per match (to prevent brownouts)

**Tank**: Stores compressed air (typically 120 PSI)

**Regulator**: Reduces pressure to working level (60 PSI typical)

**Solenoid**: Electrically controlled valve
- **Single**: One coil, spring return
- **Double**: Two coils, holds position

**Cylinder**: Converts air pressure to linear force
- **Single-acting**: Air extends, spring retracts
- **Double-acting**: Air both extends and retracts

### REV Pneumatic Hub

Our robot uses the REV Pneumatic Hub (PH):
- Integrated compressor control
- 16 solenoid channels
- Pressure monitoring
- CAN bus communication

### Code Example

```java
// Create solenoid on channel 0
Solenoid intakeArm = new Solenoid(PneumaticsModuleType.REVPH, 0);

// Extend the arm
intakeArm.set(true);

// Retract the arm
intakeArm.set(false);
```

### Air Usage Considerations

Limited air supply per match:
- Compressor can only run so much
- Each actuation uses air
- Plan for worst-case usage

Strategies:
- Minimize unnecessary actuations
- Use single-acting cylinders where possible
- Monitor pressure during matches

---

## Code Structure

### Intake Files

```
subsystems/intake/
├── Intake.java             # Main subsystem
├── IntakeConstants.java    # Configuration
├── RollerIO.java          # Roller hardware interface
├── RollerIOTalonFX.java   # Real roller implementation
├── ArmIO.java             # Arm hardware interface
└── ArmIOSolenoid.java     # Real pneumatic implementation
```

### Feeder Files

```
subsystems/feeder/
├── Feeder.java             # Main subsystem
├── FeederConstants.java    # Configuration
├── SpindexerIO.java       # Spindexer interface
├── SpindexerIOSpark.java  # Real spindexer (SparkFlex)
├── KickerIO.java          # Kicker interface
└── KickerIOSpark.java     # Real kicker (SparkFlex)
```

### Key Methods

#### Intake.java

```java
public class Intake extends SubsystemBase {

    public void deploy() {
        armIO.extend();
    }

    public void retract() {
        armIO.retract();
    }

    public void runForward() {
        rollerIO.setVoltage(intakeVoltage);
    }

    public void runReverse() {
        rollerIO.setVoltage(-outtakeVoltage);
    }

    public void stop() {
        rollerIO.setVoltage(0);
    }

    public boolean isDeployed() {
        return armIO.isExtended();
    }
}
```

#### Feeder.java

```java
public class Feeder extends SubsystemBase {

    public void runSpindexer(double speed) {
        spindexerIO.setVelocity(speed);
    }

    public void runKicker(double speed) {
        kickerIO.setVelocity(speed);
    }

    public void feed() {
        runSpindexer(feedSpeed);
        runKicker(feedSpeed);
    }

    public void stop() {
        spindexerIO.setVoltage(0);
        kickerIO.setVoltage(0);
    }
}
```

### Command Composition

Typical intake command:

```java
public Command intakeCommand() {
    return Commands.sequence(
        // Deploy intake
        Commands.runOnce(() -> intake.deploy()),
        // Wait for deployment
        Commands.waitSeconds(0.1),
        // Run intake until button released
        Commands.run(() -> {
            intake.runForward();
            feeder.runSpindexer(indexSpeed);
        }).finallyDo(() -> {
            intake.stop();
            intake.retract();
        })
    );
}
```

Typical shooting sequence:

```java
public Command shootCommand() {
    return Commands.sequence(
        // Wait for launcher ready
        Commands.waitUntil(() -> launcher.isOnTarget()),
        // Feed one fuel
        Commands.run(() -> feeder.feed())
            .withTimeout(0.5),
        // Stop feeder
        Commands.runOnce(() -> feeder.stop())
    );
}
```

---

## Common Issues

### Intake Problems

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| Arm won't deploy | No air pressure | Check compressor, tank |
| Arm slow | Low pressure | Wait for compressor |
| Rollers don't spin | Motor disconnected | Check CAN wiring |
| Fuel bounces out | Rollers too fast | Reduce intake speed |
| Fuel jams | Rollers too slow | Increase intake speed |

### Feeder Problems

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| Fuel won't index | Spindexer jammed | Clear obstruction |
| Double feeds | Kicker too fast | Reduce kicker speed |
| Weak shots | Kicker timing wrong | Adjust feed delay |
| Fuel backs up | Shooting too slow | Increase shot rate |

### Pneumatic Problems

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| No actuation | Compressor off | Enable in code |
| Weak actuation | Low pressure | Check for leaks |
| Slow actuation | Flow restriction | Check tubing |
| Won't hold position | Solenoid stuck | Replace solenoid |

---

## Related Documentation

- [OVERVIEW.md](OVERVIEW.md) - Project architecture
- [LAUNCHER.md](LAUNCHER.md) - Where fuel goes after feeder
