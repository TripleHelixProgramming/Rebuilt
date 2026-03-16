# Rebuilt — FRC 2363 2026 Robot Code
[![CI](https://github.com/TripleHelixProgramming/Rebuilt/actions/workflows/build.yml/badge.svg)](https://github.com/TripleHelixProgramming/Rebuilt/actions/workflows/build.yml)

Java robot code for the 2026 FRC season, built on [WPILib](https://docs.wpilib.org) and [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit).

---

## Requirements

- [WPILib 2026](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) (includes Java 17 and VS Code extensions)
- A roboRIO-connected robot or simulation environment

---

## Building

Use the Gradle wrapper — no separate Gradle installation needed.

**Build (compile + check):**
```bash
./gradlew build
```

**Deploy to robot:**
```bash
./gradlew deploy
```

**Run in simulation:**
```bash
./gradlew simulateJava
```

---

## Code Formatting

This project uses [Spotless](https://github.com/diffplug/spotless) with Google Java Format. Formatting is applied automatically on every build.

**Apply formatting manually:**
```bash
./gradlew spotlessApply
```

**Check formatting without modifying files:**
```bash
./gradlew spotlessCheck
```

> CI will fail if `spotlessCheck` does not pass. Always run `spotlessApply` before pushing.

---

## AdvantageScope Custom Assets

The `ascope-assets/` directory contains custom robot models and camera configurations for [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope).

**To enable them in AdvantageScope:**

1. Open AdvantageScope.
2. Go to **Help → Show App Directory** (or press `Cmd/Ctrl+Shift+.`).
3. In your AdvantageScope settings (or via **File → Preferences**), set the **Custom Assets** folder to the `ascope-assets/` directory in this repository.
   - Example path: `/path/to/Rebuilt/ascope-assets`
4. Restart AdvantageScope. The robot model and camera views will appear in the 3D field viewer.

---

## Utility Controls

### Align Encoders

Resets the swerve drive absolute encoders. This can be triggered while the robot is disabled.

1. Open [Glass](https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/index.html).
2. Go to **NetworkTables → Triggers → Align Encoders**.
3. Toggle the value to `true`.

---

## Contributing

1. Fork or branch from `main`.
2. Run `./gradlew build` locally to verify your changes compile and pass formatting.
3. Open a pull request targeting `main`. CI will run a build check and Spotless formatting check automatically.
4. Keep PRs focused — one feature or fix per PR.
