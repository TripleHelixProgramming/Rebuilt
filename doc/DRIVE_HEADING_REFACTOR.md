# Drive Heading Refactor

## What was wrong

The `Drive` subsystem conflated two unrelated things: knowing where the robot is on the field (pose estimation) and knowing which direction the driver considers "forward" (a joystick mapping preference). The driver's "forward" concept was implemented as a `headingOffset` field inside `Drive`, which created a second rotation that competed with the pose estimator's rotation. Different parts of the code picked whichever one they wanted, and when they disagreed, things went wrong.

Concretely, this caused three problems:

### 1. `setPose()` didn't set the pose

It only adjusted `headingOffset`. The X/Y position was never set. This meant auto routines relied entirely on vision to establish field position before the match started. When vision couldn't converge (e.g., only seeing distant single tags with high ambiguity), the pose stayed near origin and the path follower saw a 12-meter error, saturated its output, and sent the robot at full speed in an arbitrary direction. This is what happened in E4.

### 2. `followTrajectory()` had a frame mismatch

The path follower computed position corrections using `getPose()` (vision estimator rotation), then converted those corrections from field-relative to robot-relative using `getHeading()` (gyro + offset). These were two different rotations. When they disagreed, the corrections got rotated into the wrong frame and the robot drove in the wrong direction.

How this works: `fromFieldRelativeSpeeds()` takes velocities expressed in the field frame and rotates them into the robot frame. It needs to know which direction the robot is facing to do that rotation. If the PID says "drive +2 m/s in field-Y" and the conversion uses a heading that's 45° off from reality, the robot drives diagonally instead of sideways.

A related inconsistency existed in `joystickDriveAtFixedOrientation()`: the angular PID used `getPose().getRotation()` as its process variable, but reset its initial state from `getHeading()`. If the two rotations disagreed, the PID started from the wrong value.

### 3. `wheelRadiusCharacterization()` used the wrong rotation

It measured cumulative gyro delta using `getHeading()` rather than `getRawGyroRotation()`. Previously this happened to work because `getHeading()` was based on the raw gyro (plus a constant offset), so vision updates didn't affect it. But with `getHeading()` now returning the pose estimator's rotation, a vision correction mid-characterization would corrupt the measurement. The fix is to use `getRawGyroRotation()` directly, which is correct regardless of how `getHeading()` is implemented.

## What changed

We separated the two concerns cleanly: **Drive owns the robot's physical state, DriveCommands owns the human interface.**

### In Drive: one pose, one rotation, no overlays

- Removed the `headingOffset` field.
- `setPose(Pose2d pose)` resets the full pose estimator via `visionPose.resetPosition(rawGyroRotation, getModulePositions(), pose)`. It does what it says.
- `getHeading()` returns `getPose().getRotation()`. One source of truth.
- `followTrajectory()` uses `getPose().getRotation()` for both error computation and field-to-robot conversion. No more frame mismatch.
- In `DriveCommands`, `wheelRadiusCharacterization()` uses `getRawGyroRotation()` since it's measuring a physical quantity and shouldn't be affected by vision updates.
- `addVisionMeasurement()` on first vision estimate while disabled: resets the full pose from vision instead of just the heading offset. This is actually better — the robot gets both position and heading initialized from vision rather than just heading.

### In DriveCommands: driver forward direction

The driver's notion of "forward" now lives here. It's a static field that only affects how joystick inputs are interpreted:

```java
private static Rotation2d driverForwardDirection = Rotation2d.kZero;

public static void resetDriverForward(Drive drive) {
    driverForwardDirection = drive.getPose().getRotation();
}
```

A private helper computes the heading for field-relative joystick conversion, incorporating both the driver forward offset and the `fieldRotated` alliance flip:

```java
private static Rotation2d getDriverRelativeHeading(Drive drive, boolean fieldRotated) {
    Rotation2d heading = drive.getPose().getRotation().minus(driverForwardDirection);
    return fieldRotated ? heading.plus(Rotation2d.kPi) : heading;
}
```

The driver's reset button calls `DriveCommands.resetDriverForward()` instead of `drive.resetHeading()`.

## Impact on the rest of the code

We audited every caller of `getHeading()`, `getPose()`, `resetHeading()`, `setPose()`, and `fromFieldRelativeSpeeds()` across the codebase. The impact is small and localized:

- **Vision system**: unaffected. It already uses `getRawGyroRotation()` for observation scoring and feeds measurements through `addVisionMeasurement()`.
- **Pose estimator internals**: unaffected. It already uses `rawGyroRotation` in its update loop.
- **PathCommands** (goToTargetPose, dockToTargetPoint, etc.): unaffected. They use `getPose()` for position, which doesn't change.
- **Auto routines**: gain a working `setPose()`. Both Choreo's `resetOdometry()` and PathPlanner's `AutoBuilder` call `setPose()` — both now actually reset the robot's position to the trajectory start.
- **GameState targeting**: uses `getPose()`, unaffected.
- **LED display**: uses `getPose()`, unaffected.

The only files that changed:

| File | What changed |
|------|-------------|
| `Drive.java` | Removed `headingOffset`, fixed `setPose()`, `getHeading()`, `followTrajectory()`, `addVisionMeasurement()` |
| `DriveCommands.java` | Added `driverForwardDirection`, `resetDriverForward()`, and `getDriverRelativeHeading()` helper; updated joystick commands to use them; fixed `wheelRadiusCharacterization()` to use raw gyro |
| `Robot.java` | Updated heading reset button bindings to call `DriveCommands.resetDriverForward()` |

## What we gained

- **Auto routines start with a known pose.** No more depending on vision to converge before the match. If vision later provides a better estimate, the Kalman filter incorporates it normally.
- **Trajectory following uses a consistent frame.** Position error and velocity conversion use the same rotation. The robot drives where the path follower intends.
- **Functions do what their names say.** `setPose()` sets the pose. `getHeading()` returns the heading. No hidden side channels.
- **One rotation source of truth in Drive.** No more guessing whether to call `getHeading()` or `getPose().getRotation()` — they're the same thing.
- **Driver controls work identically.** The driver forward reset button still works exactly as before. The implementation just moved from Drive (where it didn't belong) to DriveCommands (where it does).
