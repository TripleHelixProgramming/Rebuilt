# Vision Calibration and Validation

The pose estimator produces a guess about where the robot is on the field. This document covers how to make that guess as close to physical reality as possible, and — just as importantly — how to detect when it's wrong.

There are four links in the chain from "light hits the camera sensor" to "robot pose on the field." Any one of them being wrong makes the whole estimate wrong. They are:

1. **Camera intrinsics** — the camera's internal model (focal length, distortion)
2. **Camera extrinsics** — where the camera is mounted on the robot
3. **AprilTag field layout** — where the tags physically are on the field
4. **Pose estimation pipeline** — the software that turns tag detections into a robot pose

## 1. Camera intrinsics (calibration)

Camera calibration estimates the lens's focal length, optical center, and distortion coefficients. PhotonVision uses OpenCV's calibration pipeline with a chessboard pattern.

### Why it matters

Bad calibration is the highest-leverage source of error. It affects every observation from that camera, at every distance, in every match. A poorly calibrated camera at 4 meters can produce 10-15 cm of systematic pose error that no amount of software filtering can fix — the raw measurement is wrong.

### How to do it well

**Print the calibration board on something rigid.** Glue it to foam board, MDF, or a clipboard. A floppy sheet of paper introduces calibration error from the board warping between shots.

**Calibrate at your operating resolution.** If you run at 1280x720, calibrate at 1280x720. Changing resolution after calibration invalidates it.

**Take 25+ images with full coverage.** This is where most teams go wrong. The images need to:
- Cover the entire frame — corners and edges matter most because that's where lens distortion is worst. If all your images have the board in the center, your edge distortion coefficients will be garbage.
- Vary the board angle significantly — tilt it 30-45° toward and away from the camera, rotate it. Flat/parallel images don't constrain all the distortion parameters.
- Vary the distance — some close (board fills ~half the frame), some far (board fills ~1/5).

**Check the reprojection error.** PhotonVision reports this after calibration. Target under 0.5 pixels. Under 0.3 is excellent. Above 1.0 means something went wrong.

### When to recalibrate

- After changing the camera lens, focus, or resolution
- After the camera is physically disturbed (remounted, robot dropped, bumper impact near the camera)
- At minimum, once per build season
- The OV2311 sensors have fixed-focus lenses, so they won't drift from auto-focus changes — that's one less thing to worry about

## 2. Camera extrinsics (mounting)

The `robotToCamera` transforms in `VisionConstants.java` tell the system where each camera is relative to the robot's center. Getting these right is critical because angular errors scale with distance.

### Error sensitivity

| Error Type | At 1 m | At 2 m | At 4 m |
|-----------|--------|--------|--------|
| 1 cm position error | ~1 cm | ~1 cm | ~1 cm |
| 1° angle error | ~1.7 cm | ~3.5 cm | ~7 cm |
| 2° angle error | ~3.5 cm | ~7 cm | ~14 cm |
| 5° angle error | ~8.7 cm | ~17 cm | ~35 cm |

**Angular errors are far more damaging than position errors, and they get worse with distance.** Pitch is the most critical axis because it directly affects estimated distance to the tag. A 2° pitch error at 4 meters causes 14 cm of range error.

### How to measure them

**From CAD (best):** If the robot is fully modeled, extract the camera mount positions and angles from the model. This gives sub-millimeter position accuracy and sub-degree angle accuracy, assuming the physical build matches CAD.

**Physical measurement:** Use calipers for position (measure from the center of the drivebase at floor level to each camera's front lens element). Use a digital level or angle finder for pitch. A smartphone inclinometer works if nothing else is available. Get pitch within 1° and position within 5 mm.

### Your current transforms

```
Front cameras: (-10.572", ±12.337", 16.688") — 20° pitch, 55° yaw
Back cameras:  (-13.162", ±12.162", 20.267") — 15° pitch, 135° yaw
```

These are specified as quaternions, which is correct but hard to verify by inspection. If you ever suspect a mounting error, the easiest validation is the single-camera isolation test described below.

### Software refinement

Some teams use a "calibrate by driving" approach: place the robot at a precisely known position, log each camera's individual pose estimate, compare to ground truth, and adjust the transforms to minimize the offset. Your CAD values get you 90% there; field-based refinement corrects the last degree or centimeter.

## 3. AprilTag field layout

The pose estimate is only as good as the tag positions it's computed against. If the software thinks tag 1 is at (13.0, 0.6) but it's physically at (13.02, 0.58), every pose computed from that tag is off by 2 cm.

### How accurate are competition fields?

FIRST's field assembly specifications allow roughly ±0.5 inches (12.7 mm) for tag placement. In practice, the actual variance depends on the event:
- **Tags on the field perimeter walls** are generally more accurate — the walls are rigid structures.
- **Tags on game-specific elements** (speaker, amp, stage in 2024; reef in 2025; hub in 2026) have more variation because those elements are assembled per-event by volunteers.
- **Championship fields** are typically the most precise. Regional/district fields vary.
- **Practice fields** can be significantly off, especially for custom-built practice elements.

A 1-inch tag placement error at 4 meters translates to roughly 1-2 cm of robot pose error. Multi-tag solutions are more robust because the errors partially cancel when multiple tags are visible.

### What you can do about it

You already have infrastructure for custom field layouts (`customAprilTagLayoutPath` and `useCustomAprilTagLayout` in VisionConstants). If you measure tag positions at a venue and find systematic offsets, you can deploy a corrected layout JSON.

The practical challenge is time: at a competition, you have limited practice time and measuring tag positions precisely is tedious. But if you notice a systematic offset in your pose estimates during practice matches (e.g., the robot consistently thinks it's 3 cm left of where it actually is), a custom layout may be worth the effort.

## 4. Validation procedures

None of the above matters if you can't tell whether your pose estimate is accurate. Validation is the process of comparing what the robot thinks against physical reality.

### Pre-season (do once after building the robot)

**Single-camera isolation test:** This is the most important validation you can do. Place the robot at a precisely known position (e.g., aligned to a field marking). Enable only one camera at a time (you can do this by temporarily commenting out the others in Robot.java or adding a feature flag). Log each camera's estimated pose individually. All four cameras should agree within 3 cm. If one disagrees, that camera's calibration or extrinsics are wrong.

**Distance accuracy test:** Place the robot at a known distance from a tag (use a tape measure). Start at 1 m and move back in 0.5 m increments to 4 m. At each distance, compare the estimated pose to the tape measure. If accuracy degrades sharply with distance, the camera calibration (intrinsics) is the likely culprit. If there's a consistent offset at all distances, the extrinsics are off.

**Rotation test:** Spin the robot in place and compare vision yaw to gyro yaw. Your `yawConsistency` filter already does this at runtime, but doing it deliberately in a controlled setting helps isolate camera-specific issues. If one camera's yaw consistently disagrees with the gyro, that camera's yaw mounting angle is wrong.

### At each event

**Wall test (30 seconds):** Place the robot flat against a known field wall. The pose estimator should report a position that's one half-robot-width away from the wall. This is a quick sanity check that catches gross errors. Do it against two perpendicular walls to check both X and Y.

**Practice match review:** After your first practice match, open the log in AdvantageScope and check:
- Does `RobotPosesAccepted` track smoothly?
- Are the accepted poses clustered where the robot actually was, or scattered?
- Is `bestReprojError` similar to what you see on your practice field? A spike suggests the competition field tags are placed differently.

**Cross-camera correlation check:** In AdvantageScope, look at `FusedCameraCount`. If it's consistently 1 (no fusion), cameras are disagreeing with each other — either the correlation threshold is too tight or a camera is miscalibrated. If you were getting fusion on your practice field but not at competition, something changed.

### Between matches

**Quick log review:** After each match, glance at the vision log. Look for:
- Periods with mostly rejected poses (camera issue or field obstruction)
- Periods where the pose estimate was clearly wrong (check against the robot's actual trajectory)
- Camera disconnection alerts

### If you detect a problem

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| All cameras off in the same direction | Field tag positions differ from layout | Measure tags, deploy custom layout JSON |
| One camera disagrees with others | That camera's mount shifted or calibration is bad | Re-check mount, re-calibrate |
| Accuracy degrades with distance | Camera intrinsic calibration is poor | Recalibrate with better image coverage |
| Pose estimate rotates relative to reality | Camera pitch or yaw angle is wrong | Re-measure mount angle with digital level |
| Reprojection error high at this event but not at home | Field assembly variance | Custom layout or wider filter tolerances |
| No multi-camera fusion | Cameras disagree by more than 0.15 m | Check all four cameras individually |

## 5. The height test as a calibration diagnostic

Your `VisionFilter` includes a `heightError` test that rejects observations where the estimated robot Z-coordinate exceeds ±0.25 m. Since the robot doesn't fly, the Z-coordinate should always be near zero. A consistently nonzero Z is a strong signal that something is wrong:

- **Positive Z (robot thinks it's above the ground):** camera pitch angle is over-estimated, or the camera height in the extrinsics is too high
- **Negative Z (robot thinks it's below the ground):** camera pitch is under-estimated, or camera height is too low
- **Z varies with which tag is being observed:** tag-specific placement error on the field, or the camera calibration has view-dependent distortion issues

Logging the Z-coordinate of accepted observations over a match is a cheap but powerful diagnostic. If it's consistently +5 cm, your camera pitch is probably off by about 1-2°.

## Where to start

1. **Verify camera calibration quality** — check reprojection error for all four cameras. If any is above 0.5 px, recalibrate with better image coverage.
2. **Run the single-camera isolation test** — this catches the most common problems (bad calibration, wrong extrinsics) in one step.
3. **Do the wall test at every event** — 30 seconds, catches gross errors.
4. **Log and review Z-coordinates** — free diagnostic for pitch/height errors.
5. **Establish a practice match review habit** — glance at AdvantageScope after every practice match.

These procedures don't require any code changes. They're about building a validation habit that catches problems before they cost you a match.
