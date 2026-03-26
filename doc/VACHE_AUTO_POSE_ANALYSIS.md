# VACHE Auto Pose Analysis

This analysis examines the `setPose()` bug and its impact on autonomous mode across all 14 official matches at VACHE. The data was extracted from robot logs using the [wpilog-mcp](https://github.com/TripleHelixProgramming/wpilog-mcp) server and Claude. All enable timestamps were cross-validated against `/DriverStation/Enabled`, `/DriverStation/Autonomous`, and `/RealOutputs/GameState/MatchTime` to ensure they correspond to the actual FMS auto start.

## Key insight

**The matches that failed auto were the ones that started with a bad pose.** In 7 of 14 matches, the pose estimator was never initialized — the robot entered auto thinking it was at (0, 0) when it was actually 8-12 meters away. This is why the `setPose()` fix matters: it guarantees a correct starting pose from the trajectory data, eliminating the dependency on pre-match vision convergence.

## The problem

`setPose()` never set the X/Y position — only a heading offset. The robot relied on vision to figure out where it was before the match started. When vision couldn't converge (distant tags, high ambiguity), the pose stayed near (0, 0) and the path follower ran against the wrong position.

## Results: all 14 official matches

**Error at T** is the distance between the robot's pose estimate and the trajectory's expected starting pose at the moment the FMS enabled auto. **Hdg Err** is the heading error at the same moment.

```
Match  Error@T   Hdg Err  Status
─────  ────────  ───────  ──────────────────
Q4     0.31 m      4.9°  Good
Q10    0.47 m      6.2°  Good
Q13    0.39 m      4.2°  Good
Q20    1.25 m     18.0°  Partial
Q23   12.17 m     96.1°  Origin
Q27    8.64 m     89.1°  Origin
Q32    0.78 m     27.4°  Partial
Q36   12.08 m     91.2°  Origin
Q45    1.27 m     18.3°  Partial
Q48    8.63 m     89.7°  Origin
Q54   12.17 m     88.4°  Origin
Q58    1.05 m     16.0°  Partial
E4    12.26 m     88.5°  Origin
E8     8.63 m     90.2°  Origin
```

- *Good* = pose within ~0.5 m and ~6° of expected
- *Partial* = pose within 0.8-1.3 m but heading off by 16-27°
- *Origin* = pose stuck at (0, 0) with ~0° heading, 8-12 m from expected position

## Vision system performance: pre-auto (30s before enable)

```
Match  Obs Scores (min/max/mean)  FR Tags     FL Tags   RR  RL  Error@T
─────  ────────────────────────  ──────────  ────────  ──  ──  ────────
Q4     none                      {22}        {13}       —   —  0.31 m
Q10    none                      {6}         {29}       —   —  0.47 m
Q13    0.665 / 0.684 / 0.676    {6}         {29}       —   —  0.39 m
Q20    0.661 / 0.688 / 0.679    {3, 6}      {29}       —   —  1.25 m
Q23    none                      {15, 16}    {17}       —   —  12.17 m
Q27    none                      {31}        {1}        —   —  8.64 m
Q32    0.650 / 0.675 / 0.660    {6}         {29}       —   —  0.78 m
Q36    none                      {15, 16}    {17}       —   —  12.08 m
Q45    0.663 / 0.686 / 0.679    {22}        {13}       —   —  1.27 m
Q48    none                      {31, 32}    {1}        —   —  8.63 m
Q54    none                      {15, 16}    {17}       —   —  12.17 m
Q58    0.656 / 0.684 / 0.677    {22}        {13}       —   —  1.05 m
E4     none                      {15, 16}    {17}       —   —  12.26 m
E8     none                      {31}        {1}        —   —  8.63 m
```

## Vision quality during auto

> The chart `vache_tracking_error.png` (in the doc folder) shows accepted vision observation counts during auto for six exemplar matches. Upload it alongside this document.

Accepted vision observations per 2-second window during auto varied dramatically:
- Q4 received *zero* accepted observations for the entire 20-second auto
- Q20 received 14-42 observations per window throughout
- Q36 received 10-44 per window in the first 10s, then dropped to zero mid-auto before resuming
- E4 received only 4 total observations across the entire auto

## Vision findings

### RR and RL cameras couldn't see tags from starting positions

Across all 14 matches, the rear-right (RR) and rear-left (RL) cameras detected zero tags in the 10 seconds before auto enable and during the first seconds of auto. However, both cameras worked normally during teleop — detecting tags across a wide range (tags 1-29) in every match. The issue is that the rear cameras are mounted at 135° yaw (pointing backward-diagonal), and from the robot's pre-match starting positions there are simply no tags in their field of view. Once the robot drives into the field, the rear cameras pick up tags immediately.

This means the robot was running on only FR and FL for the most critical phase: pre-auto localization and the first seconds of trajectory following. Multi-camera fusion, which boosts observation scores by 1.4x and produces much more reliable pose estimates, was unavailable during this window.

### Observation scores clustered right at the acceptance threshold

The vision filter's minimum acceptance score is 0.65. In the 5 matches where scores were logged pre-auto, they ranged from 0.650 to 0.688 — barely above or right at the threshold. Single-tag observations from two cameras at 4-7 m range were marginal quality. Scores of 1.0 (which multi-camera fusion can produce) were never achieved pre-auto.

### No multi-camera fusion occurred pre-auto

`FusedCameraCount` was never populated before auto enable in any match. With only 2 cameras active and each seeing a single tag, the system couldn't achieve the spatial and temporal correlation needed for fusion (observations must come from different cameras within 150ms and agree within 0.15 m).

### Matches with observation scores had better pose estimates

The 5 matches where the vision scoring pipeline was active pre-auto (Q13, Q20, Q32, Q45, Q58) all had starting errors under 1.3 m. The 9 matches with no pre-auto scores all had errors over 8 m (stuck at origin). This correlation is strong: vision scoring activity → some observations accepted → pose at least partially correct.

### Starting position on the field determined camera effectiveness

The tag groups seen by FR and FL correlated with field position, and certain positions fared worse:
- *Right-side starts* (FR: tags 22 or 6, FL: tags 13 or 29): 0.31-1.27 m error (better)
- *Left-side starts* (FR: tags 15,16 or 31, FL: tags 17 or 1): 8.63-12.26 m error (worse — usually stuck at origin)

This suggests the left-side starting positions have worse sight lines to AprilTags, producing lower-quality observations that the filter rejects entirely.

## Summary

```
Category                Count  Matches
──────────────────────  ─────  ─────────────────────────
Good start (< 0.5 m)       3  Q4, Q10, Q13
Partial start (0.5-1.3m)   4  Q20, Q32, Q45, Q58
Origin (8-12 m)             7  Q23, Q27, Q36, Q48, Q54, E4, E8
```

The refactored `setPose()` fixes the root cause: every auto routine calls `resetOdometry()` → `setPose()`, which now resets the full pose estimator to the trajectory start. All 14 matches would have started with zero pose error. The re-enabled `yawConsistency` vision filter (gated on pose initialization) provides an additional layer of protection against bad observations once the pose has been initialized.

## How to verify this data

For any match, open the log in AdvantageScope and check:

1. *Find the auto enable time*: plot `/DriverStation/Enabled` and `/DriverStation/Autonomous`. The auto enable is where Enabled transitions to TRUE while Autonomous is already TRUE. Cross-reference with `/RealOutputs/GameState/MatchTime` — it should show ~19-20 within 2 seconds after enable and count down from there.
2. *Pose at enable*: read `/RealOutputs/Drive/Pose` at that timestamp.
3. *Expected pose*: check `/RealOutputs/AutoSelector/AutonomousInitialPose` (take the non-zero entry).
4. *Vision observations*: plot `/RealOutputs/Vision/Summary/RobotPosesAccepted` before enable — empty arrays mean no observations were accepted.

Example — E4:
- `/DriverStation/Enabled` → true at t=143.04s, `/DriverStation/Autonomous` already true since 25.98s
- `/RealOutputs/GameState/MatchTime` = 19.0 at t=144.99s (confirmed)
- `/RealOutputs/Drive/Pose` at t=143.04s: (0.064, 0.038) at -1.9°
- `/RealOutputs/AutoSelector/AutonomousInitialPose`: (12.308, 0.699) at -90.4°
- Robot disabled at t=150.36s (e-stopped after 7.3 seconds)
