# Vision Filter Tuning: Raising the Bar on Accepted Poses

**Date:** 2026-03-18
**Branch:** vision-tests
**Log analyzed:** akit_26-03-18_00-43-09.wpilog

## TL;DR

With `minScore = 0.02`, the vision filter was not actually filtering — the only
observations rejected were ones off the field or with zero tags. The scoring system
(sigmoids, weights, geometric mean) computed a quality score but the acceptance bar
was so low that the score only affected standard deviations, never accept/reject.
52 bad poses (off by up to 7m) were accepted in one session, causing 11 teleportation
events. Since 85% of our observations score 0.70+ and good corrections arrive multiple
times per second, we can raise the bar. We set `minScore = 0.6` and added a velocity
uncertainty penalty so that unverified first-in-a-while poses score below the new
threshold. Needs robot validation.

Changes: `velocityUncertainScore = 0.7` and `minScore = 0.6` in VisionConstants.java.

## Why raise the bar?

85% of observations score 0.70+. 21% score above 0.90 (multi-tag). Good corrections
come often enough that odometry never flies blind for long. A bad pose that corrupts
the estimator for 300-500ms does far more damage than a missed good pose that would
have been replaced moments later. We can afford to be selective.

## What was actually happening

With `minScore = 0.02` (= `linearStdDevBaseline / maxStdDev`), the accept/reject gate
was effectively just two hard binary checks:

- `withinBoundaries`: on the field? (returns 1.0 or 0.0)
- `moreThanZeroTags`: saw a tag? (returns 1.0 or 0.0)

Every other test (ambiguity, pitch, roll, height, distance, velocity) fed into the
weighted geometric mean and produced scores between 0.5 and 1.0 — all well above 0.02.
The scores scaled the standard deviations passed to the Kalman filter, but never
caused a rejection. The 155 rejected observations in the log were all boundary or
zero-tag cases.

From 2,829 observations in a 168-second session:

| | Count | Rate |
|---|---:|---:|
| Accepted (correct) | 1,754 | 89.4% |
| Accepted (wrong by 1.8-7.1m) | 52 | 2.7% |
| Rejected (boundary/no tags) | 59 | 3.0% |
| Rejected (likely good\*) | 96 | 4.9% |

\* Estimated: rejected poses with |z| < 0.15m and within field bounds.

Bad poses scored **0.60-0.66**. The threshold was **0.02**. They sailed through.

## Why a threshold alone doesn't work

Bad poses and legitimate marginal observations overlap in score:

```
  0.90-1.00 : #################### 590 (20.9%)  multi-tag
  0.75-0.90 : ########################################### 1,151 (40.7%)  good single-tag
  0.70-0.75 : ######################## 665 (23.5%)  decent single-tag
  0.60-0.70 : ############ 345 (12.2%)  BAD AND GOOD OVERLAP HERE
  < 0.60    : ## 78 (2.8%)
```

| Threshold | Rejected | Catches bad poses? | Loses good data? |
|-----------|---------|-------------------|-----------------|
| 0.02 (was) | 0 (0%) | No | No |
| 0.50 | 25 (0.9%) | No — bad poses score 0.60+ | Minimal |
| 0.60 | ~78 (2.8%) | Only with velocity penalty | Minimal |
| 0.70 | 423 (15%) | Yes, but... | **Kills 12% of single-tag data** |

We need to **shift the scores** so bad poses land below the threshold. That's the
velocity uncertainty penalty.

## Root cause

PhotonVision's PnP solver sometimes returns the wrong solution for single-tag
observations — a reflected/offset pose, often meters from reality. The
`velocityConsistency` test should catch these (implied velocity would be impossibly
high), but it had a loophole: when a camera hadn't reported in >0.5s, it returned
**1.0** (perfect pass) instead of expressing uncertainty. The bad pose got accepted,
then cascaded — it became the velocity reference, so the next bad pose from the same
camera appeared consistent.

```java
// BEFORE: no-history and timeout both gave a free pass
if (ctx.lastAcceptedPose() == null) return 1.0;
if (dt > velocityCheckTimeoutSeconds) return 1.0;

// AFTER: express uncertainty instead of confidence
if (ctx.lastAcceptedPose() == null) return velocityUncertainScore;  // 0.7
if (dt > velocityCheckTimeoutSeconds) return velocityUncertainScore; // 0.7
```

## Solution

Two changes that work together:

**1. `velocityUncertainScore = 0.7`** — When velocity can't be verified, return 0.7
instead of 1.0. Projected effect (calculated, not yet measured on robot):

| Observation type | Before (measured) | After (projected) |
|-----------------|--------|-------|
| Bad pose | 0.61 | ~0.58 |
| Good single-tag | 0.75 | ~0.71 |
| Multi-tag | 0.90 | ~0.86 |

**2. `minScore = 0.6`** — Raise the acceptance bar from 0.02 to 0.6.

| Observation type | Projected score | Expected result |
|-----------------|-----------|--------|
| Bad first-in-a-while pose | ~0.58 | **Rejected** (below 0.6) |
| Good single-tag, no history | ~0.71 | Accepted |
| Good single-tag, with history | ~0.75 | Accepted |
| Multi-tag | ~0.86-0.93 | Accepted |
| With correlation boost (1.3x) | ~0.92 | Accepted |

**Startup convergence** should be preserved: when the robot is placed on the field,
all cameras get the uncertainty penalty, but agreeing cameras get the 1.3x correlation
boost (~0.71 * 1.3 = ~0.92). Even without correlation, ~0.71 > 0.6. Only a lone camera
reporting a bad pose (~0.58, no correlation) gets rejected. Needs testing to confirm.

## Evidence (dig deeper)

<details>
<summary>The t=37s incident (click to expand)</summary>

| Time (s) | Fused Pose (x, y) | What happened |
|-----------|-------------------|---------------|
| 37.372 | (3.20, 7.23) | Last good pose. Camera1 quiet >0.5s |
| **37.406** | **(3.70, 2.39)** | **4.84m jump. Camera1 bad pose (score 0.610)** |
| 37.468 | (3.82, 2.07) | Bad (y=0.86) AND good (y=7.19) accepted simultaneously |
| **37.531** | **(3.70, 2.05)** | **Camera1 bad again (0.662) — consistent with its own bad pose** |
| 37.700 | (3.17, 6.76) | Mostly recovered, 294ms later |
| **37.954** | **(3.22, 6.85)** | **Velocity check finally catches Camera1 (score 0.115)** |

The cascade: bad pose accepted → becomes velocity reference → next bad pose appears
consistent → velocity check keeps passing until a good pose breaks the cycle.

</details>

<details>
<summary>The t=121s incident (worst offset — 7.1m)</summary>

Robot at y=7.6-7.9. Camera1 produced y=0.54 — opposite side of the field.
28 poses at y~7.6 were being rejected nearby (likely because the velocity check
was comparing them against the wrong reference). The filter was accepting poses
7m off while rejecting correct ones.

</details>

<details>
<summary>All 11 bad-pose incidents</summary>

| Time range | Bad poses accepted | Offset | Correct y | Bad y |
|------------|-------------------|--------|-----------|-------|
| 35.5-37.5s | 2 | ~6.4m | ~7.2 | ~0.9 |
| 102.5-104.5s | 10 | ~2.9m | ~6.0 | ~3.1 |
| 118.3-119.7s | 3 | ~3.1m | ~3.2 | ~6.3 |
| 121.1-123.1s | 3 | ~7.1m | ~7.6 | ~0.5 |
| 124.4-125.6s | 2 | ~5.6m | ~6.6 | ~1.0 |
| 136.7-138.7s | 7 | ~2.6m | ~1.1 | ~3.7 |
| 140.1-142.1s | 1 | ~6.3m | ~6.9 | ~0.6 |
| 146.8-148.8s | 10 | ~3.2m | ~4.1 | ~7.3 |
| 153.3-155.2s | 1 | ~6.0m | ~0.6 | ~6.6 |
| 156.3-158.2s | 12 | ~1.8m | ~2.3 | ~4.1 |
| 174.5-176.5s | 1 | ~6.2m | ~7.3 | ~1.2 |
| **Total** | **52** | | | |

All PnP ambiguity errors — reflected/offset in the y-axis.

</details>

<details>
<summary>All 23 fused-pose jumps >1m in <0.5s</summary>

| From (t, x, y) | To (t, x, y) | Distance | Implied velocity |
|-----------------|--------------|----------|-----------------|
| 37.065s (3.23, 7.33) | 37.468s (4.01, 0.86) | 6.51m | 16.2 m/s |
| 37.468s (4.01, 0.86) | 37.468s (3.19, 7.19) | 6.38m | simultaneous |
| 37.468s (3.19, 7.19) | 37.531s (3.87, 0.86) | 6.36m | 101.4 m/s |
| 37.531s (3.87, 0.86) | 37.700s (3.17, 7.02) | 6.20m | 36.5 m/s |
| 102.476s (3.72, 3.02) | 102.476s (3.29, 5.90) | 2.92m | simultaneous |
| 102.705s (3.28, 5.91) | 102.773s (3.76, 3.12) | 2.83m | 42.0 m/s |
| 102.773s (3.76, 3.12) | 102.773s (3.27, 5.91) | 2.83m | simultaneous |
| 102.980s (3.31, 5.96) | 103.043s (3.73, 3.11) | 2.89m | 45.8 m/s |
| 103.043s (3.73, 3.11) | 103.043s (3.31, 5.96) | 2.88m | simultaneous |
| 103.043s (3.31, 5.96) | 103.131s (3.79, 3.22) | 2.78m | 31.3 m/s |
| 103.592s (3.75, 3.12) | 103.684s (3.28, 5.90) | 2.82m | 30.7 m/s |
| 104.028s (3.30, 5.95) | 104.080s (3.79, 3.13) | 2.86m | 55.6 m/s |
| 104.080s (3.79, 3.13) | 104.080s (3.29, 5.96) | 2.87m | simultaneous |
| 122.409s (3.75, 7.57) | 122.489s (4.45, 0.54) | 7.07m | 87.7 m/s |
| 122.489s (4.45, 0.54) | 122.489s (3.62, 7.56) | 7.07m | simultaneous |
| 124.480s (5.86, 0.59) | 124.867s (5.73, 7.28) | 6.69m | 17.3 m/s |
| 141.684s (5.55, 7.57) | 142.114s (4.68, 0.64) | 6.98m | 16.2 m/s |
| 155.126s (4.22, 0.55) | 155.178s (4.74, 6.56) | 6.03m | 115.6 m/s |
| 155.178s (4.74, 6.56) | 155.568s (5.44, 0.70) | 5.90m | 15.1 m/s |
| 162.266s (3.98, 7.24) | 162.266s (4.91, 6.53) | 1.17m | simultaneous |
| 162.266s (4.91, 6.53) | 162.430s (3.97, 7.28) | 1.20m | 7.3 m/s |
| 176.411s (3.18, 7.31) | 176.485s (3.29, 1.15) | 6.16m | 83.8 m/s |
| 176.485s (3.29, 1.15) | 176.949s (3.19, 7.32) | 6.17m | 13.3 m/s |

Drivetrain speed limit is ~4.3 m/s. Every entry is a physically impossible movement.

</details>

## Files Changed

- `VisionConstants.java`: `velocityUncertainScore = 0.7`, `minScore = 0.6`
- `VisionFilter.java`: velocity consistency returns uncertainty score instead of 1.0
- `VisionFilterTest.java`: updated 3 tests for new behavior

All 78 unit tests pass. **Not yet validated on the robot** — projected score shifts
and startup convergence need a live test session.

## Tuning Guide

**Too aggressive** (rejecting good observations): decrease `minScore` or increase
`velocityUncertainScore`.

**Too permissive** (still accepting bad poses): increase `minScore`, decrease
`velocityUncertainScore`, or decrease `velocityCheckTimeoutSeconds`.

Values were chosen to sit in the calculated gap between projected bad-pose scores
(~0.58) and projected good single-tag scores (~0.71). After the first test with
these settings, check `Vision/Summary/ObservationScore` in the log to verify.
