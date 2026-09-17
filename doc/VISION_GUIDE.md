# The Vision System

The purpose of this document is to explain how the robot estimates its
position from AprilTags, how the observation filter scores and combines what
the cameras report, and how to calibrate, tune, and validate the system. It
assumes a reader who can build and deploy the robot code but has not worked
on the vision subsystem before. Parts 1 to 3 cover the filter's design, what
the match logs show about it, and how to tune it; Parts 4 to 6 cover camera
calibration, the implementation, and log replay, and build on the earlier
parts; Part 7 is reference material.

Note that calibration is upstream of the filter: a miscalibrated or misaligned
camera produces observations that are all wrong in the same direction, and no
filter setting corrects them. Part 4 nevertheless follows the tuning material,
since its checks read the pose estimate and the filter's logs, which the
earlier parts explain.

All numbers were checked against the code as of September 2026 (commit
f3ca52c). Where a constant is named, the value in the source file named with
it (usually `VisionConstants.java` or `VisionFilter.java`) is authoritative,
and this document should be corrected if the two disagree.

## Contents

- [Part 1. Scoring model](#part-1-scoring-model)
  - [1. Pose estimation and the role of vision](#1-pose-estimation-and-the-role-of-vision)
  - [2. The seven tests](#2-the-seven-tests)
  - [3. Combining test scores](#3-combining-test-scores)
  - [4. How the score is used](#4-how-the-score-is-used)
- [Part 2. Field data](#part-2-field-data)
  - [5. Case study: the March 2026 threshold change](#5-case-study-the-march-2026-threshold-change)
  - [6. Geometric versus arithmetic mean on match data](#6-geometric-versus-arithmetic-mean-on-match-data)
- [Part 3. Tuning](#part-3-tuning)
  - [7. Tuning parameters and procedure](#7-tuning-parameters-and-procedure)
  - [8. Exercises](#8-exercises)
- [Part 4. Calibration and validation](#part-4-calibration-and-validation)
  - [9. Sources of error upstream of the filter](#9-sources-of-error-upstream-of-the-filter)
  - [10. Camera calibration (intrinsics)](#10-camera-calibration-intrinsics)
  - [11. Camera mounting (extrinsics)](#11-camera-mounting-extrinsics)
  - [12. AprilTag field layout](#12-apriltag-field-layout)
  - [13. Validation procedures](#13-validation-procedures)
- [Part 5. Implementation](#part-5-implementation)
  - [14. Source files and data flow](#14-source-files-and-data-flow)
  - [15. Multi-camera fusion](#15-multi-camera-fusion)
  - [16. Unit tests](#16-unit-tests)
  - [17. Disabled tests](#17-disabled-tests)
- [Part 6. Log replay](#part-6-log-replay)
  - [18. Overview](#18-overview)
  - [19. Procedure](#19-procedure)
  - [20. Interpreting the comparison](#20-interpreting-the-comparison)
  - [21. Limitations](#21-limitations)
- [Part 7. Reference](#part-7-reference)
  - [22. Common questions](#22-common-questions)
  - [23. Open work](#23-open-work)
  - [24. Glossary](#24-glossary)
  - [Related documents](#related-documents)

---

# Part 1. Scoring model

## 1. Pose estimation and the role of vision

The robot maintains an estimate of its pose: x, y, and heading on the field.
The estimate comes from WPILib's swerve drive pose estimator, which combines
three sources.

- Wheel odometry. Each swerve module reports the distance it has rolled. The
  result is accurate over short intervals and drifts over a match.
- The gyro. It supplies the heading and is reliable.
- Vision. Four cameras running PhotonVision detect AprilTags. From a tag's
  corners in the image, PhotonVision solves for the camera's pose relative to
  the tag. The robot code combines that with the tag's known location on the
  field and the camera's mounting transform to get the robot's pose. (With
  several tags in view, PhotonVision solves for the camera's field pose
  directly and only the mounting transform is applied; section 14 has both
  paths.)

In short, odometry drifts and vision corrects it. Throughout this document an
observation is one camera reading, and a measurement is what the pose
estimator receives, i.e., an accepted observation or a merged group of them
(section 4). The estimator applies whatever vision gives it, weighted by a
standard deviation supplied with each measurement (an estimate of how far off
the measurement might be; see section 4), and it has no independent way to
distinguish a correct measurement from a wrong one.

### Frames and units

Field coordinates follow WPILib's convention: the origin is the corner of the
field at the blue alliance wall, on the right as seen from the blue driver
station, x runs the length of the field toward the red alliance wall, and y
runs across it, positive to the left. The field is about 16.5 m by 8.0 m
(`Field.java`). A heading of 0 faces the red alliance wall, and headings
increase counterclockwise. Every pose in the logs, including each camera's
observations, is in this frame, so a wrong single-tag solution "on the
opposite side of the field" (section 5) is one whose y is near the other long
wall while its x is about right.

Robot coordinates have their origin at the robot's center at floor level, x
forward, y left, z up. The camera mounts in section 11 are given in this
frame.

Constants in `VisionConstants.java` are written in whatever unit is natural
to read, using WPILib's unit types: `Degrees.of(5)` for a tilt tolerance,
`Inches.of(36.875)` for the robot width, `Meters.of(4.0)` for a tag distance.
Each is converted once to the units the code computes in, which are always
meters, radians, and seconds, and the converted values have names ending in
`_METERS` or `_RADIANS`. Timestamps are seconds on the robot's clock.

### Why vision observations are sometimes wrong

A single AprilTag viewed from one angle projects to nearly the same image as
its mirror image viewed from another, so the solver, PnP
(Perspective-n-Point), sometimes returns the wrong one of the two solutions.
The resulting pose is typically meters from the true position, reflected
across the field. PhotonVision reports an ambiguity value from 0 to 1
describing how close the two candidate solutions were. Distant tags, motion
blur, and tags near the edge of the image degrade the solution as well.

The failure is common: in a practice session on March 18, 2026, before the
filter had a meaningful acceptance threshold, the robot accepted 52
observations that were 1.8 to 7.1 m from its true position, each of which
pulled the estimate across the field and back within about half a second.
Since auto paths and turret aiming both depend on the pose, each such
excursion costs a shot or a path segment. Section 5 covers that session in
detail.

### The filter

Every observation passes through `VisionFilter` before it can reach the pose
estimator. The filter assigns it a score from 0 to 1, and observations scoring
above `MIN_SCORE` (0.65) are accepted while the rest are discarded.

```
camera --> observation --> 7 tests --> combine --> score > 0.65? --> fuse cameras --> pose estimator
                          (7 scores)   (1 score)   accept / reject   that agree
```

An observation (`PoseObservation` in `VisionIO.java`) carries the capture
timestamp, the estimated pose, and three quality indicators from PhotonVision.
A sixth field, `type`, is always `PHOTONVISION` on the robot and is not read
by the filter.

| Field | Meaning |
|---|---|
| `timestamp` | Capture time of the frame, in seconds. Used to match observations from different cameras (section 4) and passed to the pose estimator so that the measurement is applied at the time it was taken (section 14) |
| `pose` | Robot pose in 3D: x, y, z in meters and roll, pitch, yaw in radians |
| `ambiguity` | 0 for one clear solution, 1 when two solutions fit equally well |
| `tagCount` | Number of AprilTags used in the solution |
| `averageTagDistance` | Mean distance from the camera to those tags, in meters |

The filter scores each observation as it arrives; every fifth 20 ms loop
(`PROCESSING_INTERVAL_LOOPS`) it takes the observations that were accepted,
merges those from cameras that agree, and sends the results to the pose
estimator.

## 2. The seven tests

The filter runs seven tests on each observation. Each is an entry in the
`Test` enum in `VisionFilter.java`, returns a score from 0 (certainly wrong)
to 1 (no evidence of error), and carries a weight whose role is described in
section 3.

| Test | Weight | Checks | Scoring |
|---|---|---|---|
| `moreThanZeroTags` | 1.0 | At least one tag was used | 1 or 0 |
| `withinBoundaries` | 1.0 | Position is inside the field | 1 or 0 |
| `unambiguous` | 0.8 | Single-tag only: solution was not ambiguous | Curve on ambiguity, midpoint 0.15, steepness 4 |
| `pitchError` | 0.7 | Robot is not pitched | Curve on pitch magnitude, midpoint 5 degrees, steepness 1 |
| `rollError` | 0.7 | Robot is not rolled | Curve on roll magnitude, midpoint 5 degrees, steepness 1 |
| `heightError` | 0.7 | Robot is at floor level | Curve on height magnitude, midpoint 0.25 m, steepness 1 |
| `distanceToTags` | 0.5 | Tags were close | Curve on distance, midpoint 4 m, steepness 1 |

The first two are binary: an observation with no tags, or with a position
outside the field boundary, scores 0. (The boundary is inset on every side by
half of `MIN_ROBOT_WIDTH`, 36.875 in, i.e., by 0.47 m.) The other five are
continuous.

Because the robot drives on a flat floor, a solution that has it pitched 20
degrees or 30 cm above the carpet is wrong; the pitch, roll, and height tests
measure that departure. `unambiguous` applies only to single-tag
observations; with two or more tags there is no mirror-image ambiguity, and it
returns 1.

### The scoring curve

The five continuous tests use the same function, a logistic sigmoid with a
midpoint (the tolerance constant in `VisionConstants.java`) and a steepness.
The steepness is per unit of the tested quantity: per radian for pitch and
roll, per meter for height and distance, and per unit of ambiguity. The score
is 0.5 at the midpoint, rises toward 1 below it, and falls toward 0 above it.

```java
// VisionFilter.java
public static double normalizedSigmoid(double x, double midpoint, double steepness) {
  double exponent = -steepness * (x - midpoint);
  return 1.0 / (1.0 + Math.exp(exponent));
}
// each continuous test returns:  1.0 - normalizedSigmoid(x, tolerance, steepness)
// where x is the tested quantity: ambiguity, |pitch|, |roll|, |z|, or average tag distance
```

In other words, the midpoint is where the score is 0.5, and the steepness sets
how quickly the score changes around it: with steepness k, the score falls
from 0.73 to 0.27 as the tested quantity rises from 1/k below the midpoint to
1/k above it. For pitch, k is 1 per radian, so that fall is spread over 2
radians (115 degrees), which is why the curve is nearly flat over any tilt
the robot can have. For ambiguity, k is 4, so the same fall would span 0.5
units of ambiguity, from below zero to 0.40; the score is already 0.646 at
ambiguity 0 and reaches 0.27 at 0.40. For distance, k is 1 per meter, so the
fall takes place between 3 m and 5 m.

`pitchError` at several tilts, with the 5-degree tolerance and steepness 1.0
per radian:

| Pitch | Score |
|---|---|
| 0 degrees | 0.522 |
| 5 degrees | 0.500 |
| 20 degrees | 0.435 |
| 90 degrees | 0.185 |

Two properties of these constants recur in sections 3, 6, and 7; both follow
from the current values rather than from any design intent. A robot with zero
pitch scores only 0.522, because the midpoint is close to zero and the curve
is shallow, and the same shallowness means that 90 degrees of pitch still
scores 0.185. Roll and height behave likewise.

`unambiguous`, with steepness 4 per unit of ambiguity, discriminates well by
comparison:

| Ambiguity | Score |
|---|---|
| 0 | 0.646 |
| 0.01 | 0.636 |
| 0.05 | 0.599 |
| 0.15 | 0.500 |
| 0.30 | 0.354 |
| 0.40 | 0.269 |
| 1.00 | 0.032 |

`distanceToTags`, with steepness 1 per meter and midpoint 4 m, scores 0.881
at 2 m, 0.731 at 3 m, 0.269 at 5 m, 0.119 at 6 m, and 0.018 at 8 m. Sections
6 and 20 rely on these values.

### Two disabled tests

`VisionFilter.java` also defines `velocityConsistency` and `yawConsistency`,
but neither is in `DEFAULT_ENABLED_TESTS`, so neither runs; their weights and
the constant `VELOCITY_UNCERTAIN_SCORE` have no effect until they are enabled.
Section 17 describes both.

## 3. Combining test scores

Seven scores go in and one accept-or-reject decision comes out, and the rule
for combining them determines whether six passing tests can outvote one
failing test. Four candidate rules are compared below on exactly that case.

### Arithmetic mean

The first candidate is a weighted average. Consider an observation placing the
robot at (-5, -5), 5 m outside the field, with every other test scoring 1.

```
tags 1.0   bounds 0.0   unambiguous 1.0   pitch 1.0   roll 1.0   height 1.0   distance 1.0
```

The unweighted mean is 6/7 = 0.857 and the weighted mean is 4.4/5.4 = 0.815.
Both exceed 0.65, so the observation is accepted and the estimate leaves the
field.

The scores in this example are idealized. With the real curves, a level robot
scores 0.522 on pitch and roll and 0.562 on height, and with the remaining
tests still at 1.0 the weighted mean is 0.634. With the ambiguity and tag
distance of the worked example below (0.636 and 0.731), it is 0.555, the
figure in the comparison table. Either way this particular observation is
rejected. The defect is structural regardless: an average lets six passing
tests outvote one failing test.

### Reject on any zero

Adding a rule that any zero score rejects the observation fixes the off-field
case but not the case of a level single-tag observation with a tag 3 m away
and ambiguity 0.4, where `unambiguous` scores 0.269 and nothing scores zero.
With the other scores as in the worked example below, the weighted mean is
0.686 and the observation is accepted, although its ambiguity of 0.4 is well
past the 0.15 tolerance.

### Product

Multiplying the seven scores gives zero whenever any test scores zero, and a
score of 0.269 reduces the product substantially without being zero, which is
the desired behavior.

The product, however, does not scale with the number of tests. Seven tests
each scoring 0.7 multiply to 0.7^7 = 0.082, which reads as a near-certain
failure. Every additional test shrinks the product further (seven scores of
0.9 give 0.478, eight give 0.430), so the result depends on how many tests run
and not only on how the observation performed.

### Geometric mean

Taking the seventh root of the product restores the scale: seven scores of
0.7 give 0.7, seven of 0.9 give 0.9, and an eighth test means an eighth root,
so the scale is unchanged.

The result is the geometric mean, which is what the filter computes.

```
arithmetic mean = (s1 + s2 + ... + s7) / 7
geometric mean  = (s1 * s2 * ... * s7) ^ (1/7)
```

### Weights as exponents

In a weighted arithmetic mean, each score is multiplied by its weight before
summing. In a weighted geometric mean, each score is raised to the power of its
weight before multiplying, and the root is taken with the total weight, which
is 5.4 for the seven enabled tests.

```
score = (s1^w1 * s2^w2 * ... * s7^w7) ^ (1 / (w1 + w2 + ... + w7))
```

From `scoreObservation` in `VisionFilter.java`:

```java
double weightedProduct = 1.0;
double sumOfWeights = 0.0;
for (var entry : testResults.entrySet()) {
  double score = entry.getValue();
  double weight = entry.getKey().weight();
  weightedProduct *= Math.pow(score, weight);
  sumOfWeights += weight;
}
double totalScore = Math.pow(weightedProduct, 1.0 / sumOfWeights);
```

A weight of 2 counts a score twice, whereas a weight of 0.5 counts its square
root, which is closer to 1 and so pulls the product less. A test's effective
share of the result is its weight divided by the total.

| Test | Weight | Share of total |
|---|---|---|
| `moreThanZeroTags` | 1.0 | 18.5% |
| `withinBoundaries` | 1.0 | 18.5% |
| `unambiguous` | 0.8 | 14.8% |
| `pitchError` | 0.7 | 13.0% |
| `rollError` | 0.7 | 13.0% |
| `heightError` | 0.7 | 13.0% |
| `distanceToTags` | 0.5 | 9.3% |

Changing any one weight changes the total and therefore every other test's
share.

### Worked example

A typical single-tag observation: robot at (8, 4), level, ambiguity 0.01, tag
3 m away.

| Test | Score | Weight | Score ^ weight |
|---|---|---|---|
| `moreThanZeroTags` | 1.000 | 1.0 | 1.000 |
| `withinBoundaries` | 1.000 | 1.0 | 1.000 |
| `unambiguous` | 0.636 | 0.8 | 0.696 |
| `pitchError` | 0.522 | 0.7 | 0.634 |
| `rollError` | 0.522 | 0.7 | 0.634 |
| `heightError` | 0.562 | 0.7 | 0.668 |
| `distanceToTags` | 0.731 | 0.5 | 0.855 |

The product of the right-hand column is 1 × 1 × 0.696 × 0.634 × 0.634 × 0.668
× 0.855 = 0.160. The total weight is 5.4, so the score is 0.160 ^ (1/5.4) =
**0.712**, above 0.65, and the observation is accepted. The weighted arithmetic
mean of the same scores is 0.741.

### Comparison

The following table scores five observations both ways against the 0.65
threshold.

| Observation | Arithmetic | Geometric |
|---|---|---|
| Off the field at (-5, -5) | 0.555 | **0.000** |
| Single tag, ambiguity 0.4, 3 m | 0.686 accept | **0.627 reject** |
| Typical single tag, 3 m | 0.741 | 0.712 |
| Same, pitched 20 degrees | 0.729 | 0.696 |
| Three tags at 2 m | 0.808 | 0.775 |

The typical single-tag and three-tag rows are close to the highest scores the
current constants allow. Because pitch, roll, and height score only 0.52 to
0.56 at zero error, a three-tag observation cannot score above 0.783 and a
single-tag observation cannot score above 0.734 (both limits at zero ambiguity
and a tag at zero distance). Section 7 returns to these ceilings.

The geometric mean of a set of numbers is never greater than the arithmetic
mean, with equality only when all the numbers are equal (the AM-GM
inequality), and the gap widens as the numbers spread out. A wrong
observation is usually lopsided, with most tests passing and one failing
badly, which is precisely the case the geometric mean penalizes most and the
arithmetic mean least. For instance, the pairs (0.5, 0.5) and (1.0, 0.25)
have the same geometric mean, 0.5, but arithmetic means of 0.5 and 0.625; an
average favors the lopsided pair.

### The logarithmic view

Taking the natural log of each score turns the product into a sum: the log of
the weighted geometric mean is the weighted arithmetic mean of the log scores.
ln(1) = 0, ln(0.5) = -0.69, ln(0.1) = -2.3, and ln(0) is negative infinity. In
this form the geometric mean is an ordinary average, but of a quantity that is
at most zero. No test can contribute positive evidence, so no number of passing
tests can cancel one strongly failing test. The threshold of 0.65 corresponds
to a mean log score of -0.43.

### Why this shape fits the problem

- Some tests are disqualifying: an observation off the field or with no tags
  is wrong regardless of the rest. In a product, a zero is a veto.
- Errors compound: a distant tag combined with an ambiguous solve is worse
  than the sum of the two effects.
- The score's second use, described in section 4, is as a divisor, and ratios
  compose by multiplication.

## 4. How the score is used

After the tests run, the score serves three purposes in turn: it gates the
observation, it is adjusted when cameras agree, and it sets the measurement's
weight.

**Acceptance.** In `Vision.java`, an observation is kept only if its score is
at least `MIN_SCORE`: the batch filter removes observations with
`score < MIN_SCORE`. The accepted-or-rejected logging uses a strict `>`, so a
score exactly equal to the threshold is logged as rejected but still fused
and sent to the estimator. (A NaN score fails both comparisons; see section 7,
rule 2.)

**Agreement between cameras (fusion).** Every fifth loop,
`fuseCorrelatedObservations` examines the accepted observations. Observations
from different cameras are linked when their positions lie within 0.15 m and
their timestamps within 0.15 s of each other; links chain, so a cluster can
contain observations that do not agree directly. A cluster containing two or
more cameras is merged into one averaged pose. The merged observation takes
the highest score in the group multiplied by 1.4, capped at 1.0, and is exempt
from the single-camera multiplier described next. Because of this boost,
values logged as `Vision/Summary/ObservationScore` can exceed 0.783, the
ceiling for any single observation under the current curves (section 3).
Section 15 gives the procedure.

**Measurement weight.** The pose estimator takes each pose together with a
standard deviation, i.e., an estimate of how far off the measurement could be;
a small value moves the estimate most of the way to the measurement, whereas
a large value moves it only a little. `Vision.java` derives the standard
deviation by division (an angular standard deviation is derived the same way
from `ANGULAR_STD_DEV_BASELINE`, 0.06 rad):

```java
// Vision.java
double cameraCountFactor = (fused.cameraCount() == 1) ? SINGLE_CAMERA_STD_DEV_MULTIPLIER : 1.0;
double linearStdDev = LINEAR_STD_DEV_BASELINE * cameraCountFactor / fused.score();
```

With a baseline of 0.02 m and a single-camera multiplier of 3.0:

| Score | Standard deviation sent (single camera) |
|---|---|
| 0.775 (three tags) | 0.077 m |
| 0.712 (typical) | 0.084 m |
| 0.65 (threshold) | 0.092 m |

The estimator uses that value as follows. It carries a standard deviation of
its own for the odometry-based estimate, 0.1 m in x and y and 0.1 rad in
heading (the WPILib defaults, which `Drive.java` does not override). On each
vision measurement it moves the estimate toward the measured pose by the
fraction

```
k = odometry std dev / (odometry std dev + vision std dev)
```

on each axis, at the time the frame was captured, and then re-applies the
odometry recorded since. With the values above, a single-camera observation
at the typical score of 0.712 (0.084 m) moves the estimate 54% of the way to
the measurement; one at the threshold (0.092 m) moves it 52%; a fused
observation at 0.775, which is exempt from the 3x multiplier (0.026 m), moves
it 79%. The heading axis works the same way with the angular values and
trusts vision less: 28% for that typical single-camera observation.

Note that halving the score doubles the standard deviation and, at these
values, cuts the fraction from 54% to 37%; a change in weights that shifts
every score therefore changes not only which observations are accepted but
also how strongly each accepted one corrects the estimate.

---

# Part 2. Field data

## 5. Case study: the March 2026 threshold change

The acceptance threshold and the reasoning behind it come from a practice
session logged on March 18, 2026 (`akit_26-03-18_00-43-09.wpilog`). The
session is documented here at some length because it exhibits the common
tuning errors in one place.

### The original threshold

`MIN_SCORE` was originally derived rather than chosen: `LINEAR_STD_DEV_BASELINE`
(0.02 m) divided by `MAX_STD_DEV` (1.0 m), the largest standard deviation to
be sent, 0.02 / 1.0 = 0.02. The intent was for the score to scale trust
(section 4) and for the threshold to reject only outright failures.

The five continuous tests never reach zero, and under the geometric mean none
of them can bring a total below 0.02 on its own: a tag 10 m away scores
0.0025 on `distanceToTags`, yet a typical single-tag observation with that
score still totals about 0.4. Only `withinBoundaries` and
`moreThanZeroTags` produce zeros, and `velocityConsistency`, which was enabled
in the build running that day and is disabled in the current code (section
2), could push a total below 0.02 only for implied speeds above about 19 m/s.
With a threshold of 0.02, the filter reduced to those checks. Every other
observation was accepted, with its score affecting only its standard
deviation.

In 168 seconds of driving, the log holds 1,961 observations (inputs are
logged every other loop, so the robot processed about twice that), every one
of them single-tag. Of these, 155 were rejected: 121 for lying outside the
field boundary, all within half a robot width of the two side walls, and 34
by `velocityConsistency`, which scores near zero at the speeds these
observations implied against the camera's previous accepted pose. Of those
34, about fifteen were wrong poses the test caught and about twelve were
correct poses compared with a wrong reference (see below); the rest cannot be
classified. Of the accepted observations, 52 (identified by the method at
the end of this section) were 1.8 to 7.1 m from the robot's true position.
They arrived in 11 clusters and produced 23 jumps of more than a meter within
half a second between consecutive accepted vision poses. Nearly all had
approximately the correct x and a y on the opposite side of the field, the
signature of a mirror-image PnP solution.

### Cost asymmetry

An accepted wrong observation pulls the estimate toward itself, and 300 to 500
ms of correct observations are needed to pull it back. During that interval
the turret aims at the wrong point and any running path is steering from the
wrong position.

A rejected correct observation costs almost nothing, because another correct
observation arrives within tens of milliseconds. With the code of that date,
85% of observations scored above 0.70 and about a fifth above 0.90 (every
observation in that log was single-tag; the scores above 0.90 came from the
correlation boost, which that build applied before the threshold). Because
correct observations are abundant and wrong ones expensive, the filter can
afford to be selective; this asymmetry justifies a high threshold.

### Why raising the threshold alone was insufficient

The wrong observations scored 0.60 to 0.66 under the code of that date. Many
correct single-tag observations scored in the same range.

| Threshold | Rejected | Catches the wrong observations? | Loses correct ones? |
|---|---|---|---|
| 0.02 (original) | none | No | No |
| 0.50 | under 1% | No; the wrong observations score 0.60 and above | Few |
| 0.60 | about 3% | Only with the velocity change below | Few |
| 0.70 | about 15% | Yes | Yes, about 12% of single-tag observations |

The score scale was different then, for two reasons. First, although the
curves were the same, `velocityConsistency` was in the enabled set with
weight 0.9 and scored 1.0 for nearly every observation, which raises the
total weight from 5.4 to 6.3 and lifts every score (section 7, rule 3): the
typical single-tag observation of section 3 scores 0.747 under that set
against 0.712 now, and the single-tag ceiling of 0.734 (section 3) becomes
about 0.77. Second, the correlation boost, then 1.3x, was applied to
individual scores before the threshold and before logging, so about a fifth
of the logged scores lie above even that ceiling. These figures are therefore
not comparable with current scores.

In general, when the score distributions of correct and wrong observations
overlap, no threshold separates them; the tests must change so that the
distributions separate before a threshold can be placed in the gap.

### The velocity test exception

The test intended to catch these observations was `velocityConsistency`,
which compares each observation with the last accepted observation from the
same camera and computes the implied speed. The drivetrain's top speed is
about 4 m/s, whereas the wrong observations implied speeds from 7 to 115 m/s.

The test had an exception: when a camera had no accepted observation in the
last half second, or no history at all, it returned 1.0, a full pass. The
sequence at 37 seconds into the session shows the consequence. (Camera 1 is
the front left camera; section 11 lists the indices.)

| Time (s) | Fused pose (x, y) | Event |
|---|---|---|
| 37.372 | (3.20, 7.23) | Last correct pose. Camera 1 has had no accepted observation for over half a second |
| **37.406** | **(3.70, 2.39)** | Camera 1 reports a mirror-image pose, score 0.61. With no recent history the velocity test passes it. The estimate jumps 4.8 m |
| 37.468 | (3.82, 2.07) | A wrong observation (y = 0.86) and a correct one (y = 7.19) are accepted in the same cycle |
| **37.531** | **(3.70, 2.05)** | Camera 1 reports another wrong pose, score 0.66. It is consistent with camera 1's previous wrong pose, so the velocity test passes it again |
| 37.700 | (3.17, 6.76) | Mostly recovered, 294 ms later |
| 37.954 | (3.22, 6.85) | Camera 1 reports a correct pose (y = 6.90). Measured against its wrong reference, the velocity test cuts the score to 0.115, still above the 0.02 threshold, so it is accepted and becomes camera 1's reference |

In short, one wrong observation was accepted, became the velocity reference
for that camera, and made the next wrong observation from the same camera
appear consistent; the test kept passing until a correct observation replaced
the reference. The pattern repeated at 102, 121, 137, 147, and 156
seconds, with up to 12 wrong observations per cluster.

The fix was to return an uncertainty value instead of a pass when the test
could not be evaluated:

```java
// before: no history and stale history both passed
if (ctx.lastAcceptedPose() == null) return 1.0;
if (dt > VELOCITY_CHECK_TIMEOUT_SECONDS) return 1.0;

// after: return an uncertainty value and let the other tests decide
if (ctx.lastAcceptedPose() == null) return VELOCITY_UNCERTAIN_SCORE;
if (dt > VELOCITY_CHECK_TIMEOUT_SECONDS) return VELOCITY_UNCERTAIN_SCORE;
```

With `VELOCITY_UNCERTAIN_SCORE` below 1.0, an observation without a valid
velocity reference has to be accepted on the strength of the other tests. In
the build the analysis was written against, the correlation boost was applied
before the threshold, so at startup, when every camera carries the penalty,
agreeing cameras could still be lifted over it (0.71 x 1.3 = 0.92) and the
estimate converged. The change that merged on March 19 moved the boost after
the threshold, where it remains (section 4): an observation now passes or
fails on its own score, and agreeing cameras earn a merged pose and a tighter
standard deviation only once accepted. With the current constants, the
typical single-tag observation of section 3 would score 0.695 with the
uncertainty value in the set, still above 0.65. The comment on
`VELOCITY_UNCERTAIN_SCORE` in `VisionConstants.java` still describes the
earlier order.

### What was deployed

The analysis proposed a threshold of 0.60 and an uncertainty value of 0.7,
placed between the projected scores of wrong observations (about 0.58) and of
correct single-tag observations (about 0.71). The change merged on March 19
(#171, commit 9f39974) with `MIN_SCORE = 0.65`, `VELOCITY_UNCERTAIN_SCORE =
0.6`, and `velocityConsistency` commented out of `DEFAULT_ENABLED_TESTS`. The
test has not been enabled since. In the current code the raised threshold
works alone, and the uncertainty constant is unused pending the test being
enabled and the defect described in section 17 being fixed.

The analysis write-up (the former `doc/VISION_FILTER_TUNING.md`, folded into
this section), the comments in `VisionConstants.java`, and the deployed
constants each record a different moment and disagree in detail: the write-up
proposed 0.60 and 0.7, the comments still describe a threshold of 0.6 and a
1.3x boost, and the constants are 0.65, 0.6, and 1.4. The code is
authoritative over the comments, and the log of what the robot did is
authoritative over the code.

### Identifying wrong observations in a log

Ground truth is rarely available, so the practical approach is to look for
physically impossible behavior.

- Jumps. In AdvantageScope, plot `Vision/Summary/RobotPosesAccepted`.
  Consecutive accepted poses more than a meter apart within half a second
  imply a speed the drivetrain cannot reach. Divide distance by time and
  compare with `DRIVETRAIN_SPEED_LIMIT` in `DriveConstants.java`, 4.1 m/s in
  the current code.
- Reflections. A wrong single-tag solution usually has approximately the
  correct x and a y reflected to the far side of the field.
- Disagreement. Two cameras reporting poses meters apart in the same cycle
  means at least one is wrong.
- Score distribution. After any tuning change, plot
  `Vision/Summary/ObservationScore`. It holds only the scores sent to the
  estimator, after the threshold and the fusion boost (section 4), so it shows
  the accepted side of the threshold only; the rejected side is visible only
  in a replay with the threshold lowered (Part 6).

The same signature appeared at the earlier VAALE event: a March 13 analysis
of the E9 log, recorded by a build that had no velocity check, found 140
pairs of consecutive accepted poses implying more than 7.5 m/s, the fastest
171 m/s.

<details>
<summary>Score distribution of the session (scale of that date)</summary>

| Score | Records | Share |
|---|---|---|
| 0.90 to 1.00 | 590 | 21% |
| 0.75 to 0.90 | 1,151 | 41% |
| 0.70 to 0.75 | 665 | 24% |
| 0.60 to 0.70 | 345 | 12% |
| below 0.60 | 78 | 3% |

The wrong observations sat in the 0.60 to 0.70 band together with correct
single-tag observations. The counts are of logged `ObservationScore` records
(2,829 in all, the figure the March analysis called the number of
observations), and the scale is that of the March 18 build, described above.

</details>

<details>
<summary>All 11 clusters of wrong observations from the session</summary>

| Time range (s) | Wrong observations accepted | Offset (m) | Correct y (m) | Wrong y (m) |
|---|---|---|---|---|
| 35.5 to 37.5 | 2 | ~6.4 m | ~7.2 | ~0.9 |
| 102.5 to 104.5 | 10 | ~2.9 m | ~6.0 | ~3.1 |
| 118.3 to 119.7 | 3 | ~3.1 m | ~3.2 | ~6.3 |
| 121.1 to 123.1 | 3 | ~7.1 m | ~7.6 | ~0.5 |
| 124.4 to 125.6 | 2 | ~5.6 m | ~6.6 | ~1.0 |
| 136.7 to 138.7 | 7 | ~2.6 m | ~1.1 | ~3.7 |
| 140.1 to 142.1 | 1 | ~6.3 m | ~6.9 | ~0.6 |
| 146.8 to 148.8 | 10 | ~3.2 m | ~4.1 | ~7.3 |
| 153.3 to 155.2 | 1 | ~6.0 m | ~0.6 | ~6.6 |
| 156.3 to 158.2 | 12 | ~1.8 m | ~2.3 | ~4.1 |
| 174.5 to 176.5 | 1 | ~6.2 m | ~7.3 | ~1.2 |
| **Total** | **52** | | | |

</details>

<details>
<summary>All 23 jumps of more than a meter within half a second</summary>

| From (t, x, y) | To (t, x, y) | Distance | Implied speed |
|---|---|---|---|
| 37.065 (3.23, 7.33) | 37.468 (4.01, 0.86) | 6.51 m | 16.2 m/s |
| 37.468 (4.01, 0.86) | 37.468 (3.19, 7.19) | 6.38 m | same cycle |
| 37.468 (3.19, 7.19) | 37.531 (3.87, 0.86) | 6.36 m | 101.4 m/s |
| 37.531 (3.87, 0.86) | 37.700 (3.17, 7.02) | 6.20 m | 36.5 m/s |
| 102.476 (3.72, 3.02) | 102.476 (3.29, 5.90) | 2.92 m | same cycle |
| 102.705 (3.28, 5.91) | 102.773 (3.76, 3.12) | 2.83 m | 42.0 m/s |
| 102.773 (3.76, 3.12) | 102.773 (3.27, 5.91) | 2.83 m | same cycle |
| 102.980 (3.31, 5.96) | 103.043 (3.73, 3.11) | 2.89 m | 45.8 m/s |
| 103.043 (3.73, 3.11) | 103.043 (3.31, 5.96) | 2.88 m | same cycle |
| 103.043 (3.31, 5.96) | 103.131 (3.79, 3.22) | 2.78 m | 31.3 m/s |
| 103.592 (3.75, 3.12) | 103.684 (3.28, 5.90) | 2.82 m | 30.7 m/s |
| 104.028 (3.30, 5.95) | 104.080 (3.79, 3.13) | 2.86 m | 55.6 m/s |
| 104.080 (3.79, 3.13) | 104.080 (3.29, 5.96) | 2.87 m | same cycle |
| 122.409 (3.75, 7.57) | 122.489 (4.45, 0.54) | 7.07 m | 87.7 m/s |
| 122.489 (4.45, 0.54) | 122.489 (3.62, 7.56) | 7.07 m | same cycle |
| 124.480 (5.86, 0.59) | 124.867 (5.73, 7.28) | 6.69 m | 17.3 m/s |
| 141.684 (5.55, 7.57) | 142.114 (4.68, 0.64) | 6.98 m | 16.2 m/s |
| 155.126 (4.22, 0.55) | 155.178 (4.74, 6.56) | 6.03 m | 115.6 m/s |
| 155.178 (4.74, 6.56) | 155.568 (5.44, 0.70) | 5.90 m | 15.1 m/s |
| 162.266 (3.98, 7.24) | 162.266 (4.91, 6.53) | 1.17 m | same cycle |
| 162.266 (4.91, 6.53) | 162.430 (3.97, 7.28) | 1.20 m | 7.3 m/s |
| 176.411 (3.18, 7.31) | 176.485 (3.29, 1.15) | 6.16 m | 83.8 m/s |
| 176.485 (3.29, 1.15) | 176.949 (3.19, 7.32) | 6.17 m | 13.3 m/s |

Every row exceeds the drivetrain's top speed of about 4 m/s.

</details>

## 6. Geometric versus arithmetic mean on match data

Section 3 argues for the geometric mean from first principles. Whether the
choice matters in practice was measured in September 2026 by re-scoring every
observation from two matches at the VACHE event of March 2026, E4 and E8
(14,303 observations), with both formulas under the current constants. The
re-scoring was done offline with a Python re-implementation of
`scoreObservation` checked against the Java filter; neither the script nor the
match logs are in the repository.

At the same threshold of 0.65, the arithmetic mean would accept about 330 more
observations per match than the geometric mean. Those additional observations
are of lower quality: median ambiguity 0.20, against 0.02 for the observations
both formulas accept; median distance from the robot's own pose estimate 0.25
to 0.28 m, against 0.07 m; and 11% to 26% of them more than a meter from the
estimate, against 0.1% to 0.2%. (The estimate stands in for the true
position, which was not measured; see section 21.) At an equal threshold, the
geometric mean rejects observations that are more often wrong.

An arithmetic mean with its threshold raised to about 0.69, however, makes the
same accept-or-reject decision as the current filter on all but 14 to 18
observations per match. On this data, the geometric mean behaves approximately
like an arithmetic mean with a slightly higher threshold.

The reasons follow from the curves described in section 2.

- Pitch, roll, and height score between 0.52 and 0.56 for nearly every
  observation, correct or not. They carry 2.1 of the 5.4 total weight and
  contribute almost no discrimination.
- Every one of the 14,303 observations was single-tag, so `unambiguous` ran on
  all of them. It and `distanceToTags` carry nearly all of the variation in
  the continuous scores: tags were 5 m or farther away for 37% to 39% of
  observations, where the distance test scores below 0.27.
- No off-field observation reaches the threshold under either formula, so the
  veto property is never exercised.

The geometric mean's distinguishing properties, the veto and the compounding
of partial failures, only affect the outcome when test scores spread out,
and under the current constants they mostly do not. That will change if the
tilt curves are made steeper or multi-tag observations become common, and the
formula is already in place for that case.

---

# Part 3. Tuning

## 7. Tuning parameters and procedure

### Parameters

| Parameter | Location | Effect |
|---|---|---|
| Test weights | The `Test` enum in `VisionFilter.java` | Each test's share of the combined score |
| `MIN_SCORE` | `VisionConstants.java` | The acceptance threshold (0.65) |
| `AMBIGUITY_TOLERANCE`, `PITCH_TOLERANCE`, `ROLL_TOLERANCE`, `ELEVATION_TOLERANCE`, `TAG_DISTANCE_TOLERANCE` | `VisionConstants.java` | The midpoint of each curve |
| Steepness (the last argument to `normalizedSigmoid` in each test) | `VisionFilter.java` | The slope of each curve |
| `CORRELATION_TIME_WINDOW_SECONDS`, `CORRELATION_POSE_THRESHOLD`, `CORRELATION_BOOST_FACTOR` | `VisionConstants.java` | When cameras count as agreeing, and the resulting boost |
| `SINGLE_CAMERA_STD_DEV_MULTIPLIER`, `LINEAR_STD_DEV_BASELINE` | `VisionConstants.java` | The conversion from score to standard deviation |
| `VELOCITY_UNCERTAIN_SCORE`, `VELOCITY_CHECK_TIMEOUT_SECONDS` | `VisionConstants.java` | Used only when `velocityConsistency` is enabled (sections 5 and 17) |
| `PROCESSING_INTERVAL_LOOPS`, `LOGGING_DIVISOR` | `VisionConstants.java` | The batch length in 20 ms loops (5) and the input-logging throttle (2); section 14 |

None of these can be changed at runtime; every change requires an edit, a
build, and a deploy.

Tuning presumes that the observations are individually sound: poses that are
consistently offset in one direction, or one camera that disagrees with the
other three, indicate an error upstream of the filter (Part 4), which no
filter setting corrects.

### Procedure

1. **Work on a branch.**

   ```
   git status
   git switch -c vision-tuning-lab
   ```

2. **Establish a baseline before changing anything.** Run the vision unit
   tests (section 16; these are JUnit tests, distinct from the filter's seven
   tests) and record the result.

   ```
   ./gradlew test --tests 'frc.robot.subsystems.vision.*'
   ```

   Read the line `N tests completed, M failed`. Gradle prints it only when at
   least one test fails; a run with no failures prints per-test `PASSED` lines
   and no count. `BUILD SUCCESSFUL` is printed regardless of test results,
   because `ignoreFailures` is set in `build.gradle`. The current baseline is
   **86 tests, 1 failed**. The failing test, "Rejects ambiguous PnP solution
   with wrong yaw", dates from when `yawConsistency` was in the default set
   and was not updated when the test was removed. Any change to that count
   indicates that the edit affected the suite.

3. **Change one parameter per run**, and record the expected effect before
   running.

4. **Revert after each experiment.**

   ```
   git checkout src/main/java/frc/robot/subsystems/vision/VisionFilter.java
   git checkout src/main/java/frc/robot/subsystems/vision/VisionConstants.java
   git status
   ```

5. **Verify in order.** Check a change by hand calculation, then with the
   playground test in section 8, the unit suite, log replay (Part 6), and
   finally on the practice field. Do not deploy a tuning change at an event.

### Rules

1. **The weights and the threshold are coupled.** Any weight change moves
   every score, so the threshold no longer falls at the same point in the
   score distribution. After changing one, re-examine the other.

2. **A weight of 0 disables a test.** Any score to the power 0 is 1, and the
   weight adds nothing to the total, so the test drops out of the result. Do
   not do this to `withinBoundaries` or `moreThanZeroTags`, since doing so
   removes the veto. Do not zero every weight: the computation produces NaN,
   and every comparison with NaN is false, so `Vision.java` logs the
   observation as rejected (`score > MIN_SCORE` is false) but does not remove
   it from the batch (`score < MIN_SCORE` is also false), and it reaches the
   estimator.

3. **Increasing the weight of a test that scores 1.0 loosens the filter.**
   `withinBoundaries` scores 1.0 for nearly every observation. Raising its
   weight from 1 to 2 adds nothing to the product but increases the root, so
   every score rises. For the accept-or-reject decision, doubling that weight
   is equivalent to lowering the threshold to 0.60 (0.65 to the power
   6.4/5.4).

4. **Keep `MIN_SCORE` below the score ceilings.** Because pitch, roll, and
   height cap at 0.52 to 0.56, no three-tag observation can score above 0.783
   and no single-tag observation above 0.734 (section 3). Those limits assume
   zero ambiguity and a tag at zero distance; the three-tag observation in the
   unit test "Perfect observation scores well" (ambiguity 0.01, tags 2 m
   away) scores 0.775. A `MIN_SCORE` of 0.75 rejects every single-tag
   observation, and the unit tests do not detect this.

### Symptoms and adjustments

| Symptom | Adjustment |
|---|---|
| Correct observations rejected (low accepted count, estimate drifts between corrections) | Lower `MIN_SCORE` slightly, or lower the weight of the test pulling scores down |
| Wrong observations accepted (estimate jumps more than a meter and returns) | Raise `MIN_SCORE` slightly, or raise the `unambiguous` weight |
| Ambiguous single-tag observations accepted | Lower `AMBIGUITY_TOLERANCE` or raise the `unambiguous` weight |
| Cameras never fuse (`FusedCameraCount` always 1) | Widen `CORRELATION_POSE_THRESHOLD` or `CORRELATION_TIME_WINDOW_SECONDS`, or check camera calibration |

To measure the effect of an adjustment, count jumps in `RobotPosesAccepted`
and plot the `ObservationScore` distribution of the accepted observations, as
in section 5. If the score distributions of correct and wrong observations
overlap, moving the threshold only trades one kind of error for the other,
and a test has to change first (section 5).

## 8. Exercises

Each exercise runs on the tuning branch from section 7.

### Exercise 1: hand calculation

Score the ambiguous observation from section 3 by hand: single tag, ambiguity
0.4, level, 3 m from the tag. The per-test scores are 1, 1, 0.269, 0.522,
0.522, 0.562, and 0.731, with weights 1, 1, 0.8, 0.7, 0.7, 0.7, and 0.5.
Compute the weighted geometric mean and the weighted arithmetic mean.

Expected: geometric 0.627, arithmetic 0.686. Only the arithmetic mean accepts
the observation.

### Exercise 2: scoring playground

Create `src/test/java/frc/robot/subsystems/vision/ScoringPlaygroundTest.java`
with the following content. It builds one observation, runs it through the
real filter, and prints each test's score.

```java
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import org.junit.jupiter.api.Test;

/** Scoring playground. Edit the inputs, run, read the output. Delete when done. */
class ScoringPlaygroundTest {

  @Test
  void printScores() {
    var filter = new VisionFilter();

    // Inputs. Position x, y, z in meters. Roll, pitch, yaw in radians.
    var pose = new Pose3d(8.0, 4.0, 0.0, new Rotation3d(0.0, Math.toRadians(0.0), 0.0));
    double ambiguity = 0.4; // 0 = one clear solution, 1 = two equally good solutions
    int tagCount = 1;
    double averageTagDistance = 3.0; // meters

    // Timestamp 0.0; the type field is not read by any test.
    var observation =
        new PoseObservation(
            0.0, pose, ambiguity, tagCount, averageTagDistance, PoseObservationType.MEGATAG_1);
    // Camera index 0, no velocity history (null, 0.0), no heading reference (null).
    var result =
        filter.scoreObservation(observation, 0, null, 0.0, null, VisionFilter.DEFAULT_ENABLED_TESTS);

    result.testResults().forEach((t, s) -> System.out.printf("%-17s weight %.1f  score %.3f%n", t, t.weight(), s));
    String verdict = result.score() > VisionConstants.MIN_SCORE ? "ACCEPT" : "REJECT";
    System.out.printf("total %.3f -> %s (threshold %.2f)%n", result.score(), verdict, VisionConstants.MIN_SCORE);
  }
}
```

Run it:

```
./gradlew test --tests 'frc.robot.subsystems.vision.ScoringPlaygroundTest'
```

Expected: the seven scores and `total 0.627 -> REJECT`. The build runs
Spotless before compiling, and it rewraps the three lines that exceed 100
columns; the reformatted file behaves the same.

Then change the inputs and re-run, recording the expected result first.

1. Ambiguity 0.01. Expected 0.712, accepted. Keep this value for the remaining
   steps.
2. Pitch 20 degrees. Expected 0.696. The pitch score moves from 0.522 to 0.435.
3. Pitch 0, tag count 3, distance 2.0 m. Expected 0.775. `unambiguous` is now
   1.000.
4. Position (-5, -5). Expected 0.000. Identify the test responsible.

Delete the file afterward. It is not part of the test suite.

### Exercise 3: weight change

In `VisionFilter.java`, change `distanceToTags(0.5)` to `distanceToTags(1.5)`.
Run the vision suite. Expected: still 1 failed (the count reads 87 tests if
the playground from exercise 2 is still present, because it adds one). The
suite contains no test that pins a nonzero combined score, so a weight change
causes no test to fail. If the playground is still present with ambiguity
0.4, its combined score (the `total` line) moves from 0.627 to 0.642. Revert
with `git checkout`.

### Exercise 4: threshold above the ceiling

In `VisionConstants.java`, set `MIN_SCORE` to 0.80. Run the suite. Expected:
2 failed. The new failure is "Perfect observation scores well", whose
three-tag observation scores 0.775. Set `MIN_SCORE` to 0.75 and run again.
Expected: 1 failed, although 0.75 rejects every single-tag observation the
robot will ever produce (section 7, rule 4). Revert.

### Exercise 5: zero weight

In `VisionFilter.java`, change `withinBoundaries(1.0)` to
`withinBoundaries(0.0)`. Run the suite. Expected: 4 failed. "Observation
outside field scores zero", "Can disable specific tests", and "Velocity check
uses previous pose" fail because the boundary veto is gone (the last compares
an in-field pose with one at x = 18 m, and with `velocityConsistency`
disabled only the veto told them apart); the fourth is the known yaw failure.
Revert, and confirm with `git status` that the tree is clean.

Afterward:

```
git switch main
git branch -D vision-tuning-lab
```

---

# Part 4. Calibration and validation

## 9. Sources of error upstream of the filter

The filter decides which observations to trust; it cannot correct a wrong
one. A miscalibrated camera, or one mounted a degree off, produces
observations that are all wrong in the same direction, none of which looks
unusual to the filter, and the pose estimator converges on the wrong position
with full confidence. Four elements of the pipeline must therefore be correct
before the filter is even relevant.

1. **Camera intrinsics.** The camera's internal model: focal length, optical
   center, lens distortion.
2. **Camera extrinsics.** Each camera's position and orientation on the robot.
3. **The AprilTag field layout.** The tags' true positions on the field.
4. **The pose pipeline.** The software that turns a tag in an image into a
   robot pose. Part 5 covers it.

An error in any one of these makes the estimate wrong, and the errors add. The
first three can be checked with a tape measure and a few minutes on the
practice field.

## 10. Camera calibration (intrinsics)

Calibration estimates the lens's focal length, optical center, and distortion
coefficients. PhotonVision uses OpenCV's calibration routine with a printed
chessboard or ChArUco board.

### Significance

Calibration error is the most consequential source of pose error, since it
affects every observation from that camera regardless of distance. A poorly
calibrated camera at 4 m can produce 10 to 15 cm of systematic pose error,
and no filtering can remove it, because the observation itself is wrong.

### Procedure

1. Mount the board on a rigid backing such as foam board, MDF, or a
   clipboard. A flexible sheet warps between images, and the calibration
   absorbs the warp as lens distortion.
2. Calibrate at the operating resolution, the one set for the camera in
   PhotonVision's camera settings. A calibration taken at one resolution is
   invalid at another.
3. Take at least 25 images with full frame coverage. This is the most common
   omission. Distortion is largest at the corners and edges, so the board
   must appear there; if every image has the board in the center, the edge
   distortion coefficients are unconstrained. Tilt the board 30 to 45 degrees
   toward and away from the camera and rotate it. Vary the distance so that
   the board fills about half the frame in some images and about a fifth in
   others.
4. Check the calibration's reprojection error, which PhotonVision's
   calibration page reports: the mean distance, in pixels, between where the
   fitted lens model predicts the board corners and where they were detected.
   The target is under 0.5 pixels; under 0.3 is excellent; above 1.0 indicates
   a failed calibration. This is a different quantity from the logged
   `Vision/CameraN/BestReprojError` of section 13, which comes from multi-tag
   pose solves on the field.

### When to recalibrate

- After changing the lens, focus, or resolution.
- After the camera is disturbed: remounted, dropped, or struck, or after an
  impact near its mount.
- At least once per build season.

The OV2311 cameras have fixed-focus lenses and do not drift the way an
auto-focus camera would.

## 11. Camera mounting (extrinsics)

The `CameraConfig` entries in `VisionConstants.java` give each camera's
position relative to the robot center and its orientation. The position is in
inches and the rotation is stored as a quaternion, which cannot be checked by
inspection. The comment on each entry gives the pitch and yaw in degrees.

| Camera | Log index | Position (x, y, z in inches) | Pitch | Yaw |
|---|---|---|---|---|
| Front right | 0 | (-10.572, -12.337, 16.688) | 20 degrees up | 55 degrees right |
| Front left | 1 | (-10.572, 12.337, 16.688) | 20 degrees up | 55 degrees left |
| Back right | 2 | (-13.1623, -12.1623, 20.26674) | 15 degrees up | 135 degrees right |
| Back left | 3 | (-13.1623, 12.1623, 20.26674) | 15 degrees up | 135 degrees left |

Positions are in the robot frame, x forward, y left, z up, from the robot
center at floor level. The log index is the camera's position in the `Vision`
constructor call in `Robot.java` and is the N in `Vision/CameraN`.

### Sensitivity

A mount position error shifts every pose by the same amount regardless of
tag distance: 1 cm in the transform is 1 cm in every pose.

A mount angle error also does not grow with tag distance in this pipeline.
PnP places the camera in the field from the image, the lens model, and the
tag layout, with no reference to the mount. The mount transform is composed
afterward, `fieldToCamera.plus(robotToCamera.inverse())` in
`VisionIOPhotonVision.java`, for single-tag and multi-tag results alike. An
angle error therefore rotates the robot pose by that angle and shifts its
position by about the camera's distance from the robot center (0.6 to 0.7 m)
times the angle: 2 degrees is 1.5 to 2 cm of position error at 1 m, 4 m, or
8 m, together with 2 degrees of heading error for a yaw error or 2 degrees
of tilt for a pitch or roll error. Yaw is the mount angle that matters most
for the 2D estimate, because a yaw error appears directly as a heading error
of the same size in every observation and the estimator fuses vision
heading. A pitch or roll error appears as pitch or roll in the 3D pose, which
the filter scores (section 2). Hence the distance test in section 13
attributes a constant offset to the mount, and the rotation test attributes a
heading disagreement to it.

What does grow with distance is an error in the bearing from the camera to
the tag: a lens model that is wrong (section 10), or a tag whose orientation
on the field differs from the layout (section 12). A bearing error rotates
the camera's computed position about the tag, so the pose error is the
distance times the angle.

| Error | Pose error at 1 m | At 2 m | At 4 m |
|---|---|---|---|
| 1 cm in mount position | 1 cm | 1 cm | 1 cm |
| 2 degrees in mount angle | 1.5 to 2 cm, plus 2 degrees of rotation | same | same |
| 1 degree of bearing | 1.75 cm | 3.5 cm | 7 cm |
| 2 degrees of bearing | 3.5 cm | 7 cm | 14 cm |
| 5 degrees of bearing | 8.75 cm | 17.5 cm | 35 cm |

### Measurement

**From CAD.** If the robot is fully modeled, the mount positions and angles
can be read from the model to sub-millimeter and sub-degree precision,
provided the build matches the model.

**By hand.** Measure with calipers or a tape from the center of the drivebase
at floor level to the front element of each lens, and measure pitch with a
digital level or angle finder; a phone inclinometer is adequate. Target pitch
within 1 degree and position within 5 mm.

**Refinement on the field.** Place the robot at a precisely known position,
log each camera's individual pose estimate, compare with the true position,
and adjust the transforms to remove the offset. CAD values are typically
within a degree or a centimeter, and field measurement corrects the
remainder. The single-camera test in section 13 identifies whether this is
needed.

## 12. AprilTag field layout

The pose is computed relative to the tag positions in the layout file. If the
layout places a tag at (13.00, 0.60) and the tag is at (13.02, 0.58), every
pose computed from that tag is 2 cm off.

### Field accuracy

FIRST's field specifications allow roughly half an inch of tolerance on tag
placement. Actual variance depends on the venue.

- Tags on the perimeter walls are generally accurate; the walls are rigid.
- Tags on game elements assembled per event by volunteers (the hub, in 2026)
  vary more.
- Championship fields are typically the most precise. District and regional
  fields vary.
- Practice fields, and custom-built practice elements in particular, can
  depart substantially from the layout.

A tag placement error moves every single-tag pose computed from that tag by
the same amount at any distance: one inch of placement error is 2.5 cm of
pose error. A tag whose orientation is off by one degree moves the pose by
about 1.7 cm per meter of distance. Multi-tag solutions are more robust
because the errors partly cancel.

### Custom layouts

The robot code can load a custom layout in place of the official one. Set
`USE_CUSTOM_APRIL_TAG_LAYOUT` to true; `CUSTOM_APRIL_TAG_LAYOUT_PATH` already
names `stemgym-2026.json` in `src/main/deploy`, the layout of the team's
practice gym, and a layout for another venue goes in the same directory, in
the format of WPILib's official field layout files. On the robot this layout
affects only single-tag observations, which `VisionIOPhotonVision` computes
from it. Multi-tag observations arrive from PhotonVision already solved
against the layout loaded on the coprocessor, so the same file also has to be
uploaded to every PhotonVision coprocessor, and the official layout restored
there before competition.

The robot code skips the custom layout when the driver station is attached to
the field management system, but the check runs once, the first time the
layout is needed (the first frame in which a camera sees a tag), and the
result is cached for the life of the program (`cachedLayout` in
`Vision.java`). A robot that sees a tag before the driver station connects to
the field keeps the custom layout for the whole match, so the flag must be
false in any build deployed for competition. Custom layouts are for practice
fields.

Measuring tag positions precisely is slow and practice time at an event is
limited, so a custom layout is justified only when practice matches show a
consistent offset (e.g., the robot always believing it is 3 cm left of its
true position).

## 13. Validation procedures

Validation compares the robot's estimate with a measurement of its true
position.

### Once per season, after the robot is built

**Single-camera isolation test.** This is the most informative check
available. Place the robot at a precisely known position, for example aligned
to a field marking, and record each camera's estimate on its own. The
simplest way is to set `LOG_INDIVIDUAL_CAMERA_POSES` to true in
`VisionConstants.java`, deploy, and read `Vision/CameraN/RobotPoses` for each
camera from one log (the raw observations are also always available as
`Vision/CameraN/PoseObservations`). The alternative is to enable one camera
at a time by temporarily commenting out the others where the cameras are
constructed in `Robot.java`, in the REAL case, which also renumbers the
remaining cameras. All four should agree within 3 cm. A camera that disagrees
has a calibration or mounting error.

**Distance test.** Place the robot a measured distance from a tag. Start at 1
m and step back in 0.5 m increments to 4 m, comparing the logged vision poses
(`Vision/Summary/RobotPosesAccepted`, or `Vision/CameraN/RobotPoses` per
camera) with the tape at each stop. Accuracy that degrades with distance
indicates a calibration (intrinsics) problem. A constant offset at every
distance indicates a mounting (extrinsics) problem.

**Rotation test.** Rotate the robot in place and compare each camera's vision
heading (the yaw of its poses in `Vision/CameraN/RobotPoses`) with the gyro
heading (`Drive/Gyro/YawPosition`). The raw gyro's zero is arbitrary, so
compare the change in each heading over the rotation, not the absolute
values. A camera whose heading differs from the gyro's by an offset the other
cameras do not share has a yaw mounting error.

### At every event

**Wall test.** Place the robot flat against a known field wall. The estimate
should be half the robot's width, bumpers included, from the wall
(`MIN_ROBOT_WIDTH` in `VisionConstants.java` is 36.875 in, so 0.47 m if the
bumpers match it). Repeat against a perpendicular
wall to check both axes. This takes about 30 seconds and catches gross errors.

**Practice match review.** After the first practice match, open the log in
AdvantageScope and check:

- `Vision/Summary/RobotPosesAccepted`: the accepted poses should track smoothly
  and cluster where the robot was.
- `Vision/CameraN/BestReprojError`: compare with values from the practice
  field. A jump suggests this field's tags are placed differently. The value
  only updates on multi-tag solutions, so on a field where the cameras see
  single tags it may never change.
- `Vision/Summary/FusedCameraCount`: a value that is always 1 means cameras
  are not agreeing with each other. Either the agreement threshold is too
  tight or a camera is miscalibrated. Fusion that worked on the practice field
  and not at the event indicates something changed in transit.

### Between matches

Check the log for periods with mostly rejected poses (a camera fault or an
obstructed view), periods where the estimate was visibly wrong, and the
`Faults/Vision/CameraNDisconnected` flags.

### Symptoms and causes

| Symptom | Likely cause | Action |
|---|---|---|
| All cameras offset in the same direction | Field tags differ from the layout | Measure tags; deploy a custom layout |
| One camera disagrees with the others | Its mount moved or its calibration is bad | Re-check the mount; recalibrate |
| Accuracy degrades with distance | Poor calibration | Recalibrate with full frame coverage |
| Estimated heading is rotated relative to reality | Camera yaw is wrong | Re-measure the mount angle |
| Reprojection error higher than on the practice field | Field assembly variance | Measure the tags; deploy a custom layout |
| No multi-camera fusion | Cameras disagree by more than 0.15 m | Check each camera individually |

### Height as a diagnostic

The z coordinate of every observation should be near zero. The `heightError`
test scores an observation lower as its z moves away from zero, crossing 0.5
at 0.25 m. A consistently nonzero z indicates a problem upstream of the
filter.

- **Positive z** (the robot appears above the floor): the camera height in
  the extrinsics is smaller than the real height, by the same amount.
- **Negative z**: the extrinsics height is larger than the real height.
- **z that depends on which tag is in view**: that tag's placement is off, or
  the calibration has view-dependent distortion.

A mount pitch error contributes only a few millimeters of z per degree,
because it acts on the camera's offset from the robot center rather than on
the tag distance (section 11); it shows up instead as pitch or roll in the 3D
pose. Plotting the z component of `Vision/Summary/RobotPosesAccepted` across
a match is cheap and exposes a height error directly: a steady 5 cm most
likely means that the camera height in the extrinsics is 5 cm off.

### Priorities

1. Check the calibration reprojection error for all four cameras in the
   PhotonVision interface (section 10). Recalibrate any above 0.5 pixels.
2. Run the single-camera isolation test. It catches the most common problems
   in one step.
3. Do the wall test at every event.
4. Plot z across a practice match.
5. Review the log after every practice match.

None of these require a code change.

---

# Part 5. Implementation

## 14. Source files and data flow

| File | Role |
|---|---|
| `VisionIO.java` | The interface for a camera: connected, frame rate, latency, best reprojection error, pose observations, tag IDs. Defines `PoseObservation`. |
| `VisionIOPhotonVision.java` | Reads one real PhotonVision camera over NetworkTables and converts each result to a `PoseObservation`. |
| `VisionIOPhotonVisionSim.java` | The same for a simulated camera viewing a simulated field. |
| `VisionThread.java` (in `frc/robot/util`) | A background thread that polls every camera at 50 Hz so network reads do not block the main loop. |
| `Vision.java` | The subsystem. Copies the latest snapshots, logs them, scores every observation, batches, fuses, and passes results to the drive. |
| `VisionFilter.java` | The tests, the geometric mean, and fusion. No hardware dependencies, which is what makes it unit-testable. |
| `VisionConstants.java` | All constants: camera transforms, tolerances, the threshold, fusion settings, logging flags. |
| `Drive.addVisionMeasurement` (in `Drive.java`) | Receives fused observations and feeds WPILib's pose estimator. |

### Path of one observation

```
PhotonVision, on the coprocessor
  detects the tags in a frame, solves the camera pose, and publishes the
  result with its capture timestamp
        |  NetworkTables
        v
VisionThread, in the background every 20 ms
  VisionIOPhotonVision.updateInputs drains the unread results, converts
  each to a PoseObservation, and stores a snapshot per camera
        |  snapshot copy
        v
Vision.periodic, on the main loop every 20 ms
  logs the inputs under Vision/CameraN (every other loop), scores each
  observation with scoreObservation, appends it to the batch, and logs
  its pose as accepted or rejected
        |  every fifth loop
        v
the batch
  drop score < MIN_SCORE, fuseCorrelatedObservations, sort by timestamp,
  compute the standard deviations, log ObservationScore
        |  one call per fused observation
        v
Drive.addVisionMeasurement
  WPILib's swerve pose estimator blends each measurement with odometry
```

The steps in detail:

1. **On the coprocessor**, the separate small computer on the robot that runs
   PhotonVision. PhotonVision detects AprilTags in a frame and solves for the
   camera pose. It publishes the result over NetworkTables, the protocol the
   roboRIO and the coprocessors use to share values over the robot network,
   with the capture timestamp.
2. **On the background thread.** Every 20 ms, `VisionThread` calls
   `updateInputs` on each camera's IO. `VisionIOPhotonVision` drains all unread
   results. For a multi-tag result it takes PhotonVision's best
   field-to-camera transform and applies the inverse of the camera's mount
   transform to obtain field-to-robot. For a single-tag result it reads the
   tag's pose from the field layout and chains tag-to-camera and
   camera-to-robot. Either way the result is a `PoseObservation`: timestamp,
   3D pose, ambiguity, tag count, average tag distance. The thread stores an
   immutable snapshot so that the main loop never waits on the network.
3. **In `Vision.periodic`, every 20 ms.** The subsystem copies each camera's
   snapshot and, every other loop, logs the inputs under `Vision/CameraN` (the
   throttle is `LOGGING_DIVISOR`, which is why a replay sees half the frames;
   see Part 6). For every observation, it computes the score with
   `scoreObservation`, appends the observation to the batch (a buffer of the
   observations scored since the last fusion pass), and logs the pose as
   accepted or rejected according to the threshold.
4. **Every fifth loop** (`PROCESSING_INTERVAL_LOOPS`, 5). The subsystem
   removes from the batch everything scoring below the threshold
   (`score < MIN_SCORE`; the accepted-or-rejected logging in step 3 uses
   `score > MIN_SCORE`, so a score exactly equal to 0.65 is logged as rejected
   but still sent), fuses the remainder (section 15), sorts by timestamp,
   computes the standard deviations (section 4), and calls
   `drive::addVisionMeasurement` for each fused observation, recording
   `ObservationScore` and `FusedCameraCount` after each call. AdvantageKit
   keeps one value per key per loop, so when a batch sends several fused
   observations only the last one's score and camera count reach the log. It
   then clears the batch.
5. **In the drive.** The first vision estimate received while the robot is
   disabled resets the pose estimator to that pose (`setPose`), so the robot
   has a position before the match starts. Every measurement, that first one
   included, then goes into WPILib's swerve pose estimator, which weights each
   against odometry by its standard deviation.

`scoreObservation` packs the observation, its camera index, that camera's last
accepted pose and timestamp, and the heading reference into a `TestContext`,
and every enabled test reads its inputs from that object. The playground in
section 8 passes null for both references, which the seven enabled tests
never read.

The batching interval exists because the cameras do not fire together: the
analysis behind the fusion change found them reporting 80 to 150 ms apart, a
figure that now survives only in the comment on
`CORRELATION_TIME_WINDOW_SECONDS` and that explains why that window is 150 ms
rather than the original 50 ms. A 100 ms batch gives all four a chance to
report before fusion decides which agree.
The cost is up to 100 ms of added latency on corrections, which the estimator
tolerates because every observation carries its own capture timestamp.

The background thread exists because NetworkTables reads can block, and a
block on the main loop is a loop overrun.

### Logged keys

| Key | Content | Frequency |
|---|---|---|
| `Vision/CameraN/PoseObservations`, `TagIds`, `Connected`, `Fps`, `LatencyMs`, `BestReprojError`, `LatestTargetObservation` | Raw inputs from camera N | Every other loop |
| `Vision/Summary/RobotPosesAccepted`, `RobotPosesRejected` | The poses of the observations scored in that loop, split by the threshold (`LOG_ACCEPTED_POSES` and `LOG_REJECTED_POSES`, both true by default). The arrays are rebuilt every loop but written only on logged loops, so poses scored on the other loops are never recorded | Every other loop |
| `Vision/Summary/RobotPoses`, `TagPoses` | All poses scored that loop and the tags in view | Only when `LOG_SUMMARY_POSES` is true (default false), every other loop |
| `Vision/Summary/ObservationScore`, `FusedCameraCount` | Score and camera count of the last fused observation sent in that batch. AdvantageKit keeps one value per key per loop, so earlier ones in the same batch are overwritten, and the log writer records a value only when it differs from the previous record | Every fifth loop |
| `Faults/Vision/CameraNDisconnected` | Camera N is not responding | Every loop |
| `Vision/CameraN/RobotPoses`, `TagPoses`, `PassRate`, and related | Per-camera detail | Only when `LOG_INDIVIDUAL_CAMERA_POSES` is true (default false) |

Per-test scores and the standard deviations are not logged; both are listed
in section 23.

### Simulation

In SIM mode, `VisionIOPhotonVisionSim` builds a PhotonVision `VisionSystemSim`
with the official tag layout and four simulated cameras matching the real
ones: 800 by 600 pixels, 70-degree diagonal field of view, 35 frames per
second, about 30 ms latency, and a quarter pixel of calibration noise. On each
update it moves the simulated robot to the drive's current pose estimate, and
the simulated cameras report what they would see from there.

The simulation is therefore self-referential. It exercises the pipeline, the
logging keys, and the veto: the drive's estimate starts at (0, 0) unless an
auto or a pose reset moves it, and (0, 0) lies in the half-robot-width margin
that `withinBoundaries` excludes, so every observation scores zero and
`RobotPosesAccepted` stays empty. It cannot, however, assess accuracy; replay
(Part 6) can.

## 15. Multi-camera fusion

`fuseCorrelatedObservations` merges observations from cameras that agree.

1. Every observation in the batch starts in its own cluster.
2. For every pair of observations from different cameras, if the timestamps
   are within `CORRELATION_TIME_WINDOW_SECONDS` (0.150 s) and the positions
   within `CORRELATION_POSE_THRESHOLD` (0.15 m), the two clusters are merged.
3. For each cluster containing at least two distinct cameras: the positions
   are averaged, weighted by score; the headings are averaged the same way but
   as a circular mean, so that 359 degrees and 1 degree average to 0; the
   timestamps are averaged with the same score weights; and the score is the
   highest in the cluster multiplied by `CORRELATION_BOOST_FACTOR` (1.4),
   capped at 1.0. The result is one `FusedObservation` whose `cameraCount` is
   the number of cameras.
4. Each single-camera cluster passes through unchanged with `cameraCount` 1.

Downstream, `cameraCount` 1 selects the 3x standard-deviation multiplier. The
motivation is jitter: four cameras each reporting a slightly different pose,
all accepted, make the estimate oscillate. A fused pose is already an average
and so receives full weight, whereas a single-camera pose receives a third of
it.

Note two characteristics of the current implementation.

- **Any two cameras, not a majority.** The design notes for the March 2026
  branch (the former `doc/VISION_TESTS.md`) and the comment on
  `CORRELATION_BOOST_FACTOR` describe boosting a cluster only if it contained
  a majority of the cameras that reported, so that a two-versus-two split
  boosted neither side; no version of `VisionFilter.java` on main has that
  rule. The current code merges any two agreeing cameras. In a two-versus-two
  split it produces two fused observations, both boosted, in different places.
  This has not caused a problem in logged matches, where the four cameras
  rarely split evenly, but it is a known trade-off.
- **The batch is capped at 32 observations** (`MAX_OBSERVATIONS` in
  `VisionFilter.java`). Above that, fusion is skipped and every observation
  passes through as single-camera.

## 16. Unit tests

`src/test/java/frc/robot/subsystems/vision/VisionFilterTest.java` contains 86
unit tests. In this section, "test" alone means a unit test; the filter's own
tests are called filter tests.

```
./gradlew test --tests 'frc.robot.subsystems.vision.*'
```

The groups, in file order:

| Group | Coverage |
|---|---|
| One group per filter test, named `Test.unambiguous`, `Test.pitchError`, `Test.rollError`, `Test.heightError`, `Test.withinBoundaries`, `Test.moreThanZeroTags`, `Test.distanceToTags`, `Test.velocityConsistency`, and `Test.yawConsistency` in the Gradle output | Each filter test's curve, edge cases, and symmetry |
| `scoreObservation` | The weighted geometric mean, enabling and disabling filter tests, compounding of failures |
| `fuseCorrelatedObservations` | Clustering, weighted averages, the boost and its cap |
| `normalizedSigmoid` | The curve function |
| `TestContext` | The object that carries an observation and its history through the filter tests |
| `Integration and Edge Cases` | End-to-end scoring scenarios |

The suite has three limitations, which the exercises in section 8
demonstrate.

- **No golden values.** The suite checks orderings (a good observation
  outscores a bad one; more tags outscore fewer) and the binary cases (a
  score of exactly 0 or 1), but it pins no combined score other than zero, so
  a weight change causes no test to fail.
- **No check of the threshold against the ceilings.** A `MIN_SCORE` of 0.75
  adds no failure (the only test that requires a score above `MIN_SCORE` uses
  a three-tag observation scoring 0.775) yet rejects every single-tag
  observation.
- **Failures do not fail the build.** `ignoreFailures` is set in
  `build.gradle`, so `BUILD SUCCESSFUL` says nothing about the tests. The
  `FAILED` lines and the count line, which Gradle prints only when at least
  one test fails, are the indicators (section 7).

One test is known to fail on main: "Rejects ambiguous PnP solution with wrong
yaw" in the `Test.yawConsistency` group (Gradle prints it as
`Test.yawConsistency > Rejects ambiguous PnP solution with wrong yaw`). It was
written when `yawConsistency` was in the default set and not updated when the
test was removed.

The build runs Spotless before compiling, so any edited file is reformatted to
Google Java style.

## 17. Disabled tests

`VisionFilter.java` defines nine tests, of which seven are enabled.

### velocityConsistency

Compares each observation with the last accepted observation from the same
camera and computes the implied speed. The score is the same sigmoid as the
other continuous tests, with its midpoint at `MAX_REASONABLE_VELOCITY_MPS`
(1.5x `DRIVETRAIN_SPEED_LIMIT`, about 6.2 m/s) and steepness 2 per m/s: 0.99
at 4 m/s, 0.5 at 6.2 m/s, 0.02 at 8 m/s. With no history, or history older
than `VELOCITY_CHECK_TIMEOUT_SECONDS` (0.5 s), it returns
`VELOCITY_UNCERTAIN_SCORE` (0.6) rather than 1.0, for the reasons in section
5.

The comparison is against the same camera's history rather than the robot's
own velocity for two reasons: the pose estimator's velocity is partly derived
from vision, so using it to judge vision would be circular, and a robot may
be carried, pushed, or lifted, so wheel speed is not robot speed.

The test has run on the robot only in the March 18, 2026 practice session
of section 5, on a `vision-tests` branch build that had it enabled and wrote
the per-camera history. It was commented out of `DEFAULT_ENABLED_TESTS` the
same day, after a replay showed it rejected nothing that `yawConsistency` did
not; the March 19 merge carried that state, and the test has been disabled
since. It would also
not function if enabled: `Vision.java` allocates the `lastAcceptedPose` and
`lastAcceptedTimestamp` arrays and passes them to the filter, but never
writes to them. Every observation would see "no history" and score 0.6 on
this test, which at weight 0.9 pulls the typical single-tag total from 0.712
to 0.695 and the three-tag total from 0.775 to 0.747 while catching nothing.
The branch wrote those arrays after each accepted observation, and the writes
were lost when the batch-and-fuse loop was rewritten for the merge; the fix
is to restore them.

### yawConsistency

Compares the heading in the observation with the heading the robot already
holds (weight 1.0). A mirror-image PnP solution almost always has a badly
wrong heading, typically 30 to 90 degrees off, because the mirror ambiguity is
a rotation. A heading check therefore targets exactly the failure described in
section 5. The tolerance is `YAW_TOLERANCE`, 10 degrees, with steepness 4 per
radian, the same curve as `unambiguous`: a heading that matches the reference
scores 0.67, a 10 degree error 0.50, 30 degrees 0.20, and 90 degrees 0.004.

The reference heading comes from `drive::getFieldRelativeHeading`, which is
the pose estimator's own heading and is null until the pose has been
initialized; with a null reference the test returns 1.0.

The test was enabled for the first morning of the VACHE event in March 2026
and then removed. That morning's build compared vision against the raw
power-on gyro, which is in a different frame from the field. The two
disagreed by about 90 degrees throughout, and vision produced no accepted
observations for two full matches (qualification matches 4 and 10). A midday
rebuild removed the test. The defect was in the reference, not in the test
logic.

An offline re-scoring of all 14 VACHE match logs in September 2026, by the
method of section 6 rather than the log replay of Part 6, measured the test's
value. Among observations the current filter accepts, those whose heading
differs from the robot's own estimate by more than 20 degrees are more than a
meter from that estimate in about two cases out of three, while those within
10 degrees almost never are (as in section 6, the estimate stands in for the
true position). The yaw check would have removed about 40% of the accepted
observations that were more than a meter off (81 of 210 over the ten matches
with usable data), including every multi-meter mirror flip, at a cost of
under 2% of correct observations. The same re-scoring found one match in
which the robot had been placed 180 degrees from the auto's starting heading;
with the test enabled, vision would have produced no accepted observations
for that entire match, with no recovery path.

Re-enabling it requires four changes: (a) a reference heading that is checked
against the selected auto before the match; (b) sampling of that reference at
the observation's timestamp rather than at scoring time, about 66 ms later,
which halves the tail of the heading noise; (c) skipping the test while the
robot is disabled; and (d) a sanity bound on observation timestamps, since
167 PhotonVision observations in those logs were stamped 38 to 52 seconds in
the future.

---

# Part 6. Log replay

## 18. Overview

AdvantageKit records the inputs the robot code receives, including camera
observations (from every other loop; see section 21), encoder readings, gyro
samples, and driver input, into a `.wpilog` file. Replay runs the robot code
on a development machine, feeds it the recorded inputs in order as fast as
possible, and records what the code does with them. The output is a new file,
`<name>_sim.wpilog`, containing two sets of outputs:

- `RealOutputs`: what the robot did during the original run, copied from the
  source log.
- `ReplayOutputs`: what the replayed code did with the same inputs.

Since the filter is deterministic, identical inputs produce identical outputs
unless the code changed, which makes replay the closest available
approximation to running a change in a real match. A full match replays in
about 4 seconds.

## 19. Procedure

This procedure was verified in September 2026 on the VACHE E8 log against the
current main. The commands are for macOS or Linux. On Windows, use
`gradlew.bat` and make the edits by hand.

**1. Clone into a throwaway directory.** Replay requires two edits that must
never be committed.

```
git clone /path/to/Rebuilt ~/replay-lab && cd ~/replay-lab
```

**2. Set replay mode.** In `Constants.java`, change `simMode` from `Mode.SIM`
to `Mode.REPLAY`.

```
sed -i '' 's/simMode = Mode.SIM;/simMode = Mode.REPLAY;/' src/main/java/frc/robot/Constants.java
```

On Linux, omit the `''` after `-i`.

**3. For logs recorded by a build older than June 4, 2026, restore the old
module names.** PR #192 renamed the swerve modules' log keys from
`Drive/Module0` through `Drive/Module3` to `Drive/ModuleFrontLeft`,
`Drive/ModuleFrontRight`, `Drive/ModuleBackLeft`, and
`Drive/ModuleBackRight`. Older logs carry the old keys, and without this
edit the replay crashes in the launcher code when auto starts. In
`Drive.java`, where the four `Module` objects are constructed, change the
names back to `"0"`, `"1"`, `"2"`, `"3"`.

```
sed -i '' -e 's/"FrontLeft")/"0")/' -e 's/"FrontRight")/"1")/' -e 's/"BackLeft")/"2")/' -e 's/"BackRight")/"3")/' src/main/java/frc/robot/subsystems/drive/Drive.java
```

**4. Run the baseline.** The match logs are not in the repository; copy the
one to be replayed into a directory with a short path, such as
`~/replay-logs`, which the commands below assume:

```
AKIT_LOG_PATH=~/replay-logs/e8.wpilog ./gradlew simulateJava
```

The process exits when the log ends. Two checks are required, because
`BUILD SUCCESSFUL` is printed even when the replay has crashed:

- The console must not contain `The robot program quit unexpectedly`. With a
  March log and no module-name edit, it does: a `NullPointerException`
  reported at `Launcher.aim`, because `Drive` computes its chassis speeds
  only while processing module odometry samples, and none are found under
  the new keys.
- The output file must be full size. A complete E8 replay is about 41 MB. A
  crashed one is about 8 MB.

Rename the output so the next run does not overwrite it:

```
mv ~/replay-logs/e8_sim.wpilog ~/replay-logs/e8_sim_baseline.wpilog
```

**5. Apply the change and run again.** Edit the weight, threshold, or
tolerance under test, run the same command, and rename the output to
identify the change:

```
mv ~/replay-logs/e8_sim.wpilog ~/replay-logs/e8_sim_dist15.wpilog
```

**6. Compare.** The repository's comparison script requires Python 3 and
`pip3 install msgpack`.

```
python3 scripts/compare_vision_logs.py ~/replay-logs/e8.wpilog ~/replay-logs/e8_sim_baseline.wpilog
python3 scripts/compare_vision_logs.py ~/replay-logs/e8.wpilog ~/replay-logs/e8_sim_dist15.wpilog
```

Run it once per replay and compare the two reports. Section 20 describes what
to read.

**7. Delete the clone afterward.** Never push from it.

A `replayWatch` Gradle task re-runs the replay on every file save. It has not
been tried on this project.

## 20. Interpreting the comparison

The script prints a long report, of which the relevant parts are described
below using the September verification as the example (baseline main against
the `distanceToTags` weight raised from 0.5 to 1.5).

**`ReplayOutputs` versus `RealOutputs`.** `ReplayOutputs` is the output of the
code under test. `RealOutputs` is what the robot did that day with whatever
build was deployed, and the robot also scored and fused the observations from
the loops that were not logged (section 21), so even an unchanged filter
replays differently from the recording. Compare replays with each other, not
with the recording.

**`Vision/Summary/ObservationScore` count and distribution.** Note that a
record is not one fused observation. `Vision.periodic` records the key once
per fused observation, but AdvantageKit keeps one value per key per loop, so
only the last fused observation of each 100 ms batch reaches the log, and the
writer does not record a value equal to the previous record. In the E8
baseline the filter sent 2,560 fused observations in 1,809 batches, and the
log holds 1,445 records. Fewer records usually means a stricter filter, but
the count is a sample, not a total. The same applies to `FusedCameraCount`,
which is recorded only when the count changes.

| Selected rows from the report | Baseline | Distance weight 1.5 |
|---|---|---|
| `ObservationScore` records | 1,445 | 1,316 |
| `FusedCameraCount` records | 725 | 696 |
| Of those, records with a count above 1 | 442 | 426 |
| Mean of the logged scores | 0.784 | 0.779 |
| Records from 0.6 to 0.7 | 255 | 205 |
| Records from 0.7 to 0.8 | 812 | 842 |
| Records from 0.9 up to but not including 1.0 | 109 | 8 |
| Records equal to 1.0 (in no bucket) | 269 | 261 |

The last two rows need care. The script's buckets are half-open, so a score
of exactly 1.0, which two agreeing cameras produce whenever the better of
them scored 0.714 or more (0.714 x 1.4 = 1.0), falls in no bucket; those
fully boosted fusions barely changed. The 0.9 to 1.0 bucket holds fusions
whose best member scored between 0.65 and 0.714, which in this match were
single-tag observations about 4 to 5 m from the tag. At 4.5 m, a level
observation with ambiguity 0.01 scores 0.670 under the baseline and 0.612
with the distance weight at 1.5, so the heavier weight rejects it before
fusion. The bucket emptied because marginal observations were dropped, not
because good ones scored lower. About a third of the observations in E8 were
5 to 8 m from the tag, where the distance test scores 0.27 down to 0.02;
those were at or below the threshold under the baseline already. Replay
reveals this kind of consequence; the unit tests do not.

**The `RobotPosesAccepted` and `RobotPosesRejected` rows are record counts,
not pose counts.** Each record is one logging cycle's array of poses, and the
replay writer records an array only when it differs from the previous one, so
the E8 baseline's 4,293 accepted records are 3,454 non-empty arrays plus 839
empty ones, holding 4,549 poses. The recording shows 8,357 records for the
same poses. Neither count is a useful metric. The script does not count the
poses in `ReplayOutputs` (section 23); until it does, counting accepted poses
for a replay means opening `ReplayOutputs/Vision/Summary/RobotPosesAccepted`
in AdvantageScope and inspecting each cycle's array.

**The false-positive section's trajectory analysis reads the recording, not
the replay.** Its pose jumps and impossible velocities (part 3 of that
section) are computed from `RealOutputs`, so they are identical for every
replay of the same log; parts 1 and 2 match `ObservationScore` timestamps
between `RealOutputs` and `ReplayOutputs`. Extending the script to analyze
`ReplayOutputs` is listed in section 23. Until then, jump counting for a
replay is done in AdvantageScope: put
`ReplayOutputs/Vision/Summary/RobotPosesAccepted` on the field view and step
through the match.

### Assessment without ground truth

The robot's true position was not measured, so no replay can be shown to be
more accurate than another in absolute terms; the available evidence is
indirect.

- **Jumps and reflections.** Count them in each replay by the method in
  section 5: consecutive accepted poses implying more than 4.1 m/s cannot
  both be correct, and poses with approximately the correct x and a y
  reflected across the field are wrong PnP solutions.
- **Smoothness.** In AdvantageScope, the drive's estimated pose should move
  continuously.
- **Camera agreement.** A higher proportion of multi-camera fusions means the
  accepted set is more self-consistent.
- **What was lost.** Fewer accepted observations is not automatically an
  improvement. Look at when the rejections occur. Rejections spread evenly
  through a match indicate a stricter filter; rejections concentrated in one
  interval indicate the loss of a camera or a region of the field.

## 21. Limitations

- **No ground truth.** Everything in section 20 is a proxy.
- **Per-test scores are not logged.** Replay shows that an observation was
  rejected but not which test rejected it.
- **Inputs are logged every other loop.** `LOGGING_DIVISOR` is 2, so the log
  holds half the frames the robot processed. The replay is faithful to the
  log, not to the robot.
- **The recording is from an older build.** `RealOutputs` is a historical
  record, not a control. The control is a baseline replay of the current
  code.
- **One match is one sample.** There are 14 VACHE match logs. A change that
  helps one match may not help another; the wrong observations in section 5
  arrived in clusters that a different match might not contain.
- **Replay does not measure timing.** It runs as fast as possible on a
  development machine. CPU load and loop overruns on the roboRIO are a
  separate question.
- **Replay is open-loop.** The drive in replay has no hardware, and its pose
  estimate is built from the recorded odometry plus the replayed filter's
  output. That is correct for evaluating the filter. A change that would have
  altered how the robot drove, and therefore what the cameras saw, cannot be
  evaluated this way.

---

# Part 7. Reference

## 22. Common questions

**Why not take the minimum score?** The minimum discards six of the seven
scores and every weight. Under the current curves it is unusable: pitch never
scores above 0.522, so the minimum could never reach the 0.65 threshold and
every observation would be rejected. It becomes a reasonable alternative if
the tilt curves are made steeper.

**Is a weight of 0.5 half as important?** It has half the share, not half the
effect. The effective share is the weight divided by the total weight, and the
total changes whenever any weight changes.

**Why does a perfect observation score only 0.78?** Because pitch and roll
score 0.522 and height scores 0.562 at zero error. Steeper curves would raise
the ceiling, and since a level robot keeps all three readings below the
midpoints, they would raise the score of nearly every real observation with
it; the threshold would then need to be re-derived in the same change.

**Is a score of 0.7 a 70% probability that the observation is correct?** No.
The tests are correlated (pitch, roll, height, and ambiguity all come from the
same solve) and none of them is calibrated as a probability. The score is a
ranking, a veto, and the divisor for the standard deviation.

**Two agreeing cameras receive a 1.4x boost. Does that break the 0-to-1
scale?** The boost is applied after the threshold, only when cameras agree,
and is capped at 1.0. It never admits a rejected observation.

**Why is there no record of which test rejected an observation?** Because
per-test scores are not logged; only the fused score of accepted observations
is. Adding that logging is listed in section 23.

**When is the arithmetic mean the right choice?** When the quantities being
averaged are additive, such as points or seconds. When a second failure makes
an observation twice as bad rather than two units worse, the product is the
appropriate combination.

## 23. Open work

Each item is a self-contained change.

- Log every observation's per-test scores, per camera. Without them, the
  cause of a rejection can only be recovered by replay.
- Log the standard deviations sent to the estimator. The score's second role
  does not appear in the logs.
- Add runtime tuning with AdvantageKit's `LoggedNetworkNumber` for each weight
  and for `MIN_SCORE`, so the threshold can be adjusted from a dashboard on the
  practice field.
- Add golden-value tests pinning the typical single-tag score at 0.712 and the
  three-tag score at 0.775, and asserting that both exceed `MIN_SCORE`, so
  that a weight change or a threshold above the single-tag ceiling (section
  16) fails a test.
- Fix the stale "Rejects ambiguous PnP solution with wrong yaw" test so the
  baseline is 86 tests, 0 failed.
- Treat a NaN score as rejected in `Vision.java`. With every weight zero
  (section 7, rule 2) the score is NaN, which passes both comparisons and
  reaches the estimator.
- Fix `velocityConsistency` by writing `lastAcceptedPose` and
  `lastAcceptedTimestamp` in `Vision.java` after an observation is accepted
  (section 17).
- Re-derive the pitch, roll, and height curve steepness from replays, and
  re-derive `MIN_SCORE` in the same change. These three tests carry 2.1 of the
  5.4 total weight and cap every unfused score at 0.783.
- Extend `scripts/compare_vision_logs.py` to run its trajectory analysis on
  `ReplayOutputs` and to compare two replay logs directly.
- Re-enable `yawConsistency` with the four guards listed in section 17.
- Decide whether fusion should require a majority of cameras, as the original
  design did, or continue merging any two. Replay both on the VACHE logs.

## 24. Glossary

- **AdvantageKit.** The logging and replay framework the robot code is built
  on. It records the code's inputs and can replay them through modified code.
- **AdvantageScope.** The desktop application for viewing log files: plots,
  field views, 3D views.
- **Ambiguity.** PhotonVision's 0-to-1 measure of how close the second-best
  PnP solution was to the best. A high value means the image of the tag fits
  two camera poses about equally well.
- **AprilTag.** The square fiducial markers on the field, each with a known ID
  and position.
- **Arithmetic mean.** The ordinary average: sum, then divide.
- **Batch.** The observations scored since the last fusion pass, held in a
  buffer in `Vision.java` and processed together every fifth loop.
- **Coprocessor.** The separate small computer on the robot that runs
  PhotonVision; the roboRIO reads its results over NetworkTables.
- **Extrinsics.** A camera's position and orientation on the robot.
- **Fusion.** Merging observations from cameras that agree into one averaged
  observation.
- **Geometric mean.** Multiply, then take the root. Never larger than the
  arithmetic mean of the same numbers.
- **Intrinsics.** A camera's internal lens model, determined by calibration.
- **Jump.** Two accepted poses far apart within a short interval. The
  signature of a wrong observation.
- **NetworkTables.** The protocol the roboRIO, the coprocessors, and the
  dashboard use to share values over the robot network.
- **Observation.** One camera reading: a timestamped pose with its ambiguity,
  tag count, and average tag distance.
- **Odometry.** Dead reckoning from wheel rotations, integrated along the
  gyro heading. Section 1 counts the wheels and the gyro as separate sources.
- **PhotonVision.** The AprilTag detection and pose-solving software that
  runs on the coprocessors.
- **PnP (Perspective-n-Point).** The computation that recovers a camera pose
  from the image positions of a tag's corners.
- **Pose.** Position and orientation. The estimator's pose is 2D: x, y, and
  heading. An observation's pose is 3D: x, y, z, roll, pitch, yaw; the filter
  uses z, roll, and pitch as error signals (section 2).
- **Pose estimator.** WPILib's filter that combines odometry and vision,
  weighting each by its standard deviation.
- **RealOutputs, ReplayOutputs.** In a replay log, the recorded behavior and
  the replayed behavior, respectively.
- **Reprojection error.** After a solve, the distance in pixels between the
  observed tag corners and where the solution predicts them. Lower is better.
- **Sigmoid.** An S-shaped function from 0 to 1, used here to convert a
  measurement into a smooth score.
- **Standard deviation.** Here, the expected error magnitude sent with each
  pose. A small value gives the measurement more weight in the estimator.
- **Test.** One of the nine scoring functions in the `Test` enum of
  `VisionFilter.java`, seven of them enabled. Distinct from a unit test in
  `VisionFilterTest.java`.
- **Threshold.** `MIN_SCORE`, 0.65. An observation is accepted when its
  combined score is at least `MIN_SCORE` (section 4 gives the exact
  comparisons).
- **VACHE.** The March 2026 competition event. Its 14 match logs
  (qualification matches 4 to 58 and elimination matches 4 and 8) are the
  data behind sections 6, 17, and 19 to 21.
- **wpilog.** The binary log format written by AdvantageKit.

## Related documents

- `VACHE_AUTO_POSE_ANALYSIS.md`: the effect of a wrong starting pose on
  autonomous performance at VACHE, match by match.
- `scripts/compare_vision_logs.py`: the comparison script used in Part 6.
