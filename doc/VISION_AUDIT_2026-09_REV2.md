# Vision system audit September 2026

**Proposed rewrite for review**

The robot uses cameras to estimate where it is on the field. This audit asks
whether the code makes good use of those estimates, whether the logs explain
its decisions, and how students could safely experiment with its settings.

The main finding is that the filter's seven checks do not contribute as their
names suggest. Distance from a tag and the uncertainty of the camera's pose
solution account for most decisions. The tilt and height checks have little
influence, and the field boundary check can reject useful readings near a
wall. Accepted readings receive nearly the same uncertainty value, so the
position estimator can give a poor reading substantial influence.

**The recommended order is to improve the logs and camera data handling,
make experiments reproducible, and then change the scoring and estimator
settings.** Otherwise, an apparent improvement may come from missing or
repeated data rather than a better filter. The audit estimated the initial
logging work at about one mentor-day.

## Scope and later changes

The audit was performed on September 17, 2026, against commit `f3ca52c`.
Unless a passage explicitly says otherwise, findings, numerical results,
source line numbers, and references to the then-current `main` describe that
version. The proposed classes and tools later in this document are designs,
not features already available in the repository.

The audit used the source code and the library sources for wpimath 2026.2.1,
AdvantageKit 26.0.2, and PhotonLib 2026.3.4. Its detailed match analysis used
VACHE Q10, Q27, Q54, E4, and E8, and VAALE E4 and E6; broader counts covered
14 VACHE match logs. VACHE and VAALE identify events; Q identifies a
qualification match and E an elimination match. It also replayed E8 with additional diagnostic logging.
One reviewer wrote each finding, a second tried to disprove it, and a third
assessed its practical effect. Of 78 candidate findings, 76 remained after
review and two were rejected. Severity ratings include those corrections.
Calculated values came from the code's constants; match results identify the
log they came from. These are the September audit's results, not a new
analysis of those logs.

**Branch update checked October 8, 2026, at `13cc342`:** `vision-tuning` now
sets `LOGGING_DIVISOR = 1`, enables individual-camera and summary pose
logging, and skips starting `VisionThread` in replay. Those changes address
parts of the logging recommendations. The code still uses the two summary
score values and does not record the proposed row of individual check scores
for every observation. This note describes source changes; it does not mark
the broader recommendations as tested or complete.

## How to use this document

Start with [How vision data reaches the robot pose](#how-vision-data-reaches-the-robot-pose)
if you are new to the code. [What the audit found](#what-the-audit-found)
explains the evidence. [Recommended order of work](#recommended-order-of-work)
is the implementation checklist, and [Making settings easier to experiment with](#making-settings-easier-to-experiment-with)
describes the proposed design.

The reference sections preserve the details needed for implementation:

- [Appendix A](#appendix-a-detailed-findings) contains all 76 findings, their
  original identifiers, source locations, severity ratings, and proposed fixes.
- [Appendix B](#appendix-b-findings-that-were-disproved) explains the two
  rejected findings.
- [Appendix C](#appendix-c-behavior-the-audit-checked-and-found-correct) lists
  the behavior that was checked and found correct.
- [Appendix D](#appendix-d-original-design-sketches) retains the original
  Java sketches, with their limitations called out.

The [vision guide](VISION_GUIDE.md) provides longer
explanations of calibration, scoring, and replay. You do not need to read it
before this audit.

## How vision data reaches the robot pose

A **pose** is a position and orientation. For driving, it usually means the
robot's x and y coordinates on the field and its heading. Camera calculations
also estimate height, pitch, and roll. Pitch is the forward or backward tilt;
roll is the sideways tilt; yaw is the heading around the vertical axis.

An **AprilTag** is a printed marker whose position on the field is known.
PhotonVision finds tags in camera images and calculates possible camera
poses. The robot code converts these to robot poses using each camera's
configured mounting position and orientation. One such reading, with its
capture time and related information, is an **observation**.

The stages in the audited code are:

1. **Read the cameras.** `VisionIOPhotonVision` converts PhotonVision results
   to `PoseObservation` records. `VisionThread` polls the cameras on a
   background thread and hands snapshots to the main robot loop.
2. **Score each observation.** `Vision.periodic` calls `VisionFilter`.
   Seven enabled checks examine ambiguity, pitch, roll, height, field
   boundaries, tag count, and tag distance. Two additional checks, for
   velocity and yaw consistency, are disabled.
3. **Collect a batch and reject low scores.** The main loop runs every
   20 ms. Every fifth loop, the code processes the collected observations,
   giving a batch interval of 100 ms. `MIN_SCORE` is 0.65.
4. **Combine agreeing cameras.** The filter merges observations whose
   positions and capture times are close. This is called **fusion**.
5. **Tell the position estimator how uncertain each measurement is.**
   `Vision.periodic` calculates linear and angular standard deviations and
   passes the measurements to `Drive`. A smaller standard deviation tells
   the estimator to put more trust in the camera measurement.
6. **Update the robot pose.** WPILib's `SwerveDrivePoseEstimator` combines
   vision with wheel and gyro measurements. Estimating movement from the
   wheels and gyro is called **odometry**.

Useful places to start reading the code are:

| File | Responsibility |
|---|---|
| `src/main/java/frc/robot/subsystems/vision/VisionIO.java` | Defines the observation and camera-input records. |
| `src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVision.java` | Converts camera results into observations. |
| `src/main/java/frc/robot/util/VisionThread.java` | Polls the cameras and hands data to the main loop. |
| `src/main/java/frc/robot/subsystems/vision/VisionFilter.java` | Scores observations and combines agreeing readings. |
| `src/main/java/frc/robot/subsystems/vision/Vision.java` | Runs scoring, batching, logging, and delivery to `Drive`. |
| `src/main/java/frc/robot/subsystems/vision/VisionConstants.java` | Defines camera transforms, thresholds, and other settings. |
| `src/main/java/frc/robot/subsystems/drive/Drive.java` | Owns the pose estimator and handles vision measurements. |
| `src/test/java/frc/robot/subsystems/vision/VisionFilterTest.java` | Tests filter behavior. |

### Terms used in the findings

| Term | Meaning here |
|---|---|
| Check or criterion | One rule used to judge an observation. The current Java enum calls these `Test`; this document uses “unit test” for automated software tests. |
| Gate | A rule that accepts or rejects outright, rather than giving a gradually changing score. |
| Ambiguity | PhotonVision's measure of how difficult it is to choose between possible pose solutions. A low reported value normally favors one solution, but special values need separate handling. |
| PnP | “Perspective-n-Point,” the calculation that estimates a camera pose from known points in an image. A single flat tag can produce two plausible solutions, sometimes called mirror solutions. |
| Single-tag and multi-tag | Whether one camera's pose calculation uses one tag or several. Multi-tag is different from combining readings from several cameras. |
| Sigmoid | A smooth S-shaped scoring curve. Its midpoint is where the score is 0.5; its steepness controls how quickly the score changes. |
| Score ceiling | The highest total score possible with the selected checks and settings. |
| Standard deviation | The uncertainty supplied to the pose estimator: meters for position and radians for heading. It is not automatically a measured error. |
| Gain | The fraction of the difference between the camera measurement and the existing estimate that the estimator applies as a correction. |
| Innovation or residual | The difference between a measurement and a reference estimate. An innovation gate rejects a measurement when that difference is too large. |
| Replay | Running robot code on recorded AdvantageKit inputs. It can run much more than the vision filter. |
| NetworkTables or NT | The system used to exchange values between the robot, camera computers, and dashboards. |
| FMS | The Field Management System used at an event. Its connection is part of the proposed protection against accidental tuning during a match. |
| NaN | “Not a Number,” an invalid floating-point result. Ordinary comparisons with it return false. |

## What the audit found

### 1 Distance and ambiguity dominate the score

Each enabled check returns a value from 0 to 1. The code combines them using
a **weighted geometric mean**: raise each score to its weight, multiply the
results, then take a root based on the sum of the weights.

```text
total score = (product of each score raised to its weight)
              raised to (1 / sum of weights)
```

With a positive weight, a score of zero rejects the observation regardless
of the other checks. A score of one leaves the product unchanged but still
adds its weight to the root. This second effect matters here.

The five gradually changing checks use
`normalizedSigmoid(x, midpoint, steepness)`. Steepness is measured in the
inverse of the input unit: per radian for angles and per meter for distances.
Using the same numerical steepness for these different quantities does not
make them equally selective.

For pitch and roll, steepness is 1.0 per radian and the midpoint is 0.087 rad,
or 5 degrees. The score takes about 2 radians, or 115 degrees, to fall from
0.73 to 0.27. A level robot scores 0.522; one pitched 45 degrees scores 0.332.
The checks do respond, but their curves are much too broad for the intended
5-degree tolerance. Together, pitch, roll, and height change a typical total
score by only hundredths. In E8, accepted and rejected observations had
similar pitch distributions, with medians of 1.5 and 1.8 degrees.

Commit `decba74` reduced the angle midpoints from 30 to 5 degrees and the
height midpoint from 0.75 to 0.25 m without changing steepness. That produced
much less tightening than the new tolerance values suggest.

The other gradually changing checks respond more usefully:

| Check | Example scores from the audited settings |
|---|---|
| Distance | 0.98 at 0 m, 0.5 at 4 m, and 0.02 at 8 m. |
| Ambiguity | 0.646 at zero ambiguity and 0.269 at ambiguity 0.4. |

The two yes-or-no checks need separate treatment. `moreThanZeroTags` always
passes for the observations the camera code emits: it only emits a reading
when it has a target, and the single-tag branch sets `tagCount = 1`.
`withinBoundaries` can reject observations, including useful ones near walls
as described below. Among observations that pass it, its score is always 1.0.
Neither provides a gradual quality distinction between those observations.

These two checks contribute 2.0 of the total weight of 5.4. For a typical
observation, the geometric mean of the five gradual scores is 0.583, but
including the two passing gates raises the total to 0.712. A threshold of
0.65 on the combined score is equivalent to about 0.505 on the gradual
checks alone. Doubling one gate's weight has the same acceptance effect as
lowering the original threshold to about 0.600.

If the other five scores are held at their usual nearly constant values,
the acceptance condition reduces approximately to:

```text
ambiguity score ^ 0.8 × distance score ^ 0.5 >= 0.363
```

That gives the following practical limits:

- Even an unambiguous single-tag observation is rejected beyond about 5.0 m,
  or about 4.4 m with realistic tilt.
- At 4 m, ambiguity must be below about 0.22; at 1 m, below about 0.37.
- A multi-tag observation would be rejected beyond about 5.9 m, even though
  multi-tag estimates are the more reliable kind at range.

The audit also asked: **If just this check had given a perfect score, what
percentage of all observations would have changed from rejected to accepted?**
This is what the original audit called “would-flip-if-perfect.” It measures
sensitivity to a check, not the fraction uniquely caused by that check; the
columns should not be added together.

| VACHE match | Distance made perfect | Ambiguity made perfect |
|---|---:|---:|
| Q27 | 32% | 9% |
| E8 | 6% | 4% |
| E4 | 12% | 4% |
| Q54 | 10% | 14% |

Distance has the larger effect in three of these four matches; ambiguity has
the larger effect in Q54. Together they explain essentially all rejections
in the audit's scoring analysis, subject to the separate boundary failures.

### 2 Accepted scores occupy a narrow range

With pitch, roll, and height scores around 0.52 to 0.56 and the ambiguity
score no higher than 0.646 for a single tag, the best possible total is
0.734 for a single-tag observation or 0.783 for a multi-tag observation.
The acceptance threshold, 0.65, is only 0.084 below the single-tag ceiling.

In E8, accepted single-camera observations had a maximum score of 0.7317 and
a median of 0.711. Applying the 1.4× fusion boost puts accepted combined
readings in the range 0.91 to 1.0. The scores therefore form two groups:
a narrow band for single-camera readings and another near 1.0 for fused
readings.

Comments describe a typical single-tag score of about 0.75 and multi-tag
scores above 0.9. Those values are unreachable before fusion under these
settings. Raising `MIN_SCORE` to 0.735 would reject every single-tag
observation, and the existing unit tests would not detect that loss.

All 178,000 observations in the 14 VACHE match logs used one tag. No multi-tag
observation was found in that data. The reason is unknown; the repository
does not include the coprocessor settings needed to determine it.

### 3 The uncertainty value changes little with observation quality

The uncertainty sent to the estimator is calculated as:

```text
one camera:       standard deviation = baseline × 3 / score
combined cameras: standard deviation = baseline / score
```

The narrow accepted score range makes the linear standard deviation for a
single-camera observation only 0.082 to 0.092 m, a spread of about 13%.
The mapping has no direct distance term: measurements from a tag at 1 m and
at 5 m receive values within that same narrow band, even though single-tag
PnP position error grows roughly with distance squared.

The largest distinction is the roughly 4.2× change in standard deviation
between a single-camera reading and a boosted fused reading. Three lines in
`Vision.periodic` implement the mapping. None of the 86 unit tests covers
it, its outputs are not logged, and `MAX_STD_DEV` is defined but unused.

### 4 One camera reading can move the estimated pose substantially

`Drive` constructs `SwerveDrivePoseEstimator` with WPILib's default state
standard deviations: 0.1 m for x and y and 0.1 rad for heading. The gain per
axis is `q / (q + sqrt(q * r))`, where `q` and `r` are the state and vision
variances, respectively. With the vision uncertainties above, the audit
calculated these approximate corrections:

| Measurement | Fraction of position difference applied | Fraction of heading difference applied |
|---|---:|---:|
| Accepted single-camera reading | 0.52 | 0.27 |
| Fused reading | 0.83 | 0.63 |

There is no additional check that rejects a measurement for disagreeing too
much with the existing estimate. The result follows raw single-tag PnP
readings quickly, often within one or two updates.

Q27 provides a concrete example. The robot was stationary during the
disabled interval between autonomous and driver control. The estimate was
(7.34 m, 6.39 m, 123 degrees). A rear camera reported
(7.08 m, 7.24 m, 110 degrees), with ambiguity exactly 0.0, height -0.23 m,
and a tag 4.5 m away. Its score was 0.660, so it passed. The estimate moved
(-0.14 m, +0.44 m, -3.6 degrees), about 46 cm in position. The gain calculation
predicted (-0.14 m, +0.45 m, -3.5 degrees); using the unrounded values, the
audit reported agreement within 3 mm.

Each camera can supply two or three frames in a 100 ms batch, and each
single-camera measurement is applied separately. Their combined position
correction is about 0.77 to 0.89 of the difference. Three such updates can
therefore exceed the roughly 0.83 correction from one fused measurement;
two approach it. Agreeing cameras are collapsed into that one measurement.

Heading deserves particular attention. The alternate solutions for a flat
tag can differ substantially in yaw, yet each accepted observation replaces
27% to 63% of the heading difference even when the gyro is working well.

### 5 Useful wall readings are rejected and startup has no consensus step

The allowed rectangle is the field inset by half the robot width, 0.468 m,
and is built once when the class initializes. A robot against a wall has its
center at this limit. A few centimeters of camera error toward the wall can
therefore make the boundary check reject an otherwise useful reading.

In E4, from 148 to 168 s, the estimated position was 3.6 to 5.4 m outside the
field and all 414 observations were rejected. Of those, 205 had y between
0.40 and 0.45 m, absolute height below 0.07 m, tilt below 3 degrees,
ambiguity below 0.03, and a tag 1.3 to 1.7 m away. Without the boundary
rejection they would have scored 0.667 to 0.674. These were among the best
readings in the match, but this check prevented them from correcting the
pose for 20 s.

A different problem occurred before E4 and E8. The robot sat at its starting
position for about 110 s with tags visible. All 1,722 observations in E4 and
all 1,895 in E8 were rejected, leaving the estimate at the origin when auto
started. Those rejections were appropriate: the tags were 5.8 to 7.3 m away,
ambiguity was 0.5 to 0.6, and the cameras disagreed by 6 to 8 m. Most
readings represented the wrong mirror solution.

The missing capability is to use repeated agreement while the robot is
stationary. Hundreds of readings from several cameras could be grouped to
look for a consistent starting pose. Simply lowering the threshold would
admit the wrong readings. By the audited version of `main`, autonomous
routines reset odometry at the start, limiting this issue to the disabled
period, matches without auto, and the first seconds of auto. The older
`setPose` behavior at VACHE let it affect an entire match.

### 6 Combining cameras does not check heading or account for motion

The filter combines readings when their positions are within 0.15 m and
their capture times are within 0.15 s. It does not require their headings
to agree. It averages heading using a score-weighted circular mean, which
handles angle wraparound correctly but cannot decide whether the headings
should have been combined in the first place.

For example, equally weighted cameras can agree on position but differ by
90 degrees in heading. Their mean heading is 45 degrees between them, even
though neither camera reported it. The combined score is at least 0.91;
an angular standard deviation of about 0.066 rad gives a heading gain of
about 0.60. None of the 18 fusion unit tests uses a nonzero heading.

The score boost is also too coarse. It multiplies the best member's score by
1.4, so readings scoring 0.72 and 0.65 produce a combined score capped at
1.0. That loses distinctions between groups of readings.

Timing causes two more problems. Batches end every fifth loop, so roughly
one fifth of readings with an agreeing partner inside the time window fall
on opposite sides of a batch boundary. They are processed separately, with
the single-camera 3× standard-deviation multiplier. Also, the code directly
compares field positions captured as much as 150 ms apart. At 2 m/s, the
robot moves 0.15 m in only 75 ms, effectively halving the useful time window
and reducing fusion while driving.

Finally, a camera whose mounting position or orientation has shifted can
disagree with every other camera and still have its readings accepted
individually at the normal single-camera trust. No running measure tracks
how consistently each camera disagrees with the others.

### 7 The logs omit the information needed to explain decisions

The filter calculates individual check scores in an `EnumMap` and then
discards them. The logs cannot show which checks lowered a particular
observation's score.

`Vision/Summary/ObservationScore` and `FusedCameraCount` are written once for
each fused measurement inside a loop. AdvantageKit keeps only one value for
a key in each robot cycle and writes a new value only when it changes. If a
batch sends four measurements, the summary retains only the last one's
score. Counting changes to those values is not counting observations;
earlier analyses were effectively measuring batch summaries.

`RobotPosesRejected` contains only `Pose3d` values. Without camera index,
capture time, or score, a displayed rejected pose cannot reliably be matched
to its input record. The summary score is written only for accepted
measurements, so its histogram necessarily omits all scores below the
threshold. It cannot support the guide's suggested analysis of rejected
scores.

At the audited commit, `LOGGING_DIVISOR = 2` logged camera inputs on even
loops while the filter scored every loop. The log therefore omitted half
the input cycles the robot processed. Replaying that code saw only half the
observations; replaying E4 with divisor 1 against a divisor-2 log instead
scored each recorded observation twice. This is why changing the replay's
logging interval alone cannot recover missing historical data.

No parameter values were recorded. Both VACHE builds included uncommitted
changes, so the exact settings that produced those match results cannot be
recovered from their commit IDs alone.

Passing the filter also does not prove the estimator applied the reading.
The estimator silently ignores measurements older than 1.5 s and removes
later vision updates when an earlier timestamp arrives. The code records
no feedback about either case.

### 8 Camera data can be repeated lost or accepted with invalid times

Every 20 ms, a WPILib `Notifier` runs the background camera poll and replaces
one shared snapshot. The main loop also runs every 20 ms and reads whichever
snapshot is present. There is no generation number identifying whether it
has already read that snapshot.

Two main-loop reads between camera polls can process the same frames twice.
In Q27, where inputs were logged every loop, 3.6% of front-right-camera
frames were duplicated. Applying the same correction twice increases the
position correction from about 0.52 to 0.77. Conversely, two camera polls
between main-loop reads can overwrite a snapshot before it is processed.
Those frames are lost because `getAllUnreadResults` has already consumed
them.

An exception from PhotonLib inside the shared `Notifier` can stop polling
all four cameras. The last snapshot remains in place and can still report
that its cameras are connected.

The code also does not compare capture timestamps with the roboRIO's FPGA
clock. After a NetworkTables reconnect, the coprocessor's time-sync client
starts with an offset of zero and can publish its own clock for about a
second. In Q10, the coprocessor serving the front-left and rear-left cameras
was 38 s ahead. The audit found 167 observations stamped 38 to 52 s into
the future within a second of each reconnect. The estimator uses “now” for
the pose lookup but stores the update under the future timestamp. The next
correctly stamped reading deletes that update, making the estimate switch
between cameras at about 10 Hz during the problem.

The camera conversion code drops useful information too. Its single-tag
branch takes `result.targets.get(0)` from an unsorted AprilTag list, discards
the other targets, and logs only the selected tag's ID. It keeps only one of
the two possible PnP solutions. Comparing both solutions with the heading at
capture time could help choose the correct one.

With ambiguity above about 0.38, no single-tag observation can pass the
existing score threshold. This removes 29% to 42% of observations per match.
However, many such readings are also far away; selecting the alternate pose
alone would not make all of them suitable. `PoseObservation` lacks tag ID,
alternate pose, target area, latency, and sequence number, preventing a full
comparison of alternative policies with the existing logs.

Finally, ambiguity has no validity check. PhotonVision uses -1 to indicate
an invalid ambiguity value, but the scoring curve gives it about 0.99.
Exactly 0.0 receives the best possible ordinary score; in Q27 it accompanied
the worst accepted observation.

### 9 Replay and unit tests can give misleading reassurance

A full replay runs the whole robot program with the current build. Its
`ReplayOutputs/Drive/Pose` includes effects from changes to autonomous
routines, odometry, and other code as well as vision. Replaying E8 through
the then-current `main` produced a 13 m difference at auto start because of
a `setPose` change, even though the individual filter decisions were
byte-for-byte identical.

Replay requires editing `Constants.java`. Logs from before June also
require a `Drive.java` edit: a renamed input key otherwise silently receives
a default value, and the program crashes about 130 s later in `Launcher.aim`.
There is no automated check that unchanged filter settings reproduce the
recorded decisions, although the audit verified that behavior for E8.

The 86 filter unit tests mainly check ordering, such as whether a good
observation scores higher than a bad one. They do not require specific curve
shapes, reference scores, score ceilings, or a useful margin above the
acceptance threshold. Flat curves can still satisfy those comparisons.

The test “Rejects ambiguous PnP solution with wrong yaw” uses the default
checks after `yawConsistency` was removed from them. The audit found it had
failed on every build that ran it since March, but `ignoreFailures = true`
turns the failure into a warning and still prints `BUILD SUCCESSFUL`.

Other gaps include:

- Tests use a 16.54 by 8.21 m field from 2024, while the code uses the 2026
  dimensions, 16.513 by 8.043 m. No boundary test checks the far edge.
- Thirteen tests exercise the disabled yaw or velocity checks without
  making that distinction clear.
- “Velocity check uses previous pose” rejects its supposed impossible move
  because the pose is outside the field, so it does not establish that the
  velocity check works.
- No test covers `Vision.periodic` as a whole: buffering, the acceptance
  decision, sorting, uncertainty calculation, and delivery into the
  first-pose initialization path are untested together.
- `Vision.enabledTests` and `VisionFilter.DEFAULT_ENABLED_TESTS` refer to the
  same mutable `EnumSet`. Removing a check from one changes the other and
  can change the tests' expectations too.

### 10 Disabled features and unusual inputs have additional problems

`velocityConsistency` is disabled. The per-camera history arrays it reads
are allocated but never updated. Their updates existed on the March branch
and were lost when fusion was merged. Enabling this check now would multiply
typical scores by about 0.930 without detecting unrealistic movement.

`yawConsistency` is also disabled. Its reference is the estimator's heading
at scoring time, 50 to 170 ms after capture, and vision itself changes that
heading. The reference is therefore both late and partly dependent on the
measurements being checked. At 180 degrees per second, a 100 ms delay alone
creates 18 degrees of apparent disagreement. Enabling it as written could
reject good readings while the robot spins. Both checks need repairs before
being offered in a tuning interface.

Other cases need explicit handling:

- The choice between a custom and official AprilTag layout checks FMS only
  at first load and then caches the result. A layout selected before FMS
  connects can remain active during the match.
- The first vision measurement delivered while disabled can directly set
  the estimator's initial pose without an additional check of its quality,
  distance, or number of cameras. That initialization remains armed through
  auto and could fire in the disabled gap before teleop on the audited `main`.
- A batch larger than 32 observations, such as a reconnect backlog, silently
  skips fusion.
- A NaN score passes through the current combination of comparisons and can
  reach the estimator with NaN uncertainty values.

## Recommended order of work

The order below preserves the audit's priorities. Severity describes the
consequence of a problem; effort is the audit's estimate for a mentor who
knows this code. The ratings apply to each group of work. Appendix A gives
individual ratings, which sometimes differ from the group's rating.
References such as [A35](#a35-individual-scores-are-not-logged) point to the detailed findings there.

The early steps make later experiments trustworthy. In particular, changing
curve shapes should wait until the offline experiment tool exists, and
changing the uncertainty supplied to the estimator needs robot testing.

| Priority | Work and intended result | Severity | Estimated effort | Details |
|---|---|---|---|---|
| 1 | Record every observation's check scores, total, acceptance decision, camera, time, and strongest reason for a low score. Record each fused measurement and its uncertainty. Replace the two summary score values. | High | Small | [A35](#a35-individual-scores-are-not-logged), [A36](#a36-rejected-poses-cannot-be-traced-back-to-their-scoring-decisions) |
| 2 | Log camera inputs every loop and measure the cost. Do not start the camera thread in replay. | High | Trivial | [A37](#a37-logging-fewer-input-cycles-changes-replay-behavior), [A38](#a38-replay-can-overwrite-recorded-inputs-with-an-empty-camera-snapshot) |
| 3 | Log every effective filter setting as an input, plus the enabled checks and configuration hash. | Medium | Trivial | [A39](#a39-logs-do-not-identify-the-filter-settings-that-ran) |
| 4 | Prevent repeated or overwritten camera snapshots; identify snapshots, count losses, catch exceptions per camera, and expire stale data. | Medium | Small | [A21](#a21-snapshot-replacement-can-repeat-or-lose-frames), [A22](#a22-one-camera-exception-can-stop-all-camera-polling) |
| 5 | Reject and count readings more than 20 ms in the future or 0.5 s in the past. Log latency and clock-sync health. | Medium | Small | [A24](#a24-camera-timestamps-and-clock-synchronization-are-not-validated), [A25](#a25-a-negative-time-interval-receives-a-perfect-velocity-score) |
| 6 | Use one acceptance decision, `score >= minScore`, for processing and logging. Reject non-finite poses and uncertainty values. | Medium | Small | [A08](#a08-invalid-numerical-scores-can-reach-the-estimator) |
| 7 | Give the curves meaningful shapes, then choose a threshold from recorded data and test the score ceilings. Target scores are 0.95 at zero error, 0.5 at tolerance, and 0.05 at twice tolerance. | High | Small code change, medium risk | [A01](#a01-tilt-and-height-scores-change-too-slowly), [A02](#a02-accepted-scores-have-little-room-above-the-threshold), [A03](#a03-the-threshold-acts-mostly-as-a-distance-limit), [A06](#a06-the-ambiguity-curve-has-a-low-maximum-and-allows-uncertain-readings) |
| 8 | Reduce reliance on single-tag heading while the gyro works. Derive position uncertainty from distance squared and the number of tags, cameras, and frames. Reject implausible corrections after initialization, and log the measurements sent. | High | Small, with robot time | [A10](#a10-accepted-measurements-can-cause-large-corrections), [A11](#a11-the-uncertainty-calculation-has-little-relation-to-distance), [A12](#a12-repeated-readings-from-one-camera-can-have-more-influence-than-fusion) |
| 9 | Allow measurement noise near field walls. Use the physical field plus a tolerance for outright rejection and a gradual penalty near the inset. Make the margin configurable. | High | Small | [A09](#a09-the-field-inset-rejects-useful-readings-near-walls) |
| 10 | Evaluate mandatory checks before calculating the geometric mean. Exclude their weights from the mean and reselect the threshold; 0.505 on the gradual checks is equivalent to the current 0.65. Keep this behavior change behind a flag until evaluated. | Medium | Small | [A04](#a04-mandatory-checks-distort-the-quality-score), [A05](#a05-changing-weights-also-changes-the-meaning-of-the-threshold) |
| 11 | Introduce one configuration record, remove duplicate definitions of values in different units, protect the default enabled set, and correct the comments. | High for tunability | Medium | [A56](#a56-there-is-no-shared-interface-for-filter-settings), [A57](#a57-some-settings-have-a-changeable-original-and-a-frozen-conversion), [A58](#a58-the-active-and-default-check-sets-are-the-same-mutable-object), [A61](#a61-comments-describe-old-settings-and-unreachable-scores) |
| 12 | Repair the unused velocity history and provide yaw at the observation's capture time from the raw gyro plus a fixed field offset. Keep these checks out of the tuning interface until repaired. | Medium | Small | [A59](#a59-the-velocity-check-never-receives-updated-history), [A60](#a60-the-yaw-reference-is-late-and-partly-determined-by-vision) |
| 13 | Repair the stale yaw and velocity tests; make vision test failures fail the check; test reference values, curve shapes, score margins, far field edges, and the full batch operation. | Medium | Small | [A63](#a63-a-stale-yaw-test-fails-without-failing-the-build), [A64](#a64-tests-do-not-require-useful-curve-shapes-or-score-margins), [A65](#a65-the-batch-processing-operation-has-no-direct-test), [A66](#a66-a-velocity-test-passes-because-another-check-rejects-its-pose), [A67](#a67-boundary-tests-use-the-wrong-season-field-size) |
| 14 | Calculate an actual acceptance rate over 2 s. Warn after 5 s with visible tags but no accepted observations, and report an uninitialized pose while disabled under FMS. | Medium | Small | [A53](#a53-other-code-cannot-tell-that-the-pose-is-uninitialized), [A54](#a54-rejecting-every-observation-does-not-raise-an-alert) |
| 15 | Preserve more camera information and compare alternate pose solutions. Investigate absent multi-tag results, select targets deliberately, and handle invalid or uncorroborated zero ambiguity. Version the observation format. | Medium | Medium | [A07](#a07-invalid-and-zero-ambiguity-values-need-special-handling), [A26](#a26-the-alternate-camera-pose-is-discarded), [A27](#a27-the-single-tag-branch-chooses-the-first-target-without-ranking-it), [A28](#a28-the-recorded-matches-contain-no-multi-tag-observations), [A29](#a29-the-observation-record-omits-useful-diagnostic-information), [A30](#a30-changing-the-observation-format-can-break-old-replay-logs) |
| 16 | Check heading agreement, replace the score boost, combine readings across batch boundaries, account for robot motion, and track persistent disagreement by camera. | Medium | Small to medium | [A13](#a13-a-camera-that-consistently-disagrees-is-still-trusted-individually), [A14](#a14-cameras-can-be-combined-despite-conflicting-headings), [A15](#a15-the-score-boost-quickly-reaches-its-maximum), [A16](#a16-batch-boundaries-separate-readings-that-should-be-compared), [A17](#a17-robot-motion-reduces-the-chance-of-combining-camera-readings) |
| 17 | Initialize only from agreement across cameras or frames; require an uninitialized pose; prevent initialization in the FMS gap between auto and teleop. Add agreement checking for distant tags while disabled. | Medium | Small | [A50](#a50-the-first-disabled-measurement-can-set-the-pose-without-extra-checks), [A51](#a51-pose-initialization-can-remain-armed-through-autonomous), [A52](#a52-repeated-distant-readings-are-not-used-to-establish-a-starting-pose) |
| 18 | Make replay selectable without source edits, support old input keys, check replay inputs and repeatability, label experiments, and build the offline comparison tool. | Medium | Medium | [A41](#a41-replay-needs-an-explicit-way-to-change-settings), [A43](#a43-missing-replay-inputs-cause-a-delayed-and-confusing-crash), [A44](#a44-no-test-proves-that-replay-reproduces-observation-decisions), [A45](#a45-full-replay-mixes-filter-changes-with-changes-elsewhere), [A46](#a46-replay-results-do-not-identify-or-preserve-the-experiment), [A49](#a49-comparing-settings-currently-requires-too-much-replay-setup) |
| 19 | Recheck the layout when FMS attaches, or explicitly select it at deployment. Make the AndyMark versus welded-field choice visible. | Low | Small | [A33](#a33-the-field-layout-choice-can-survive-a-later-fms-connection), [A34](#a34-the-field-construction-variant-is-fixed-and-unlogged) |
| 20 | Use clearer names, remove unused interfaces and misleading statistics, name cameras in alerts, align simulation with real cameras, and tidy the remaining camera diagnostics. | Low | Trivial | [A31](#a31-multi-tag-distance-and-tag-count-refer-to-different-target-sets), [A32](#a32-the-latest-target-can-remain-after-its-data-is-stale), [A55](#a55-camera-alerts-use-numbers-that-are-hard-to-identify-physically), [A68](#a68-the-name-test-has-several-meanings), [A69](#a69-simulated-cameras-do-not-resemble-the-recorded-real-cameras), [A72](#a72-the-camera-pass-rate-is-actually-an-average-score), [A73](#a73-unused-interfaces-make-the-subsystem-harder-to-understand) |

## Making settings easier to experiment with

The audit's second purpose was to let students change weights, thresholds,
and curve shapes without editing constants, rebuilding, and deploying for
every experiment. It considered four designs: live tuning built around
AdvantageKit, a configuration object, an offline experiment tool, and a
dashboard designed around student use.

Three reviewers compared them for reproducible replay, protection during
matches, student usability, implementation cost, testability, and fit with
the code. Two preferred the AdvantageKit design and one preferred the
offline design. All three recommended combining the same useful features.
The following proposal is that combined design.

### Requirements

1. **A log must contain the settings that actually ran.** Replay should
   restore those settings even if the source defaults have since changed,
   including logs recorded with live tuning disabled.
2. **Practice settings must not carry into a match.** With tuning disabled,
   the initial refactor should preserve the current arithmetic. Changing
   filter behavior is a later, separately evaluated step.
3. **Each setting must have one default.** The robot, replay, unit tests,
   and offline tool should all use the same definition.
4. **Students must be able to explain a decision.** They should see each
   check's score and why an observation was rejected.
5. **The 86 existing tests should retain their valid expectations.** The
   configuration refactor should require at most mechanical changes; the
   known stale test needs the separate repair already identified above.
6. **The design must fit the robot's processing budget.** Measure it on the
   roboRIO before an event. The design target is a 50 Hz loop with up to
   40 observations per second; an estimate alone is insufficient.

### Store the settings together

Introduce an immutable `VisionFilterConfig` record containing all settings
used for one processing batch. Immutable means code cannot change the
contents after creating it, including the collection of per-check settings.

For each check, store whether it is enabled, its weight, its midpoint in
standard units, and its steepness per standard unit. Global settings cover:

- Acceptance threshold and boundary margin.
- Velocity-history timeout and the score used when velocity cannot be checked.
- Fusion time window, position threshold, and score boost.
- Linear and angular uncertainty baselines and the single-camera multiplier.
- Number of loops between batch processing steps.

Build `DEFAULTS` once from the human-readable, unit-typed values in
`VisionConstants`. Remove the duplicate `_METERS` and `_RADIANS` constants so
there is one source for each default. Each parameter should have one name,
such as `test.pitchError.steepness`, `minScore`, or
`fusion.timeWindowSeconds`. Use that name consistently in NetworkTables,
AdvantageKit inputs, and property files used for experiments.

### Use the same processing code everywhere

Move the processing sequence out of `Vision.periodic` into
`VisionPipeline.step(observations, headingReference, config)`. It should
score readings, collect them, reject failures, update accepted-history
values, fuse each batch, calculate uncertainty, and return the measurements
to send. The robot, replay, tests, and offline tool should call this same
implementation.

Initially, `VisionFilter` can retain its `Test` enum. Each check reads its
settings from the configuration in its context. Add configuration-taking
versions of `scoreObservation` and `fuseCorrelatedObservations`, and let
existing signatures call them with `DEFAULTS` so existing callers keep
compiling. A later naming cleanup can use `Criterion`, `evaluate`,
`ScoringContext`, `ScoredObservation`, and `DEFAULT_CRITERIA` to distinguish
observation checks from JUnit tests.

### Record the effective settings as replay inputs

Before scoring each loop, call
`Logger.processInputs("Vision/Config", ...)` with the effective settings.
On the robot, this records the values. During replay, the same call restores
them from the log. AdvantageKit writes changed values, so a stable
configuration mainly costs a record at startup and another when it changes.

For deliberate replay experiments, apply an override file after restoring
the recorded settings. Log the override under
`ReplayOutputs/Vision/ConfigOverride`, and give the output log the override's
name so one experiment does not overwrite another. The default replay must
continue to use the settings from the recorded run.

### Allow dashboard changes during practice

Represent the numeric settings as `LoggedNetworkNumber` values under
`/Tuning/Vision/<key>`. Students can edit them in AdvantageScope's tuning view
or an Elastic dashboard tab. The values take effect only when
`/Tuning/Vision/Enable` is true and `DriverStation.isFMSAttached()` is false.
Check both conditions every loop.

Validate all changes before using them. The proposed rules include declared
ranges, clamping values to those ranges, converting negative weights to zero,
and preventing the two mandatory gate checks from being disabled through
NetworkTables. A zero total enabled weight must never reach the geometric
mean because it would make the calculation invalid.

The original design gives two conflicting fallbacks: it says non-finite
values should return to defaults and a zero weight sum should reset the
whole record, but it also says an invalid edit should keep the last valid
configuration. **Choose and test one fallback policy before implementing
live tuning.** In either case, show an alert naming the invalid setting and
log the values that actually take effect.

Adopt a changed configuration only when the observation buffer is empty.
A batch must be scored and fused using the same settings. With the proposed
100 ms batches, an ordinary edit would take effect within 100 ms. The
implementation also needs an explicit rule for a partially filled batch
when FMS attaches, so practice settings cannot affect the match.

### Prevent old practice values from affecting a match

Under FMS, use defaults and log `Vision/ConfigSource` as `defaults`. At each
robot-code start, write every dashboard value back to its default using
`set`, not `setDefault`: the `LoggedNetworkNumber` constructor can otherwise
retain a value republished by a reconnecting dashboard.

Show an alert whenever a dashboard setting differs from its default. Record
a configuration hash, a compact identifier of the settings, in startup
metadata and as an output when the effective configuration changes. This
lets a log show both the starting settings and edits during the session.

The proposal includes the tuning layer in every build. A compile-time
`TUNING_MODE` switch could provide stronger separation, but switching it
requires two deployments per practice session and makes behavior depend on
which build recorded the log. One design and one reviewer preferred that
option; the audit instead recommended the FMS check, startup reset, and
alerts. A `MatchType != None` lock could be an additional restriction.

### Show each observation and its decision

Log `Vision/Scored` every loop, with one structured row per observation,
including rejected observations. Each row should contain camera identity,
capture time, a sequence identifier, all nine check scores, total score,
acceptance decision, and the check contributing most to a low score. Mark
disabled check scores as NaN. Include the threshold margin and cluster
identifier where applicable.

For the geometric mean, each check's contribution can be expressed as
`weight * ln(score)`. The most negative contribution is the largest downward
influence on the total; a failed mandatory gate contributes negative
infinity. This defines the original audit's “weakest test” precisely. It is
an explanation of the scoring decision, not proof that this check alone
caused a rejection or that the observation was physically wrong.

Log `Vision/Fused` for each processed batch: pose, capture time, score,
camera count, and the linear and angular standard deviations actually sent.
`Vision/YawReference` should record the heading used by `yawConsistency`.
These records replace the two scalar summary keys.

Display curve steepness as a width in familiar units. Define the width as
the change in the input between scores 0.73 and 0.27. The existing pitch
curve would display about 115 degrees, making its lack of selectivity
visible on a slider. The implementation can still store steepness per
radian or per meter.

### Compare settings offline using the real Java filter

Create a JUnit test tagged `lab`, run by a dedicated
`./gradlew visionLab` task with the desktop HAL native libraries configured
by GradleRIO. This is a proposed task. It should read a `.wpilog` using
WPILib's `DataLogReader`, decode `PoseObservation[]` through AdvantageKit's
public `LogTable` interface, reconstruct robot cycles, and run them through
`VisionPipeline` with one or more configurations supplied as property files.
Using this path avoids a second binary decoder; `RecordStruct` itself is
package-private.

The tool should report:

- Accepted and rejected counts by camera and match phase.
- A histogram of all observation scores, showing the threshold.
- Pose-jump counts and a table of which checks most reduced scores.
- Observations whose acceptance decision changed between configurations.
- Results across a grid of parameter values and across a directory of logs.

The audit estimated about 12,700 observations in a VACHE match and about
25 ms to score them. On that estimate, testing 200 configurations would take
seconds. These are planning figures, not a measured guarantee for the
finished tool. Compare configurations across all 14 matches rather than
selecting one favorable match.

For filter experiments, this would replace the guide's Part 6 workflow of
making a throwaway clone, editing two source files, running full replays, and
using a Python comparison script. Full AdvantageKit replay remains useful
for questions involving the estimator or autonomous routines. An optional
later extension is a second filter that only logs its decisions, allowing
comparison without changing the measurements sent to the robot's estimator.

### Specify which settings take precedence

Use the following order, highest priority first, and log which source won:

1. A replay-only override file, selected by
   `VISION_CONFIG_OVERRIDE=<file>`.
2. Recorded `Vision/Config` inputs during replay.
3. NetworkTables values on the robot or in simulation, when tuning is enabled
   and FMS is not attached.
4. `DEFAULTS`.

Use an environment variable for the replay override. Gradle does not forward
`-D` properties automatically to the forked simulation JVM; `AKIT_LOG_PATH`
is an existing working example of the environment-variable approach.

### Implement the design in stages

Each phase should be useful on its own and end with something a student can
do that was previously difficult or impossible.

| Phase | Implementation | What becomes possible |
|---|---|---|
| 0 Make the inputs and checks reliable | Log camera inputs every loop; repair snapshot handoff and exception handling; validate timestamps; reject invalid scores; update velocity history; repair the yaw test; stop ignoring vision test failures; use current field dimensions; capture reference scores in tests. | Trust that the log contains what the filter processed and that a failing test is reported as a failure. |
| 1 Put settings and processing in shared code | Add `VisionFilterConfig`, `TestParams`, and `Kind`; read settings through the context; add compatible overloads; extract `VisionPipeline`. Reduce `Vision.periodic` to copying data, processing logged inputs, running the pipeline, sending results, and logging outputs. | Change a weight or steepness in a unit-test experiment with one line, without editing global constants. |
| 2 Record the decisions and settings | Add the scored and fused arrays, configuration inputs, yaw reference, and configuration source. Retire summary scalars and correct comments. | Explain each rejection and recover the settings from the log itself. |
| 3 Build the offline tool | Add `visionLab`, its log reader, metrics, parameter sweep, A/B comparison, directory mode, and a short fixture log with a smoke test. | Re-score a match in seconds and compare settings across all 14 matches. |
| 4 Add live tuning | Add `TunableVisionConfig`, enable control, FMS restriction, startup reset, alerts, and configuration hash. Commit an Elastic layout showing curve widths in degrees and each camera's latest observation. | Edit a setting at practice, watch the scores respond, and replay the session later. |
| 5 Improve the actual filtering | Introduce curves based on error relative to tolerance; separate gates from scoring; let distance primarily set uncertainty with a separate maximum-distance rejection; reduce single-tag angular trust; check implausible corrections; compare alternate poses; establish a starting pose from repeated agreement. Evaluate changes offline before deployment. | Use scores and thresholds whose behavior is supported by data. |

The audit estimated roughly two mentor-days for phases 0 through 2, one day
for phase 3, and one day for phase 4. Phase 5 is a season-long set of
experiments. The phase table originally proposed a five-second fixture log,
while detailed finding [A49](#a49-comparing-settings-currently-requires-too-much-replay-setup) proposed ten seconds; the required property is a
short, repeatable fixture with known results, and its final duration remains
to be chosen.

### Alternatives the audit considered

A compile-time tuning switch remains an option, with the deployment and
replay tradeoffs described above. The audit also considered and rejected:

- **WPILib `Preferences` as the settings source.** The modules already use
  it for turn-zero offsets, but its values persist through restarts and are
  not AdvantageKit inputs. A forgotten value could survive a reboot without
  being available to replay. The module use can replay correctly because
  its effects are already captured in logged hardware inputs; that does not
  make it suitable for filter settings.
- **A JSON file in the robot's deploy directory for live settings.** It
  would allow a file outside the code's parameter definitions to alter match
  behavior. The proposal allows a file only as an explicit replay override.
- **Logging only raw NetworkTables values.** A match recorded with tuning
  off would still lack its effective defaults, so replay could substitute
  different defaults from the current source tree. Log the complete effective
  configuration as inputs instead.
- **Reimplementing the filter in Python for offline analysis.** The audit
  cites a standing team rule against this. The proposed tool runs the same
  Java implementation used by the robot.

### Questions that need measurements or team decisions

1. **Logging cost on the robot.** The proposal estimates under 0.5 ms per
   loop for every-loop observation and scoring records on the roboRIO 2, but
   this has not been measured. Serialization was previously measured at
   12 ms and prompted the logging divisor. Use the existing
   `PROFILING_ENABLED` timers to measure the proposed format and rate.
2. **Why multi-tag results are absent.** Check the PhotonVision multi-target
   option, calibration at the active resolution, and whether two tags from
   the uploaded layout appear together. The settings export is missing from
   the repository.
3. **Front-camera placement.** The configured front cameras sit behind the
   robot center despite their names. The transforms are internally
   consistent, and the front-left and rear-left cameras agreed within 2 cm
   in Q27. Confirm that the configured locations match the hardware.
4. **How much to rely on the gyro.** A very large angular standard deviation
   for single-tag measurements would leave heading mostly to the gyro.
   Measure its drift over a match before choosing that setting.
5. **How to measure actual accuracy.** Offline statistics mostly measure
   agreement within the system. A practice-field procedure using marked,
   known robot positions is needed to tell whether a change improves real
   pose accuracy rather than merely making estimates agree more closely.
6. **How to establish field-relative gyro heading.** The raw gyro plus a
   fixed offset recorded at `setPose` would avoid using vision-corrected
   heading to judge vision. Check that offset against the selected auto
   before each match.

## Appendix A Detailed findings

The 76 entries below preserve the original findings and their proposed
remedies. Their original identifiers remain available for searching older
notes, but each entry has a plain-language title. Locations and severities
refer to the September audit at `f3ca52c`; they are not a current completion
checklist. Numerical suggestions for future settings are starting points for
evaluation, not validated competition settings.

### Scoring
#### A01 Tilt and height scores change too slowly

**Severity:** High. **Audited location:** `VisionFilter.java:156`.

Original identifier: `flat-tilt-height-sigmoids`.

The pitch, roll, and height curves use a steepness of 1.0 per radian or per
meter. Their response is too gradual over the range the robot should accept.

Define the curve using error divided by its tolerance, so the input has no
units: `1 - logistic(k * (x / tolerance - 1))`, with `k = ln 19` and
`logistic(z) = 1 / (1 + exp(-z))`. This gives 0.95 at zero error, 0.5 at the
tolerance, and 0.05 at twice the tolerance. Then choose `MIN_SCORE` again
and add tests for the curve shape.

#### A02 Accepted scores have little room above the threshold

**Severity:** High. **Audited location:** `VisionConstants.java:145`.

Original identifier: `score-ceiling-and-narrow-dynamic-range`.

The maximum scores are about 0.734 for a single tag and 0.783 for multiple
tags. Accepted single-tag readings therefore occupy only 0.65 to 0.734.

Revisit the threshold after fixing the curves. Test that the single-tag
ceiling at the intended operating distance exceeds `MIN_SCORE` by a useful
margin, and log the calculated ceiling.

#### A03 The threshold acts mostly as a distance limit

**Severity:** High. **Audited location:** `VisionFilter.java:323`.

Original identifier: `effective-bar-is-distance-cutoff`.

With five checks at nearly constant values, the threshold effectively
rejects even a good single-tag reading beyond about 5 m.

After correcting the curves, choose `MIN_SCORE` from an explicit meaning
for each check's score. Let distance primarily determine the measurement's
standard deviation, with a separate, larger maximum distance for outright
rejection. Multi-tag readings should not inherit the single-tag distance
curve without evaluation.

#### A04 Mandatory checks distort the quality score

**Severity:** Medium. **Audited location:** `VisionFilter.java:189`.

Original identifier: `hard-vetoes-inside-weighted-mean`.

The two yes-or-no checks add weight to the geometric mean even when they
pass with a score of 1. `moreThanZeroTags` does not fail for emitted readings.
Also, setting a gate's weight to zero silently disables it because `0^0 = 1`.

Run mandatory checks first and immediately reject failures with a recorded
reason. Calculate the geometric mean over the gradual checks only. Reselect
`MIN_SCORE` for that scale; the current 0.65 corresponds to about 0.505
on the gradual checks alone.

#### A05 Changing weights also changes the meaning of the threshold

**Severity:** Medium. **Audited location:** `VisionFilter.java:314`.

Original identifier: `root-normalization-couples-bar-to-weight-set`.

The root in the weighted geometric mean depends on the total enabled weight.
Changing weights or enabled checks therefore changes what `MIN_SCORE` accepts.

Keep the root: the review found that thresholding the unnormalized product
would make this coupling worse. Document the need to reevaluate the threshold
when changing the checks, and provide a helper that reports the score ceiling
and the equivalent requirement on an individual check.

#### A06 The ambiguity curve has a low maximum and allows uncertain readings

**Severity:** Medium. **Audited location:** `VisionFilter.java:143`.

Original identifier: `unambiguous-curve-too-permissive`.

The ambiguity score reaches only 0.646 even at zero ambiguity, yet permits
reported ambiguity up to about 0.37 at short range.

A candidate replacement has steepness about 19.6 per unit, producing scores
of 0.95, 0.5, and 0.05 at ambiguity 0, 0.15, and 0.30. Consider outright
rejection above 0.25 to 0.3. Prefer choosing between alternate pose solutions
when possible instead of relying only on an ambiguity penalty.

#### A07 Invalid and zero ambiguity values need special handling

**Severity:** Medium. **Audited location:** `VisionFilter.java:143`.

Original identifier: `ambiguity-sentinel-and-zero`.

PhotonVision's invalid value, -1, receives a score of about 0.99. Exactly
0.0 receives the best ordinary ambiguity score even when the pose is wrong.

Treat negative ambiguity as unknown, either assigning 0.5 or rejecting it.
Treat exactly 0.0 as unknown unless another reading supports it. Test both
cases and log ambiguity distributions separately for each camera.

#### A08 Invalid numerical scores can reach the estimator

**Severity:** Medium. **Audited location:** `Vision.java:226`.

Original identifier: `nan-score-fails-open`.

A NaN score makes both ordinary comparisons false. The current code can
therefore send it to the estimator with NaN standard deviations.

Define acceptance once as `score >= minScore`, which is false for NaN, and
store that decision with the scored observation. Use it for both logging and
filtering. Reject non-finite pose components when reading camera data and
protect the measurement consumer against non-finite standard deviations.

#### A09 The field inset rejects useful readings near walls

**Severity:** High. **Audited location:** `VisionFilter.java:183`.

Original identifier: `within-boundaries-hard-margin`.

The fixed 0.468 m inset rejected 205 otherwise strong E4 observations.

Use the physical field boundary plus about 0.3 m of tolerance for outright
rejection, and use a gradual score for proximity to the inset. Put the inset
and its steepness in the configuration, rebuild the rectangle when they
change, and record the boundary result for each observation.

### Estimator trust

#### A10 Accepted measurements can cause large corrections

**Severity:** High. **Audited location:** `Vision.java:242`.

Original identifier: `kalman-gain-no-gate`.

Each accepted single-camera observation moves the estimate roughly halfway
toward its position, with no check on the size of that correction.

The audit proposes a very large angular standard deviation while the gyro
is connected, particularly for single-tag readings. Derive linear standard
deviation from distance squared and tag count, aiming initially for a gain
of about 0.05 to 0.2 per observation. After initialization, add a check in
`Drive.addVisionMeasurement` for implausible differences, with roughly 1 m
and 30 degrees as candidate limits. Allow a deliberate reinitialization
policy after N consecutive rejections. Log standard deviations and the
corresponding gains.

#### A11 The uncertainty calculation has little relation to distance

**Severity:** High. **Audited location:** `Vision.java:242`.

Original identifier: `stddev-mapping-carries-no-information`.

Single-camera linear standard deviation stays between 0.082 and 0.092 m
regardless of tag distance.

Move the calculation to `VisionFilter.stdDevsFor(fused, config)`. Base it on
physical predictors of measurement error independently of the acceptance
score, test it directly, and log its result for every measurement. Either
use `MAX_STD_DEV` as a limit or remove it.

#### A12 Repeated readings from one camera can have more influence than fusion

**Severity:** Medium. **Audited location:** `Vision.java:246`.

Original identifier: `repeated-single-updates-outpull-fused`.

Applying two or three frames from one camera separately in a batch gives a
combined correction of about 0.77 to 0.89, compared with about 0.83 for one
fused measurement. Three updates can exceed the fused correction.

Possible fixes are to average each camera's readings within the window,
increase its standard deviation by the square root of its frame count, or
send only one measurement per camera per batch. Log the combined correction
strength per batch so the alternatives can be compared.

#### A13 A camera that consistently disagrees is still trusted individually

**Severity:** Medium. **Audited location:** `VisionFilter.java:454`.

Original identifier: `miscalibrated-camera-passthrough`.

A miscalibrated or shifted camera can fail to agree with any other camera
while its accepted readings retain normal single-camera trust.

For each camera, track a running average of its difference from the fused or
estimated pose at the reading's capture time, giving more weight to recent
readings. This is an exponentially weighted residual. Increase that camera's
standard deviation as the disagreement grows and alert above a threshold.
Track the same statistic by tag ID to help identify a damaged field tag.

### Fusion

#### A14 Cameras can be combined despite conflicting headings

**Severity:** Medium. **Audited location:** `VisionFilter.java:384`.

Original identifier: `translation-only-clustering-yaw-average`.

The grouping rule compares position and time but not heading. Averaging
conflicting headings can produce a direction neither camera reported.

Require pairwise heading agreement, initially considering 10 to 15 degrees.
Another option is the circular mean's resultant length, R: a value near 1
means the headings align, and a small value means they disagree. Refuse
fusion below about 0.9 or multiply angular standard deviation by `1/R`.
Expose the relevant settings and add tests with 45-degree and 180-degree
disagreements. Consider leaving yaw unfused while all observations use one tag.

#### A15 The score boost quickly reaches its maximum

**Severity:** Medium. **Audited location:** `VisionFilter.java:445`.

Original identifier: `correlation-boost-saturates`.

Multiplying the best score by 1.4 provides little distinction among the
combined readings in the reachable range.

Average readings within each camera first, then consider combining
uncertainties with `1 / sqrt(sum(1/sigma^2))`, where sigma is a measurement's
standard deviation. A score-weighted mean is another option. If a score
boost is retained, the audit suggests adding it in log-odds space, which
avoids reaching 1.0 through a simple cap; log-odds means `ln(p / (1 - p))`.
These are design alternatives to evaluate, not a claim that the current
quality score is already a calibrated probability.

#### A16 Batch boundaries separate readings that should be compared

**Severity:** Medium. **Audited location:** `Vision.java:224`.

Original identifier: `batch-boundaries-split-correlated-pairs`.

Fixed loop-count batches split about one fifth of pairs that otherwise meet
the fusion criteria.

Use a sliding window based on capture times. Keep observations newer than
the newest timestamp minus the window across processing calls. Emit a group
once its newest member is old enough to leave that window.

#### A17 Robot motion reduces the chance of combining camera readings

**Severity:** Medium. **Audited location:** `VisionFilter.java:384`.

Original identifier: `batch-latency-and-fusion-window`.

The code compares field positions directly even when their capture times
are up to 150 ms apart. Movement during that interval can make agreeing
cameras appear to disagree.

Use the odometry change to project readings to a common time before
comparing them. Alternatively, compare each vision pose's difference from
odometry at its own capture time. Log how often fusion occurs at different
chassis speeds.

#### A18 A chain of agreeing pairs can form an overly broad group

**Severity:** Low. **Audited location:** `VisionFilter.java:391`.

Original identifier: `transitive-chaining-exceeds-thresholds`.

The union-find algorithm correctly joins connected pairs, but a chain of
such pairs can have endpoints outside both the allowed time and position
separation. A unit test currently expects that behavior.

Require a new member to agree with the group's center or with every member,
or explicitly limit the maximum separation within a group. Update the unit
test to match the chosen rule.

#### A19 Large batches silently skip fusion

**Severity:** Low. **Audited location:** `VisionFilter.java:347`.

Original identifier: `max-observations-fusion-fallback`.

A batch with more than 32 observations falls back to processing without
fusion.

Log the batch size and a `fusionSkipped` flag. Allocate storage dynamically
or raise the limit and make it configurable. Consider dropping results older
than about 0.3 s at ingestion and count those drops. This 0.3 s suggestion
is a separate candidate from the general 0.5 s age limit in [A24](#a24-camera-timestamps-and-clock-synchronization-are-not-validated).

#### A20 Older readings can undo newer estimator updates

**Severity:** Low. **Audited location:** `Vision.java:234`.

Original identifier: `cross-batch-out-of-order-discards-updates`.

Sorting one batch does not stop an older timestamp in the next batch from
causing the estimator to remove newer vision updates.

Track the last timestamp sent and drop older measurements, or delay batches
by the expected spread in camera latency. Use `Double.compare` for timestamp
sorting.

### Camera data handling

#### A21 Snapshot replacement can repeat or lose frames

**Severity:** Medium. **Audited location:** `VisionThread.java:42`.

Original identifier: `snapshot-replace-dup-and-loss`.

The latest-snapshot handoff has no record of whether the main loop already
processed its contents. It can process frames twice or lose an overwritten
snapshot.

A queue drained by `periodic` can preserve pending data. A generation number
can prevent processing the same snapshot twice, but by itself does not
recover overwritten snapshots. Keep a last-processed timestamp per camera,
log PhotonVision's `sequenceID`, and count duplicate and skipped frames.
Choose the handoff according to both requirements.

#### A22 One camera exception can stop all camera polling

**Severity:** Medium. **Audited location:** `VisionThread.java:148`.

Original identifier: `notifier-exception-freezes-vision`.

An uncaught exception on the shared camera thread stops updates for all
four cameras and leaves the old snapshot reporting its old connection state.

Catch errors for each camera, log the exception, and mark its snapshot as
faulted. Timestamp snapshots and treat one older than 0.2 s as disconnected.

#### A23 All cameras share one polling thread

**Severity:** Low. **Audited location:** `VisionThread.java:145`.

Original identifier: `single-lock-all-cameras-stall`.

The original concern named a lock, but review found the lock was not the
cause. Camera work runs sequentially on one `Notifier` thread, so a slow
camera can delay the others.

Record each polling cycle's duration and the gap between cycles as inputs.
Consider a separate `Notifier` for each camera, together with a queue handoff
so a delayed cycle does not cause data loss.

#### A24 Camera timestamps and clock synchronization are not validated

**Severity:** Medium. **Audited location:** `VisionIOPhotonVision.java:137`.

Original identifier: `no-timestamp-sanity-guard`.

After coprocessor reconnects, observations stamped 38 to 52 s in the future
were scored and sent to the estimator.

At ingestion, reject and count readings more than 20 ms ahead of the FPGA
clock or 0.5 s behind it, with `timeSinceLastPong` above 2 s, or with latency
outside 0 to 250 ms. `timeSinceLastPong` indicates the age of the last
clock-sync response. Log latency, publish time, sync health, and `sequenceID`
per observation. Wrap the estimator call to report its own rejected updates.

#### A25 A negative time interval receives a perfect velocity score

**Severity:** Low. **Audited location:** `VisionFilter.java:222`.

Original identifier: `velocity-negative-dt-free-pass`.

`velocityConsistency` returns 1.0 for a negative time difference, as it does
for nearly simultaneous observations.

Return the uncertain score when `dt < 0`. Timestamp checks reduce exposure
to bad times, but an absolute age limit alone does not guarantee that
successive readings arrive in timestamp order; keep the explicit check.

#### A26 The alternate camera pose is discarded

**Severity:** Medium. **Audited location:** `VisionIOPhotonVision.java:126`.

Original identifier: `alternate-pnp-pose-discarded`.

Keeping only one PnP solution leaves ambiguity as a rejection mechanism,
removing 29% to 42% of observations per match under the audited settings.

Calculate robot poses from both transforms. When a heading reference exists
and the candidate yaws differ by more than a few degrees, prefer the one
closer to heading at capture time. Record that choice and retain both poses
for later replay of other policies. Then reconsider the ambiguity weight and
`MIN_SCORE`. Most high-ambiguity readings are also 5.7 to 7.2 m from the tag,
so choosing the alternate solution alone will not make them acceptable.

#### A27 The single-tag branch chooses the first target without ranking it

**Severity:** Medium. **Audited location:** `VisionIOPhotonVision.java:117`.

Original identifier: `single-tag-uses-arbitrary-first-target`.

The target list is unsorted; selecting its first entry discards other
potentially better targets.

Prefer emitting an observation for each target that exists in the field
layout, including its tag ID, image area, and ambiguity, so the filter can
evaluate it. At minimum, deliberately select the largest or least ambiguous
target. Add all in-layout IDs to `tagIds` and log the targets in each result.

#### A28 The recorded matches contain no multi-tag observations

**Severity:** Medium. **Audited location:** `VisionIOPhotonVision.java:81`.

Original identifier: `multitag-never-fires-cause-unknown`.

All observations in 14 matches were single-tag. The repository does not
contain enough camera configuration information to explain why.

Log `targets.size()` and all target IDs per result. Check each coprocessor's
multi-target setting, uploaded field layout, and calibration. Commit a
PhotonVision settings export. If multiple tags never appear together in an
image, tune for single-tag operation rather than assuming multi-tag results.

#### A29 The observation record omits useful diagnostic information

**Severity:** Medium. **Audited location:** `VisionIO.java:39`.

Original identifier: `pose-observation-record-too-thin`.

`PoseObservation` does not include tag ID, alternate pose, image area,
latency, or sequence number.

Add those fields and clock-sync health using the versioning approach in
[A30](#a30-changing-the-observation-format-can-break-old-replay-logs). Existing `TagIds` and timestamps already allow some per-tag exclusion
and duplicate detection; the new fields would make richer policies possible.

#### A30 Changing the observation format can break old replay logs

**Severity:** Medium. **Audited location:** `VisionIO.java:39`.

Original identifier: `poseobservation-record-layout-is-an-unversioned-replay-contract`.

The binary observation record is part of the saved-log format. Simply
renaming its Java class does not protect old logs: AdvantageKit's
`LogTable` locates data by field key and checks only the struct prefix.

Write the new format under a new key, such as `PoseObservationsV2`, and keep
a decoder that reads the old key into the old record. Another option is to
keep the record and add new data in parallel arrays. Add a CI test that
replays an archived log fragment.

#### A31 Multi-tag distance and tag count refer to different target sets

**Severity:** Low. **Audited location:** `VisionIOPhotonVision.java:91`.

Original identifier: `multitag-avg-distance-inconsistent`.

Average distance is calculated over one target set while the reported tag
count describes another.

Average distances over `fiducialIDsUsed`, the targets used in the pose
solution. If the total number detected is useful too, report
`targets.size()` separately.

#### A32 The latest target can remain after its data is stale

**Severity:** Low. **Audited location:** `VisionIOPhotonVision.java:66`.

Original identifier: `latest-target-observation-stale`.

`latestTargetObservation` changes only when a new result arrives.

Reset it at the start of `updateInputs`, or attach a timestamp so callers can
check freshness. Complete the unfinished comment explaining its behavior.

#### A33 The field-layout choice can survive a later FMS connection

**Severity:** Low. **Audited location:** `Vision.java:314`.

Original identifier: `apriltag-layout-fms-frozen-on-notifier`.

The FMS check occurs once when the layout is first loaded and is then cached.

Load the layout in the constructor on the main thread and reconsider the
choice when FMS attaches. Alternatively, make it an explicit deployment
choice recorded in metadata, consistent with the intended match policy.
Derive the allowed field rectangle from the selected layout's dimensions.

#### A34 The field construction variant is fixed and unlogged

**Severity:** Low. **Audited location:** `VisionConstants.java:26`.

Original identifier: `field-layout-variant-hardcoded`.

The code selects the AndyMark layout. The welded layout differs by a common
1.9 cm shift and about another 2 cm for tags 13 through 16 and 29 through 32.
The log does not identify the selection.

Choose the variant through a logged input or deployment setting, display it
in a startup alert, and derive field boundaries from the loaded layout.

### Logging and replay

#### A35 Individual scores are not logged

**Severity:** High. **Audited location:** `Vision.java:251`.

Original identifier: `per-test-scores-never-logged`.

The code discards the individual check results. The two summary values can
retain only one measurement's values from a batch.

For every loop and camera, log one structured row for each scored
observation, including rejected ones. Include camera, capture time, each
check's score, total, acceptance decision, and cluster ID. For each batch,
log arrays of fused scores, camera counts, timestamps, poses, and standard
deviations. Retire the two summary scalars.

#### A36 Rejected poses cannot be traced back to their scoring decisions

**Severity:** High. **Audited location:** `Vision.java:186`.

Original identifier: `no-rejection-attribution-or-observation-key`.

A rejected pose has no observation identifier or explanation connecting it
to the original camera input and scoring results.

Assign a sequence ID when reading the camera data. Log each check's
`weight * ln(score)` contribution, the total's margin above or below the
threshold, and the observation's score even when rejected. Add accepted and
rejected counts for each cycle. This supports a histogram spanning both
sides of the threshold and a direct explanation of each decision.

#### A37 Logging fewer input cycles changes replay behavior

**Severity:** High. **Audited location:** `Vision.java:137`.

Original identifier: `logging-divisor-replay`.

At the audited commit, the filter processes every loop but records inputs
only on alternate loops.

Log every input loop and measure the cost, or process only data that is
logged. Check the expected divisor during replay or read it from metadata.
Add a replay comparison using distinct accepted observations, and repeat
prior analyses before relying on conclusions drawn from incomplete inputs.
The October branch change to divisor 1 does not restore data absent from
older logs.

#### A38 Replay can overwrite recorded inputs with an empty camera snapshot

**Severity:** Medium. **Audited location:** `Vision.java:130`.

Original identifier: `replay-odd-loops-see-empty-live-snapshot`.

On odd loops in the audited code, the empty live-thread snapshot overwrites
inputs without a following `processInputs` call to restore recorded data.

Do not start `VisionThread` in replay, or do not copy its snapshot there.
Prefer identifying new frames with per-camera sequence numbers and calling
`processInputs` every loop. See the scope note for the subsequent startup
and divisor changes.

#### A39 Logs do not identify the filter settings that ran

**Severity:** Medium. **Audited location:** `Robot.java:147`.

Original identifier: `no-parameter-values-in-log`.

The recorded build information cannot reconstruct all constants, especially
when the robot ran uncommitted changes.

Record build-time defaults with `Logger.recordMetadata` during construction.
Once settings are configurable, record the effective configuration as
inputs. Record its hash as an output too, so edits during a session remain
visible.

#### A40 Passing the filter does not prove a measurement was applied

**Severity:** Medium. **Audited location:** `Vision.java:187`.

Original identifier: `accepted-not-applied-no-feedback`.

The estimator can ignore or replace vision updates without feedback to the
filter's logs.

In `Drive.addVisionMeasurement`, record the measurement's age, whether its
timestamp is inside the odometry history buffer, and the change in the
estimated pose. Also record the pose and standard deviations sent, grouped
per batch.

#### A41 Replay needs an explicit way to change settings

**Severity:** Medium. **Audited location:** `Constants.java:25`.

Original identifier: `replay-tunables-need-override-path`.

Selecting replay requires a source edit, and recorded NetworkTables inputs
prevent ordinary live edits from changing a replay experiment.

Select replay mode with an environment variable. Apply an explicit override
after restoring logged settings, and log that override as an output so its
effect is distinguishable from the original run.

#### A42 A replay used different settings from the robot run

**Severity:** Medium. **Audited location:** `VisionConstants.java:155`.

Original identifier: `replay-used-different-build`.

The E4 replay used a different logging divisor and enabled-check set from
the recorded robot build.

For a baseline replay, use the commit SHA in `RealMetadata` or fail clearly.
Commit code before events and record the enabled set and every constant in
the log. Deliberate experiments with different settings should use the
explicit, labeled override path rather than being mistaken for a baseline.

#### A43 Missing replay inputs cause a delayed and confusing crash

**Severity:** Medium. **Audited location:** `Drive.java:101`.

Original identifier: `missing-input-keys-silently-default-then-crash-elsewhere`.

A missing input key silently receives a default value, and the program
fails about 130 s later in unrelated code.

Keep input keys stable or support aliases for the old module keys. Before
replay, check that required keys exist and list any missing ones. Initialize
`chassisSpeeds`, alert if no odometry sample arrives for N loops, and make
the replay task exit with a nonzero status when the robot program crashes.

#### A44 No test proves that replay reproduces observation decisions

**Severity:** Medium. **Audited location:** `Vision.java:251`.

Original identifier: `no-replay-fidelity-test-and-batch-scalars-move-the-wrong-way`.

There is no automatic comparison of replayed and recorded decisions. The
batch summary values can even suggest a change in the opposite direction
from the actual observation counts.

Use the structured observation arrays described above. Bundle a log fragment
and assert that replay makes the same decision for every observation when
settings match the recorded run.

#### A45 Full replay mixes filter changes with changes elsewhere

**Severity:** Medium. **Audited location:** `Drive.java:418`.

Original identifier: `replay-is-whole-robot-resimulation-not-filter-experiment`.

A non-vision change produced a 13 m pose difference even though the vision
filter's decisions were identical.

Define filter experiments using the filter's own outputs. Treat `Drive/Pose`
as a result of the whole robot program. Log the measurements actually
applied so a separate estimator, used only for comparison, can process the
same measurement sequence.

#### A46 Replay results do not identify or preserve the experiment

**Severity:** Low. **Audited location:** `Robot.java:250`.

Original identifier: `replay-outputs-have-no-provenance-or-label`.

The output can overwrite a previous run and lacks a useful experiment label
and settings record.

Select replay by environment variable and accept a label and override file
at startup. Save `<log>_<label>.wpilog` in a separate directory. Record the
label, override contents, and resolved settings in metadata. Provide a
Gradle task that reports a program crash as a failed run.

#### A47 A reported sample reduction counted log changes instead of readings

**Severity:** Low. **Audited location:** `Vision.java:139`.

Original identifier: `replay-doc-sample-count-claim-is-dedup-artifact`.

The guide's claim of “50 to 85% fewer samples” came from counting log entries
whose repeated values the writer omitted.

Compare decoded observation rows, not the number of entries in a log
stream. The audit records this correction as already applied to the guide.

#### A48 Logging performance needs measurement on the robot

**Severity:** Low. **Audited location:** `VisionFilter.java:309`.

Original identifier: `allocation-cpu-profile`.

Serialization was measured at 12 ms in earlier work, motivating the logging
divisor. That cost matters more than speculation about filter allocations.

Log the phases of `periodic` as timing outputs every loop and measure divisor
1 on the roboRIO. The audit notes that branch `fix/profiling-to-wpilog`
already records these timings. If serialization dominates, use compact
per-observation structs instead of repeated `Pose3d` arrays.

#### A49 Comparing settings currently requires too much replay setup

**Severity:** Medium. **Audited location:** `VISION_GUIDE.md:1317`.

Original identifier: `no-offline-multi-config-rescoring-harness`.

Comparing two configurations on one match takes a clone, two source edits,
and a full replay for each configuration.

Build the `visionLab` tool described above, using AdvantageKit's public
`LogTable` path because `RecordStruct` is package-private. Include a short
fixture log and a reference-result test. This finding proposed ten seconds
of fixture data; the implementation phase table proposed five seconds.

### Initial pose and alerts

#### A50 The first disabled measurement can set the pose without extra checks

**Severity:** Medium. **Audited location:** `Drive.java:429`.

Original identifier: `first-pose-bootstrap-unvetted`.

The first measurement received while disabled can initialize the estimator
without a stronger quality requirement than ordinary filtering.

Require two cameras or N consistent observations within 0.2 m, passed
through a richer measurement record that carries the necessary quality
information. Log the initialization and its score. Allow a controlled
reinitialization while disabled when vision disagrees by more than 1 m.

#### A51 Pose initialization can remain armed through autonomous

**Severity:** Medium. **Audited location:** `Drive.java:428`.

Original identifier: `seed-fires-in-fms-gap`.

An unused first-vision initialization can fire in the disabled gap between
autonomous and teleop.

Require `!poseInitialized` and agreement across cameras or frames. Prevent
this initialization while FMS is attached between auto and teleop, and log
`Drive/PoseSeeded` when initialization occurs.

#### A52 Repeated distant readings are not used to establish a starting pose

**Severity:** Medium. **Audited location:** `VisionConstants.java:73`.

Original identifier: `prematch-100pct-rejection-no-init`.

Every pre-match observation was rejected in both examined elimination
matches. Most were wrong, and cameras disagreed by 6 to 8 m.

While disabled, collect distant observations over time and across cameras.
Look for a consistent group, reject the competing mirror-solution group,
and initialize from the agreeing group's mean. The proposed remedy is
agreement across readings, not simply a lower acceptance threshold.

#### A53 Other code cannot tell that the pose is uninitialized

**Severity:** Medium. **Audited location:** `Robot.java:401`.

Original identifier: `uninitialized-pose-consumers`.

There is no explicit indication that a reliable initial pose has not been
established.

Log `Drive/PoseInitialized`. Alert while disabled with FMS attached if no
vision has been accepted for 5 s. Give the LED pose-seeking display a
distinct pattern for the no-pose state.

#### A54 Rejecting every observation does not raise an alert

**Severity:** Medium. **Audited location:** `Vision.java:153`.

Original identifier: `no-acceptance-watchdog-alert`.

The filter can see tags and reject everything without warning the operators.

Calculate each camera's actual acceptance rate over a 2 s window. Warn
when tags are visible but no reading has been accepted for 5 s, and report
an error at auto initialization if the pose is uninitialized despite tags
being visible. Unit-test the timing window.

#### A55 Camera alerts use numbers that are hard to identify physically

**Severity:** Low. **Audited location:** `Vision.java:105`.

Original identifier: `camera-alerts-by-index`.

Alerts refer to camera indexes rather than camera names.

Add `name()` to `VisionIO`, use the name in alert text and fault keys, and
log the mapping from names to indexes at startup.

### Settings and constants

#### A56 There is no shared interface for filter settings

**Severity:** High. **Audited location:** `VisionFilter.java:139`.

Original identifier: `parameters-not-runtime-tunable`.

Weights are literals in the enum, steepness values are literals inside
checks, and there is no configuration object to substitute during an experiment.

Pass the proposed `VisionFilterConfig` into `scoreObservation` and
`fuseCorrelatedObservations`. A weight of zero can disable an optional
scoring check; mandatory gates need their separate protection. Later, a
second filter could evaluate alternative settings and log decisions without
sending them to the estimator.

#### A57 Some settings have a changeable original and a frozen conversion

**Severity:** Medium. **Audited location:** `VisionConstants.java:73`.

Original identifier: `unit-typed-constants-have-final-derived-twins`.

Unit-typed tolerances are mutable, but their derived `final` doubles are
calculated once. Changing one does not necessarily change the value the
filter uses.

Keep one source per setting. Either convert the unit-typed default at the
point of use or store the converted value in the configuration. Make
settings that are not intended to change `final`.

#### A58 The active and default check sets are the same mutable object

**Severity:** Medium. **Audited location:** `Vision.java:79`.

Original identifier: `enabled-tests-aliases-default-set`.

Editing `Vision.enabledTests` also changes the default set used by tests.

Make `DEFAULT_ENABLED_TESTS` unmodifiable and give each configuration its own
copy, or represent optional-check enabling through a zero weight.

#### A59 The velocity check never receives updated history

**Severity:** Medium. **Audited location:** `Vision.java:178`.

Original identifier: `last-accepted-arrays-never-written`.

The arrays for each camera's last accepted pose and time are read but never
written. The writes existed in commit `d6de05e` and were lost in the merge.

Update them for each accepted observation after acceptance and before
fusion, or remove the unused history. Also consider comparing against the
estimated pose at the observation's timestamp: using the last accepted
reading can let one bad pose become the reference for subsequent checks.

#### A60 The yaw reference is late and partly determined by vision

**Severity:** Medium. **Audited location:** `Vision.java:180`.

Original identifier: `gyro-yaw-sampled-at-scoring-time`.

The check samples the estimator heading at processing time. That heading
includes prior vision corrections and does not necessarily match the
capture time.

Make the reference a function of timestamp, preferably raw gyro heading at
capture plus a fixed field offset recorded at `setPose`. Do not run the
check until that offset exists.

#### A61 Comments describe old settings and unreachable scores

**Severity:** Medium. **Audited location:** `VisionConstants.java:107`.

Original identifier: `stale-constant-comments`.

The comments refer to a threshold of 0.6, a 1.3× boost, a majority rule, and
score examples the current filter cannot produce.

Rewrite comments to match the implemented values. Remove `MAX_STD_DEV` or
use it as a clamp. Put numerical examples in generated tables or tests so
changes can be detected.

#### A62 Existing runtime settings are not a model for replayable tuning

**Severity:** Low. **Audited location:** `Module.java:46`.

Original identifier: `precedents-not-replay-safe`.

The repository's use of `Preferences` does not record those values as
AdvantageKit inputs, and the values persist across restarts.

Build tuning on `LoggedNetworkNumber` and the logged effective configuration.
The module's existing `Preferences` use replays safely only because its
effect is captured in logged hardware inputs; that reasoning does not cover
vision filter settings.

### Tests simulation and documentation

#### A63 A stale yaw test fails without failing the build

**Severity:** Medium. **Audited location:** `VisionFilterTest.java:608`.

Original identifier: `stale-failing-yaw-test-hidden-by-ignorefailures`.

The wrong-yaw test uses a default set that no longer includes
`yawConsistency`. The audit found it failed on every build that ran it.

Explicitly include `yawConsistency` in that test and have tests use a
configuration initialized from defaults. Stop ignoring these failures in
verification. The audit suggests either limiting `ignoreFailures` to deploy
operations or disabling it for the vision tests.

#### A64 Tests do not require useful curve shapes or score margins

**Severity:** Medium. **Audited location:** `VisionFilterTest.java:154`.

Original identifier: `tests-do-not-pin-curve-shape`.

Ordering tests can pass even when the curves barely change and the
acceptance threshold is too close to the highest possible score.

For each gradual check, test at least 0.9 at zero error, 0.45 to 0.55 at the
tolerance, and at most 0.1 at twice the tolerance. Keep explicit reference
scores, including the audit's examples 0.712, 0.775, and 0.627, with their
fixtures. These reference-value tests are often called golden tests.

For the redesigned filter, require a typical single-tag observation to
exceed `MIN_SCORE` by at least 0.1 and an impossible observation to fall
below it; also test the margin to the ceiling. These redesigned behavior
targets are distinct from preserving the old arithmetic during the initial
configuration refactor. Appendix D retains that refactor's original
reference-value sketch and its smaller ceiling margin.

#### A65 The batch processing operation has no direct test

**Severity:** Medium. **Audited location:** `Vision.java:125`.

Original identifier: `vision-periodic-untested`.

`Vision.periodic` combines responsibilities that are not tested together.

Extract batch processing into a function with explicit inputs and results
that can be tested without running the robot. Inject the `VisionThread`
dependency instead of obtaining only its singleton instance.

#### A66 A velocity test passes because another check rejects its pose

**Severity:** Medium. **Audited location:** `VisionFilterTest.java:714`.

Original identifier: `velocity-test-passes-for-wrong-reason`.

The test's supposedly impossible movement ends outside the allowed field.
The boundary rejection therefore hides whether velocity checking works.

Use a physically impossible move whose ending pose is still inside the
field, and explicitly include `velocityConsistency` in the enabled set.
Label test groups that exercise checks disabled on the robot.

#### A67 Boundary tests use the wrong season field size

**Severity:** Medium. **Audited location:** `VisionFilterTest.java:26`.

Original identifier: `test-file-hardcodes-2024-field`.

The tests hard-code 16.54 by 8.21 m rather than using the robot code's field
constants.

Use the shared constants. Add tests at the far corners and 1 mm inside and
outside each relevant edge.

#### A68 The name Test has several meanings

**Severity:** Medium. **Audited location:** `VisionFilter.java:138`.

Original identifier: `test-name-collision-and-three-meanings-of-test`.

The filter's `Test` enum collides with JUnit's `@Test`, and the documentation
uses “test” for several different ideas.

Rename the filter vocabulary to `Criterion`, `evaluate`, `ScoringContext`,
`ScoredObservation`, and `DEFAULT_CRITERIA`. Update the guide in the same
change so readers can map the explanation to the code.

#### A69 Simulated cameras do not resemble the recorded real cameras

**Severity:** Medium. **Audited location:** `VisionIOPhotonVisionSim.java:26`.

Original identifier: `sim-camera-model-does-not-match-real`.

Simulation uses 35 frames per second, 30 ms latency, and multi-tag results.
The robot data reflects 24 frames per second, about 50 ms, and single-tag
results.

Build the simulation properties from exported camera calibration. Use
24 frames per second, latency of 50 plus or minus 10 ms, and sight range
around 8 m. To exercise single-tag behavior, provide a reduced layout;
the simulator has no direct switch to disable multi-tag processing.

#### A70 The simulation starts where no useful observations are accepted

**Severity:** Low. **Audited location:** `Drive.java:84`.

Original identifier: `sim-smoke-run-rejects-everything`.

The simulated robot begins at the origin, outside the allowed arena and
6 to 8 m from any tag.

Start it inside the arena and within about 4.5 m of visible tags. Add a
simple simulation check requiring accepted observations within N seconds.

#### A71 The shared vision simulation updates four times per camera cycle

**Severity:** Low. **Audited location:** `VisionIOPhotonVisionSim.java:70`.

Original identifier: `sim-pose-read-from-notifier-and-4x-update`.

The shared simulation is updated from the `Notifier` thread once for each
of four cameras, and reads the simulated robot pose there.

Update the shared simulation once per cycle from the main thread. Transfer
the pose through an `AtomicReference`, and disable video streams except
when they are needed for debugging.

#### A72 The camera pass rate is actually an average score

**Severity:** Low. **Audited location:** `Vision.java:64`.

Original identifier: `camera-pass-rate-hardcoded-misnamed`.

Four hard-coded filters average observation scores. Their label implies the
fraction of observations accepted, which is a different quantity.

Size the array from `io.length`. Either rename the statistic or feed it
acceptance flags so it calculates a real pass rate. Log it regardless of
pose-display flags, or remove it if it is not useful.

#### A73 Unused interfaces make the subsystem harder to understand

**Severity:** Low. **Audited location:** `Vision.java:120`.

Original identifier: `dead-api-and-unused-types`.

`getTargetX` throws an exception; the observation `type` field and `MEGATAG`
enum values are unused.

Implement or remove `getTargetX`, and remove the unused field and enum
values. If multi-tag results become available, consider using
`bestReprojError`, the reported image reprojection error, as a scoring input;
otherwise stop logging it.

#### A74 Old documents describe a filter that no longer exists

**Severity:** Low. **Audited location:** `VISION_TESTS.md:93`.

Original identifier: `vision-tests-md-stale`.

At the time of the audit, four outdated documents were staged for deletion.

The audit recommended committing those deletions and replacing “majority”
in the `CORRELATION_BOOST_FACTOR` comment. The document-deletion instruction
is historical, not an instruction to delete files from today's checkout.

#### A75 Two numerical claims in the guide were corrected

**Severity:** Low. **Audited location:** `VISION_GUIDE.md:832`.

Original identifier: `vision-guide-numeric-claims-check`.

The audit identified two incorrect numerical claims in the vision guide.
It records the fixes as applied in the guide's revision on the same day.
No additional remedy was specified in this finding.

#### A76 The guide overlooks settings that change immediately

**Severity:** Medium. **Audited location:** `VISION_GUIDE.md:642`.

Original identifier: `guide-section-7-runtime-claim-hides-live-vs-frozen-split`.

The guide says settings cannot be changed at runtime, but roughly half the
static fields are read live while others have frozen derived values.
Runtime writes are not logged.

Until the configuration refactor, add a table in guide section 7 showing
which values are read live and which were calculated earlier, with an
explanation that runtime changes are unlogged. After the refactor, obtain
one configuration snapshot per loop from the shared source.

## Appendix B Findings that were disproved

Two candidate findings did not survive review:

- **A stopped camera stream can be detected.** PhotonLib's `isConnected`
  uses a heartbeat with a 0.5 s debounce, so it detects a stopped stream
  within that time. The rejected identifier was
  `stalled-camera-reports-connected`. This is different from the local
  polling-thread failure in [A22](#a22-one-camera-exception-can-stop-all-camera-polling), which leaves an old connection value frozen
  in the robot's snapshot.
- **The unit-test playground output is visible.** GradleRIO configures test
  logging to show standard output, so Gradle does display the experiment's
  output. The rejected identifier was
  `playground-output-invisible-and-not-preserved`.

## Appendix C Behavior the audit checked and found correct

These results help narrow the work. A correct calculation can still be used
with unsuitable settings or unsuitable input data.

- The weighted geometric mean implements its stated definition: changing
  the order does not change the result, zero rejects with positive weight,
  and weights act as exponents.
- The circular mean used for fused yaw is mathematically correct.
- Combining the coordinate transforms is correct in both the single-tag
  and multi-tag branches.
- The union-find grouping algorithm is correct and retains no state between
  calls. The concern in [A18](#a18-a-chain-of-agreeing-pairs-can-form-an-overly-broad-group) is whether that grouping rule is appropriate,
  not whether the algorithm implements it correctly.
- All four camera quaternions are normalized and correspond to the angles
  stated in the comments. A quaternion is the four-number representation
  used for a 3D rotation. The left and right pairs are exact mirrors.
- During normal synchronized operation, capture timestamps use the roboRIO
  clock. Observations were 48 to 88 ms old when logged across three matches.
  [A24](#a24-camera-timestamps-and-clock-synchronization-are-not-validated) concerns the separate reconnect case.
- `Drive` runs before `Vision`, so the current loop's odometry sample is
  already in the history buffer when vision measurements arrive.
- Replaying E8 with the examined code reproduced all 4,293 of 4,293 accepted
  observation records byte-for-byte. This supports deterministic filter
  decisions when inputs and settings match; it does not make incomplete
  logs or unrelated robot-code changes irrelevant.
- Snapshot arrays are newly allocated for each update, so subsequent
  updates do not mutate the arrays already handed to the main loop.

## Appendix D Original design sketches

These are the original proposal's illustrative Java sketches, retained for
implementation reference. They are not complete, compiled implementations.
The prose above defines the intended behavior. Read the sketches with these
limitations in mind:

- A Java record containing a mutable `EnumMap` is not deeply immutable by
  itself. The finished configuration must protect or copy that collection.
- The sketches omit members and implementations, use placeholders such as
  `...`, and refer to rows and helpers not fully defined in the example.
- The pipeline sketch returns `rows(buffer)` while accumulating a batch.
  The finished logging path must emit each new observation once, preserve
  rejected rows, and avoid repeatedly logging the accumulated buffer as
  though those were new observations.
- The shown heading argument is one `Rotation2d`. The capture-time heading
  recommendation requires a timestamp-dependent reference when implemented.
- Validation needs the single fallback policy discussed above. Changes in
  settings and FMS state also need the explicit batch-boundary handling.
- The reference-score test is for preserving the original arithmetic during
  refactoring. The later curve redesign must deliberately update those
  expectations and use its own target margins.

### Configuration record

```java
public record TestParams(boolean enabled, double weight, double midpoint, double steepness) {
  public TestParams withWeight(double w) { return new TestParams(enabled, w, midpoint, steepness); }
}

public record VisionFilterConfig(
    EnumMap<Test, TestParams> tests,
    double minScore,
    double boundaryMarginMeters,
    double velocityTimeoutSeconds,
    double velocityUncertainScore,
    double fusionTimeWindowSeconds,
    double fusionPoseThresholdMeters,
    double fusionBoostFactor,
    double linearStdDevBaseline,
    double angularStdDevBaseline,
    double singleCameraStdDevMultiplier,
    int processingIntervalLoops) {

  public static final VisionFilterConfig DEFAULTS = fromConstants(); // one literal per parameter, in VisionConstants

  public TestParams params(Test t) { return tests.get(t); }
  public boolean accepts(double score) { return score >= minScore; } // false for NaN

  public VisionFilterConfig with(Test t, UnaryOperator<TestParams> f) { ... }
  public Map<String, Double> toMap() { ... }          // "test.pitchError.steepness" -> 1.0
  public static VisionFilterConfig fromMap(Map<String, Double> m) { ... } // unknown key -> exception
}
```

### Scoring checks

```java
public enum Test {
  unambiguous(Kind.SIGMOID) {
    double metric(TestContext ctx) { return ctx.observation().ambiguity(); }
  },
  pitchError(Kind.SIGMOID) {
    double metric(TestContext ctx) { return Math.abs(ctx.observation().pose().getRotation().getY()); }
  },
  withinBoundaries(Kind.GATE) {
    double metric(TestContext ctx) { return ctx.insetDistanceMeters(); } // negative outside
  },
  ...;

  public final double test(TestContext ctx) {
    var p = ctx.config().params(this);
    double x = metric(ctx);
    return switch (kind) {
      case SIGMOID -> 1.0 - normalizedSigmoid(x, p.midpoint(), p.steepness());
      case GATE -> x >= 0 ? 1.0 : 0.0;
    };
  }
}
```

### Shared processing pipeline

```java
public final class VisionPipeline {
  public record Cycle(ScoredRow[] scored, FusedRow[] fused, boolean batchProcessed) {}

  public Cycle step(PoseObservation[][] perCamera, Rotation2d headingRef, VisionFilterConfig cfg) {
    loopCounter++;
    for (int cam = 0; cam < perCamera.length; cam++)
      for (var obs : perCamera[cam])
        buffer.add(filter.scoreObservation(obs, cam, lastAcceptedPose[cam], lastAcceptedTs[cam], headingRef, cfg));
    if (loopCounter % cfg.processingIntervalLoops() != 0) return new Cycle(rows(buffer), NONE, false);
    buffer.removeIf(o -> !cfg.accepts(o.score()));
    updateHistory(buffer);                       // the writes that were lost in the March merge
    var fused = filter.fuseCorrelatedObservations(buffer, cfg);
    fused.sort(byTimestamp);
    var out = fused.stream().map(f -> new FusedRow(f, filter.stdDevs(f, cfg))).toArray(FusedRow[]::new);
    buffer.clear();
    return new Cycle(rows, out, true);
  }
}
```

### Configuration source

```java
public final class VisionConfigSource {
  private final VisionFilterConfig.Inputs logged = new VisionFilterConfig.Inputs(); // LoggableInputs
  private final TunableVisionConfig tunables;      // 46 LoggedNetworkNumbers, or null in REPLAY
  private final Optional<VisionFilterConfig> replayOverride; // from VISION_CONFIG_OVERRIDE

  public VisionFilterConfig current() {
    if (!Logger.hasReplaySource()) {
      boolean live = tunables.enabled() && !DriverStation.isFMSAttached();
      logged.set(live ? tunables.validated(lastValid) : VisionFilterConfig.DEFAULTS, live ? Source.NETWORK : Source.DEFAULTS);
    }
    Logger.processInputs("Vision/Config", logged);   // records on the robot, restores in replay
    var cfg = logged.config();
    if (replayOverride.isPresent()) { cfg = replayOverride.get(); Logger.recordOutput("Vision/ConfigOverride", cfg.toMap()); }
    Logger.recordOutput("Vision/ConfigSource", logged.source());
    return cfg;
  }
}
```

### Reference score test

```java
@org.junit.jupiter.api.Test
void goldenScoresUnchanged() {
  var f = new VisionFilter();
  assertEquals(0.7123087581948762, f.scoreObservation(typicalSingleTag(), 0, null, 0, null, VisionFilterConfig.DEFAULTS).score(), 1e-12);
  assertEquals(0.6270, f.scoreObservation(ambiguous04(), 0, null, 0, null, VisionFilterConfig.DEFAULTS).score(), 5e-5);
  assertTrue(VisionFilterConfig.DEFAULTS.ceilingSingleTag() > VisionFilterConfig.DEFAULTS.minScore() + 0.05);
}
```
