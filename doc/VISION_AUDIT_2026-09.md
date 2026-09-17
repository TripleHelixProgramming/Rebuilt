# Vision System Audit, September 2026

## TL;DR

The vision filter keeps the robot on the field, but only two of its seven
tests do anything: the rest are flat or never fail, so it is a 5 m distance
gate with an ambiguity term, and the score it sends the pose estimator
carries no information. The logs cannot show this, because per-test scores
are discarded, half the frames are never logged, and no parameter value is
recorded. Fix the logging first (about a day), then put every tunable in one
logged config record with an offline re-scoring harness, and only then
re-tune the curves and the estimator's trust.

## Contents

- [Summary](#summary)
- [Part 1. How the filter behaves as built](#part-1-how-the-filter-behaves-as-built)
- [Part 2. Defects and improvements, ranked](#part-2-defects-and-improvements-ranked)
- [Part 3. Exposing the tunables](#part-3-exposing-the-tunables)
- [Appendix A. All confirmed findings, with remediation](#appendix-a-all-confirmed-findings-with-remediation)
- [Appendix B. Refuted findings](#appendix-b-refuted-findings)
- [Appendix C. What was checked and found sound](#appendix-c-what-was-checked-and-found-sound)

---

## Summary

This audit examined the vision pose-filtering system as it stood at commit
f3ca52c, with two aims: to describe how the system actually behaves, since
several of its mechanisms do not do what the code and comments suggest, and
to recommend an architecture for exposing the filter's tunables so that
students can experiment with weights, thresholds, and curves without editing
constants, rebuilding, and redeploying. Part 1 gives the description, Part 2
ranks the defects and improvements, and Part 3 gives the recommendation and
a phased plan. Readers who want the background on scoring, calibration, and
replay should start with `VISION_GUIDE.md`, which this document assumes.

The audit was carried out on 2026-09-17 by reading the code and the library
sources it depends on (wpimath 2026.2.1, AdvantageKit 26.0.2, PhotonLib
2026.3.4), by decoding the VACHE match logs (Q10, Q27, Q54, E4, E8) and the
VAALE E4 and E6 logs, and by replaying the E8 log through instrumented builds.
Each finding was written by one reviewer, then checked by a second reviewer
whose brief was to refute it, and by a third who assessed its practical
effect; severities below are the corrected ones. Of 78 candidate findings, 76
survived and 2 were refuted (Appendix B). Numbers were recomputed from the
code's constants; where a figure comes from a log, the log is named.

The filter is described, in its code and in the guide, as a seven-test
quality score with a threshold. In the data it behaves as a distance gate with
a weak ambiguity term. Three of the seven tests (pitch, roll, height) use
sigmoids so shallow that they score about 0.52 for every physically possible
pose and cannot distinguish a level robot from one tilted 45 degrees; two
(tag count, field boundary) always return exactly 1.0 or 0.0 and so contribute
nothing except to inflate the root; the remaining two (ambiguity, distance)
decide everything. Because the inert tests cap the achievable score at 0.734
for a single-tag observation and the threshold is 0.65, every accepted
single-camera observation scores between 0.65 and 0.73, so the score carries
almost no information into the standard deviation it is supposed to set.
Every one of the 178,000 observations in the 14 VACHE match logs was
single-tag; the multi-tag path has never run on the robot.

Downstream, the standard deviations sent to WPILib's pose estimator give a
Kalman gain of about 0.52 on position and 0.27 on heading for every accepted
single-camera observation, and 0.83 and 0.63 for a fused one. The estimator
is therefore a one- or two-sample low-pass of raw single-tag PnP, with no
innovation gate; one accepted observation moved a stationary robot 46 cm and
3.6 degrees in Q27, and the movement is predicted from the gain to within 3 mm.

The logs cannot show any of this directly. Per-test scores are computed and
discarded; the two summary keys record only the last fused observation of each
100 ms batch; camera inputs are logged on alternate loops, so a replay sees
half the observations the robot scored; and no parameter value is recorded, so
a log cannot say which constants produced it. The background camera thread
hands frames to the main loop through a mailbox with no sequence number, so a
frame can be scored twice (3.6% of frames in Q27) or dropped, and nothing
rejects a frame whose timestamp is 40 seconds in the future.

None of this is visible in the unit tests. They check orderings, not values,
so flat curves and a threshold above the ceiling pass; one test has failed on
every build since March, hidden by `ignoreFailures`; and the tests use the
2024 field dimensions.

The tunables architecture recommended in Part 3 therefore begins with the
logging and ingestion fixes, since a student's first experiment would
otherwise be fitting artefacts. The design itself is a single immutable
configuration record, the parameters in effect logged every loop as an
AdvantageKit input, NetworkTables overrides gated by the FMS, and an offline
harness that re-scores a match log through the real Java filter in seconds.

---

## Part 1. How the filter behaves as built

### 1. Only two tests do any work

The five continuous tests share `normalizedSigmoid(x, midpoint, steepness)`,
in which the steepness has the units of 1/x and is passed as a bare literal.
For pitch and roll the steepness is 1.0 per radian at a midpoint of 0.087 rad,
so the score falls from 0.73 to 0.27 over 2 radians (115 degrees): a level
robot scores 0.522, a robot pitched 45 degrees scores 0.332, and the three
tilt and height tests together move a typical total by hundredths. Commit
decba74 tightened the midpoints from 30 degrees and 0.75 m to 5 degrees and
0.25 m without touching the steepness, which made the tightening a near
no-op. In the E8 log the accepted and rejected observations have the same
pitch distribution (medians 1.5 and 1.8 degrees). The distance test is the
one well-shaped curve (0.98 at 0 m, 0.5 at 4 m, 0.02 at 8 m); the ambiguity
test is shaped moderately (0.646 at zero ambiguity, 0.269 at 0.4).

The two binary tests never fail on the robot: `VisionIOPhotonVision` emits an
observation only when a target exists and hard-codes `tagCount = 1`, so
`moreThanZeroTags` is constant, and `withinBoundaries` is 1.0 for every
observation that survives it. A passing gate contributes 1.0 to the product
and its weight to the root, so the two gates account for 2.0 of the 5.4 total
weight and lift every score: the soft-test-only geometric mean of the typical
observation is 0.583, which the gates raise to 0.712. The threshold of 0.65
is therefore really 0.505 on the soft tests, and doubling a gate's weight is
equivalent to lowering the threshold to 0.600.

Solving the mean for the two tests that vary, with the other five at their
plateaus, acceptance requires `unambiguous^0.8 * distance^0.5 >= 0.363`. An
unambiguous single-tag observation is rejected beyond 5.0 m (4.4 m with
realistic tilt); at 4 m the ambiguity must be below 0.22, at 1 m below 0.37;
and a multi-tag observation, were one ever to occur, would be rejected beyond
5.9 m although it is the most reliable kind at range. In Q27, E8, E4, and Q54
the distance test rejects more observations than the ambiguity test does
(would-flip-if-perfect: 32%, 6%, 12%, and 10% of all observations for
distance against 9%, 4%, 4%, and 14% for ambiguity), and the two together
account for essentially every rejection.

### 2. The score has no dynamic range

With pitch, roll, and height at 0.52 to 0.56 and ambiguity at most 0.646, the
best possible single-tag score is 0.734 and the best multi-tag score 0.783.
The threshold of 0.65 sits 0.084 below the single-tag ceiling, so every
accepted single-camera observation lies in [0.65, 0.734]; in E8 the
single-camera maximum was 0.7317 and the median 0.711. Fused observations are
boosted by 1.4x into [0.91, 1.0], so the accepted distribution is bimodal.
The comment on `MIN_SCORE` reasons about a "typical good single-tag
observation (~0.75 base)" and "multi-tag observations (0.9+)", neither of
which is reachable. A threshold of 0.735 would reject every single-tag
observation the robot produces, and the unit tests would not notice.

### 3. The standard deviation carries no information

The standard deviation sent to the estimator is `baseline * 3 / score` for a
single camera and `baseline / score` for a fused observation. Over the
reachable single-camera range that is 0.082 to 0.092 m, a 13% spread, and it
is the same for a tag at 1 m and a tag at 5 m, although single-tag PnP
translation error grows roughly with the square of distance. The only lever
the estimator sees is the 4.2x step between single and fused. The mapping is
three inline lines in `Vision.periodic`, is reached by none of the 86 tests,
and the values sent are never logged. `MAX_STD_DEV` is defined and unused.

### 4. The estimator applies every measurement about half way

`Drive` constructs `SwerveDrivePoseEstimator` with WPILib's default state
standard deviations (0.1 m, 0.1 m, 0.1 rad). The estimator's gain is
`q / (q + sqrt(q r))` per axis, which with the values above gives 0.52 on x
and y and 0.27 on heading for every accepted single-camera observation, and
0.83 and 0.63 for a fused one, applied with no gate on the innovation. The
Q27 log has the robot stationary in the disabled gap between auto and teleop
at (7.34, 6.39, 123 degrees); a rear-camera observation at (7.08, 7.24, 110
degrees) with ambiguity exactly 0.0, z = -0.23 m, and a tag 4.5 m away scored
0.660 and was accepted, and the estimate moved (-0.14, +0.44, -3.6 degrees).
The gain predicts (-0.14, +0.45, -3.5 degrees). Each camera delivers two or
three frames per 100 ms batch and each is applied separately, so a lone
camera's cumulative pull per batch is 0.77 to 0.89, larger than the 0.83 of
the single fused measurement that two agreeing cameras are collapsed into.
The heading channel is the worst case: single-tag PnP yaw, which is exactly
what the mirror ambiguity corrupts, overrides a good gyro at 27% to 63% per
observation.

### 5. The near-wall veto and the pre-match blindness

`withinBoundaries` rejects any pose outside the field inset by half the robot
width (0.468 m), built once in a static initializer. A robot pressed against
a wall has its centre at exactly the inset, so a few centimetres of noise
toward the wall vetoes the reading regardless of the other six tests. In E4,
from 148 to 168 s, the estimator sat 3.6 to 5.4 m off the field and all 414
observations were rejected; 205 of them had y between 0.40 and 0.45 m, |z|
below 0.07 m, tilt below 3 degrees, ambiguity below 0.03, and a tag 1.3 to
1.7 m away, the best observations of the match, and would have scored 0.667
to 0.674 without the veto. Vision could not correct the auto pose for 20 s
because of this test alone.

Before both E4 and E8 the robot sat at its start position for about 110 s
with tags in view, and every one of the 1,722 and 1,895 observations was
rejected, so the estimator entered auto at the origin. The rejections were
correct: the visible tags were 5.8 to 7.3 m away, the observations had
ambiguity 0.5 to 0.6, and the cameras disagreed with each other by 6 to 8 m,
so most were mirror solutions. What is missing is any mechanism for the
disabled period: a stationary robot with hundreds of frames per camera could
establish its pose by consensus across frames and cameras, but the filter
scores each frame alone. On current main the autos reset odometry at start,
so the consequence is now limited to the disabled period, no-auto matches,
and the first seconds of auto; at VACHE the VACHE-era `setPose` made it a
full-match problem.

### 6. Fusion

Two cameras' observations are merged when their positions lie within 0.15 m
and their timestamps within 0.15 s. Heading is never compared, and the fused
heading is a score-weighted circular mean, so two cameras agreeing on
position but disagreeing on yaw by 90 degrees produce a heading of 45 degrees
that neither reported, boosted to a score of 0.91 or more and sent with an
angular standard deviation of 0.066 rad (gain 0.60). None of the 18 fusion
tests uses a non-zero heading. The 1.4x boost uses the best member's score,
so a 0.72 and a 0.65 reading are treated as perfect, and it adds no
discrimination over the reachable range. Batches are cut every fifth loop
rather than by time, so about a fifth of the observations that have an
agreeing partner within the window fall on opposite sides of a boundary and
are sent as single-camera with 3x standard deviation; and the position gate
compares field-frame translations captured up to 150 ms apart, so at 2 m/s
the effective window is 75 ms and fusion switches off while driving. A camera
whose extrinsics have shifted never disagrees enough to be down-weighted: its
observations pass through as single-camera at full trust, and no per-camera
residual is tracked.

### 7. What the logs cannot tell you

Per-test scores are computed into an `EnumMap` and discarded.
`Vision/Summary/ObservationScore` and `FusedCameraCount` are recorded inside
the loop over fused observations, and AdvantageKit keeps one value per key
per loop and writes only changes, so a batch that sends four measurements
logs one score. Every earlier analysis that counted those records was
counting batches. `RobotPosesRejected` is a bare `Pose3d[]` with no camera
index, timestamp, or score, so a rejected pose seen in AdvantageScope cannot
be joined to its input row; and because the score key is written only for
accepted observations, the histogram the guide tells students to read is
truncated at the threshold by construction. Inputs are logged on even loops
only (`LOGGING_DIVISOR = 2`) while scoring runs every loop, so the log holds
half of what the robot scored and a replay of the current code sees half the
observations (the E4 replay, built with divisor 1 against a divisor-2 log,
scored every observation twice instead). No parameter value is recorded
anywhere; both VACHE builds were dirty, so the exact filter that produced the
match data is unrecoverable. Whether the estimator applied a measurement is
never known: it silently drops observations older than 1.5 s and discards
every later-stamped update when an earlier one arrives.

### 8. Ingestion

`VisionThread` replaces a single snapshot reference every 20 ms on a Notifier
and `Vision.periodic` copies whatever is there every 20 ms on the main loop,
with no generation counter. Two main-loop reads between two Notifier ticks
score, fuse, and send the same frames twice (3.6% of front-right frames in
Q27, which was logged every loop; a duplicate re-applies the same correction
and raises the pull from 0.52 to 0.77); two Notifier ticks between reads lose
a frame forever, since `getAllUnreadResults` has already consumed it. Any
exception from PhotonLib inside the Notifier ends the single thread for all
four cameras and leaves a frozen snapshot that still reports connected.

Nothing checks a capture timestamp against the FPGA clock. After an NT
reconnect the coprocessor's time-sync client restarts from an offset of zero,
and for about a second it publishes raw coprocessor time; in Q10 the
front-left and rear-left coprocessor was 38 s ahead, and 167 observations
were stamped 38 to 52 s in the future within a second of each reconnect. The
estimator clamps such a timestamp to "now", stores the update under its
future key, and then deletes it at the next correctly stamped measurement, so
the estimate alternates between cameras at 10 Hz for the duration.

The single-tag branch takes `result.targets.get(0)`, which the AprilTag
pipeline does not sort, discards every other tag in the frame, and logs only
that tag's ID; the alternate PnP solution is discarded in the same line,
although for a planar tag the two solutions differ mostly in yaw and the
standard remedy is to keep the one that agrees with the heading at capture.
With ambiguity above 0.38 no single-tag observation can pass the threshold,
which discards 29% to 42% of observations per match outright. The
`PoseObservation` record carries no tag ID, alternate pose, target area,
latency, or sequence number, so none of these policies can be evaluated
against existing logs. Finally, `ambiguity` is passed to the sigmoid with no
range check: PhotonVision's -1 "invalid" sentinel scores 0.99, and exactly
0.0, which in Q27 marked the worst accepted observation, scores as the best
possible value.

### 9. Replay and tests

Replaying a log re-simulates the whole robot program with the current build,
so `ReplayOutputs/Drive/Pose` reflects every code change since the log was
recorded, not only the filter's; replaying E8 through main produced a 13 m
divergence at auto start from a `setPose` change while the per-observation
decisions were byte-identical. Entering replay requires editing
`Constants.java`, and logs from before June need `Drive.java` edited too,
because a missing input key defaults silently and the program dies 130 s
later in `Launcher.aim`. Nothing asserts that an unchanged filter replays
identically, although it does.

The 86 unit tests check orderings (a good observation outscores a bad one)
and not values: no test pins a curve's shape, a golden score, the ceiling, or
the threshold's margin below it. One test, "Rejects ambiguous PnP solution
with wrong yaw", has failed on every build that has run it, because it scores
with the default set from which `yawConsistency` was removed; `ignoreFailures
= true` prints the failure as a warning and reports `BUILD SUCCESSFUL`. The
test file hard-codes a 16.54 by 8.21 m field (2024) while the code uses
16.513 by 8.043 m (2026); the boundary tests pass only because none probes
the far edge. Thirteen tests exercise the two disabled tests without saying
so, and "Velocity check uses previous pose" passes only because its
"impossible" pose is off the field. `Vision.periodic` itself, where batching,
the gate, the sort, the standard deviations, and the first-pose seed live,
has no test at all. `Vision.enabledTests` is the same mutable `EnumSet`
object as `VisionFilter.DEFAULT_ENABLED_TESTS`, so following the comment
"remove from set to disable" would alter the tests' expectations too.

### 10. Things that are latent rather than broken

`velocityConsistency` is disabled. Its per-camera history arrays are
allocated and read but never written; the writes existed on the March branch
and were lost in the fusion merge, so enabling the test as it stands applies a
uniform factor of 0.930 to every score while catching nothing. `yawConsistency`
is disabled; its reference is the estimator's own heading sampled at scoring
time, 50 to 170 ms after capture, and that heading is itself moved by every
accepted vision yaw, so the test is circular and would blank vision on a
spinning robot (18 degrees of apparent error at 180 degrees per second). Both
tests are traps in any tuning interface until those defects are fixed. The
custom AprilTag layout's FMS check runs once at first load and is cached, so
a robot that saw a tag before the field connected keeps the custom layout for
the match. The first vision measurement while disabled seeds the estimator
with no check on score, camera count, or distance, and the seed stays armed
through auto, so on current main it would fire in the disabled gap between
auto and teleop. A batch larger than 32 observations, for example a reconnect
backlog, silently disables fusion. A NaN score fails both comparisons and is
sent to the estimator with a NaN standard deviation.

---

## Part 2. Defects and improvements, ranked

Severity is the refuters' corrected severity. Effort is the impact assessor's
estimate for a mentor familiar with the code. The identifiers are those used
in Appendix A.

| Rank | Finding | Severity | Effort | What to do |
|---|---|---|---|---|
| 1 | per-test-scores-never-logged, no-rejection-attribution-or-observation-key | high | small | Log one struct row per scored observation (camera, timestamp, every test score, total, accepted, weakest test) and one per fused measurement (pose, timestamp, score, cameras, std devs). Retire the two scalar summary keys. |
| 2 | logging-divisor-replay, replay-odd-loops-see-empty-live-snapshot | high | trivial | Log inputs every loop, measured with the existing profiling hooks. Do not start `VisionThread` in REPLAY. |
| 3 | no-parameter-values-in-log | medium | trivial | Record every filter parameter as an AdvantageKit input each loop (Part 3). Record the enabled set and a config hash in metadata. |
| 4 | snapshot-replace-dup-and-loss, notifier-exception-freezes-vision | medium | small | Replace the mailbox with a drained queue or a generation counter; log a sequence number per snapshot; catch exceptions per camera and age out a stale snapshot. |
| 5 | no-timestamp-sanity-guard, velocity-negative-dt-free-pass | medium | small | Drop and count observations stamped more than 20 ms in the future or 0.5 s in the past; log latency and time-sync health per observation. |
| 6 | nan-score-fails-open | medium | small | Define acceptance once as `score >= minScore` (false for NaN), store it on the scored observation, and use it for both logging and the gate; reject non-finite poses at ingestion. |
| 7 | flat-tilt-height-sigmoids, score-ceiling-and-narrow-dynamic-range, effective-bar-is-distance-cutoff, unambiguous-curve-too-permissive | high | small code, medium risk | Make the curves dimensionless (score 0.95 at zero error, 0.5 at tolerance, 0.05 at twice tolerance), then re-derive `MIN_SCORE` from the data using the offline lab, and pin the new ceilings in tests. Not before the lab exists. |
| 8 | kalman-gain-no-gate, stddev-mapping-carries-no-information, repeated-single-updates-outpull-fused | high | small, needs robot time | Send a large angular std dev for single-tag observations while the gyro is connected; scale linear std dev with distance squared and camera and frame count; add an innovation gate in `Drive.addVisionMeasurement` once the pose is initialized; log what is sent. |
| 9 | within-boundaries-hard-margin | high | small | Gate on the physical field plus a tolerance and add a soft inset term; make the margin a config value and rebuild the rectangle on change. |
| 10 | hard-vetoes-inside-weighted-mean, root-normalization-couples-bar-to-weight-set | medium | small | Evaluate gates before the mean and exclude their weights from the root, behind a flag until `MIN_SCORE` is re-derived (the soft-only equivalent of the current 0.65 is 0.505). |
| 11 | parameters-not-runtime-tunable, unit-typed-constants-have-final-derived-twins, enabled-tests-aliases-default-set, stale-constant-comments | high (tunability) | medium | The configuration record of Part 3. Collapse each unit-typed constant and its `_METERS`/`_RADIANS` twin into one default. Rewrite the constant comments. |
| 12 | last-accepted-arrays-never-written, gyro-yaw-sampled-at-scoring-time | medium | small | Write the history arrays after acceptance (or remove them); give `yawConsistency` a reference sampled at the observation timestamp from the raw gyro plus a fixed field offset. Keep both tests out of any tuning interface until then. |
| 13 | stale-failing-yaw-test-hidden-by-ignorefailures, tests-do-not-pin-curve-shape, test-file-hardcodes-2024-field, velocity-test-passes-for-wrong-reason, vision-periodic-untested | medium | small | Fix the yaw test's enabled set; turn `ignoreFailures` off for the vision package; add golden-score, curve-shape, ceiling-margin, and far-corner tests; use the code's field constants; extract the batch step so it can be tested. |
| 14 | no-acceptance-watchdog-alert, uninitialized-pose-consumers | medium | small | A true per-camera pass rate over a 2 s window, an alert when tags are seen but nothing is accepted for 5 s, and an alert while disabled with FMS attached and no pose. Log `Drive/PoseInitialized`. |
| 15 | alternate-pnp-pose-discarded, pose-observation-record-too-thin, single-tag-uses-arbitrary-first-target, multitag-never-fires-cause-unknown, ambiguity-sentinel-and-zero | medium | medium | Emit one observation per in-layout target with tag ID, area, alternate pose, latency, and sequence number; find out why multi-tag never fires (settings export, calibration, layout on the coprocessor); treat ambiguity -1 and exactly 0.0 as unknown. Version the record (Appendix A, poseobservation-record-layout). |
| 16 | translation-only-clustering-yaw-average, correlation-boost-saturates, batch-boundaries-split-correlated-pairs, batch-latency-and-fusion-window, miscalibrated-camera-passthrough | medium | small to medium | Add a yaw agreement test to clustering and inflate the fused angular std dev by the resultant length; replace the multiplicative boost with an information-based combination; cluster on a sliding time window; compensate motion with odometry; track a per-camera residual and alert. |
| 17 | first-pose-bootstrap-unvetted, seed-fires-in-fms-gap, prematch-100pct-rejection-no-init | medium | small | Seed only from a fused or multi-frame consensus observation, gated on `!poseInitialized`, never between auto and teleop under FMS; add a disabled-period consensus mode for far tags. |
| 18 | replay-tunables-need-override-path, missing-input-keys-silently-default-then-crash-elsewhere, replay-is-whole-robot-resimulation-not-filter-experiment, no-replay-fidelity-test, replay-outputs-have-no-provenance-or-label, no-offline-multi-config-rescoring-harness | medium | medium | Select replay mode by environment variable; alias the old module keys; a replay preflight that lists missing input keys; a fidelity test; and the offline lab of Part 3, which makes most full replays unnecessary. |
| 19 | apriltag-layout-fms-frozen-on-notifier, field-layout-variant-hardcoded | low | small | Re-evaluate the FMS check per cycle or force the official layout whenever FMS is attached; make the Andymark/Welded variant explicit. |
| 20 | test-name-collision-and-three-meanings-of-test, dead-api-and-unused-types, camera-pass-rate-hardcoded-misnamed, camera-alerts-by-index, sim-camera-model-does-not-match-real, latest-target-observation-stale, multitag-avg-distance-inconsistent | low | trivial | Rename `Test` to `Criterion`; delete `getTargetX`, the `MEGATAG` types, and the misnamed pass-rate filters; name cameras in alerts; build the sim camera from the real calibration. |

---

## Part 3. Exposing the tunables

### Requirements

The original question was whether an architecture exists that lets students
experiment with weights, thresholds, and curves without the edit, build, and
deploy cycle, perhaps by reading overrides from NetworkTables. Four candidate
designs were produced independently, each from a different premise
(AdvantageKit-native live tuning, a data-driven configuration object, an
offline-first experiment loop, and a student-experience-first dashboard), and
three judges scored them on replay safety, match safety, student ergonomics,
implementation cost, testability, and fit with the codebase. Two judges
chose the AdvantageKit-native design and one the offline-first design, and
all three named the same ideas to graft from the others. The recommendation
below is that merge. It has to satisfy six requirements:

1. Replay determinism. Every parameter in effect must be in the log as an
   input, so that a match recorded with tuning off still replays with the
   constants that ran, whatever the replaying tree's constants say.
2. Match safety. A leftover value from a practice session cannot affect a
   match, and with tuning off the arithmetic is identical to the current one.
3. One definition. Each parameter has one default, read by the unit tests,
   the offline lab, replay, and the live robot.
4. Observability. A student can see every test's score and the reason an
   observation was rejected.
5. The 86 tests keep passing, or need only a mechanical change.
6. CPU. The roboRIO runs the pipeline at 50 Hz with up to 40 observations per
   second; the design must be measured, not estimated, before an event.

### The recommendation

**One record.** All tunables move into an immutable `VisionFilterConfig`
record: for each test, `enabled`, `weight`, `midpoint` (SI units), and
`steepness` (per SI unit); and the globals `minScore`, `boundaryMargin`,
`velocityTimeout`, `velocityUncertainScore`, the three fusion parameters, the
three standard-deviation parameters, and `processingIntervalLoops`. Its
`DEFAULTS` are built once from the existing unit-typed constants in
`VisionConstants`, which remain the human-readable source of the defaults and
lose their `_METERS`/`_RADIANS` twins. Every key has one flat string name
(`test.pitchError.steepness`, `minScore`, `fusion.timeWindowSeconds`) that is
used unchanged as the NetworkTables topic, the AdvantageKit input key, and
the property name in a file.

**One pipeline.** The control flow of `Vision.periodic` (score, buffer, gate,
fuse every N loops, compute standard deviations, update history) is extracted
into `VisionPipeline.step(observations, headingReference, config)`, which is
the single implementation used by the robot, by replay, by the unit tests,
and by the offline lab. `VisionFilter` keeps its `Test` enum, but each test
reads its midpoint, steepness, and weight from the context's config instead
of from static finals, and `scoreObservation` and `fuseCorrelatedObservations`
gain overloads that take the config; the existing signatures delegate with
`DEFAULTS`, so the current tests compile unchanged.

**Parameters as inputs.** Once per loop, before scoring, the effective config
is written with `Logger.processInputs("Vision/Config", ...)`. On the robot
that records it (the writer stores a value only when it changes, so the disk
cost is one burst at start and one per edit); in replay the same call
restores it from the log, so the pipeline runs with the parameters that were
in effect, whether tuning was on or off. A replay override file, applied
after the restore and recorded under `ReplayOutputs/Vision/ConfigOverride`,
is the only way parameters differ in replay, and the output log is suffixed
with the override's name so experiments do not overwrite each other.

**NetworkTables overrides.** On the robot each key is a `LoggedNetworkNumber`
under `/Tuning/Vision/<key>`, editable from AdvantageScope's tuning view or
an Elastic tab. They take effect only while `/Tuning/Vision/Enable` is true
and `DriverStation.isFMSAttached()` is false, both checked every loop. Values
are validated before use: non-finite values revert to the default, knobs are
clamped to declared ranges, negative weights become zero, a zero enabled
weight sum reverts the whole record (this is the NaN trap), and the two gate
tests cannot be disabled from NT. An invalid edit keeps the last valid config
and raises an alert naming the key. The snapshot is rebuilt only when the
observation buffer is empty, so a batch is never scored with one config and
fused with another; an edit therefore takes effect within 100 ms.

**Match safety.** Under FMS the tunables are inert and `Vision/ConfigSource`
logs `defaults`, so a competition log proves what ran. At every robot-code
start each knob is written back to its default with `set`, not
`setDefault`, because `LoggedNetworkNumber`'s constructor keeps a value a
reconnecting dashboard has republished; an alert shows while any knob differs
from its default; and the effective config's hash goes into the log metadata.
The layer is compiled into every build. The alternative of a compile-time
`TUNING_MODE` flag, which one design proposed and one judge preferred, is
stronger in principle but costs two deploys per practice session, which is
the step most likely to be skipped, and it makes replay depend on which build
recorded the log. The FMS gate, the boot reset, and the alert are the
recommended protection; a `MatchType != None` lock can be added as a second
gate.

**Observability.** Every loop logs `Vision/Scored`, one struct row per
observation: camera, timestamp, all nine test scores (NaN when disabled), the
total, the accepted flag, and the weakest test, defined precisely as the
enabled test with the most negative `weight * ln(score)`, which for a failed
gate is -infinity. Every batch logs `Vision/Fused`: pose, timestamp, score,
camera count, and the two standard deviations sent. `Vision/YawReference`
records what `yawConsistency` compared against. These replace the summary
scalars. Steepness is exposed on the dashboard as a width in display units
(the tested quantity between scores 0.73 and 0.27; 115 degrees for
pitch), so the flat-curve defect is visible on the slider, while the code
keeps steepness per SI unit.

**The offline lab.** A JUnit test tagged `lab`, run by a dedicated
`./gradlew visionLab` task that GradleRIO configures with the desktop HAL
natives, reads a `.wpilog` with WPILib's `DataLogReader`, decodes the
`PoseObservation[]` arrays through AdvantageKit's own `LogTable` path (so
there is no second decoder), reconstructs the cycles, and runs them through
`VisionPipeline` with one or more configs given as property files. It prints
accepted and rejected counts per camera and match phase, a score histogram
with the threshold drawn, the jump count, a blame table, the observations
whose decision flipped between two configs, and a sweep over a parameter
grid; it runs a whole directory of logs so that a change is judged on all
14 matches and not one. A VACHE match holds about 12,700 observations and
scores in about 25 ms, so a 200-point sweep finishes in seconds. This
replaces Part 6 of the guide (the throwaway clone, the two source edits, and
the Python comparison script) for filter experiments; full AdvantageKit
replay remains the tool for questions that involve the estimator or the
autos.

### Precedence

Highest first. Each source is logged so a reviewer can tell which applied.

1. Replay override file (replay only; `VISION_CONFIG_OVERRIDE=<file>`, an
   environment variable, since Gradle does not forward `-D` properties to the
   forked simulation JVM; the working precedent is `AKIT_LOG_PATH`).
2. Logged `Vision/Config` inputs (replay only).
3. NetworkTables values, when `Enable` is true and FMS is not attached (robot
   and simulation only).
4. `DEFAULTS`.

### Code sketches

The configuration record and per-test parameters:

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

The test shape, with the gate distinction explicit and the parameters read
from the context:

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

The pipeline step, which is what the robot, replay, the tests, and the lab
all call:

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

The configuration source and its precedence:

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

A golden test, written before any refactor so that the tuning-off path is
proven identical:

```java
@org.junit.jupiter.api.Test
void goldenScoresUnchanged() {
  var f = new VisionFilter();
  assertEquals(0.7123087581948762, f.scoreObservation(typicalSingleTag(), 0, null, 0, null, VisionFilterConfig.DEFAULTS).score(), 1e-12);
  assertEquals(0.6270, f.scoreObservation(ambiguous04(), 0, null, 0, null, VisionFilterConfig.DEFAULTS).score(), 5e-5);
  assertTrue(VisionFilterConfig.DEFAULTS.ceilingSingleTag() > VisionFilterConfig.DEFAULTS.minScore() + 0.05);
}
```

### Phased plan

Each phase is independently shippable and ends with something a student can
newly do.

| Phase | Goal | Changes | New for a student |
|---|---|---|---|
| 0. Prerequisites | Make the data trustworthy | Log inputs every loop; queue or sequence-number the snapshot handoff and catch Notifier exceptions; timestamp sanity gate; fail-closed acceptance; write the history arrays; fix the yaw test and turn `ignoreFailures` off for the vision package; use the code's field constants in tests; golden-score tests | A log that holds what the robot scored; a green test suite that fails when broken |
| 1. One definition | The config record and the pipeline | `VisionFilterConfig`, `TestParams`, `Kind`; tests read the context's config; overloads; `VisionPipeline` extracted; `Vision.periodic` reduced to copy, process inputs, step, send, log | A weight or steepness can be changed in a unit-test playground with one line and no constants edited |
| 2. Logging layout | Observability | `Vision/Scored[]`, `Vision/Fused[]`, `Vision/Config` inputs, `Vision/YawReference`, `Vision/ConfigSource`; retire the summary scalars; rewrite the constant comments | Every rejection in a log names its weakest test; a log records its own parameters |
| 3. The lab | Offline experiments | `visionLab` Gradle task, log reader, metrics, sweep, A/B report, directory mode, a five-second fixture log and smoke test | Re-score a match with a new config in seconds; sweep a grid; compare two configs on all 14 matches |
| 4. Live tuning | Practice-field loop | `TunableVisionConfig`, `Enable`, FMS gate, boot reset, alerts, config hash in metadata, committed Elastic layout with widths in degrees, per-camera last-observation panel | Edit a knob on the practice field and watch the per-test scores respond; replay the session that evening |
| 5. Scoring redesign | Fix the filter itself | Dimensionless curves; gates outside the mean; distance decoupled from the gate and driving the std dev; angular std dev large for single-tag; innovation gate; alternate-pose disambiguation; consensus seeding while disabled; all evaluated through the lab before deployment | A score that means something, and a threshold derived from data |

Phases 0 to 2 are roughly two mentor-days; 3 and 4 one day each; 5 is a
season-long programme of experiments, which is the point.

### Rejected alternatives

A compile-time tuning flag was rejected for the reasons above. Reading
overrides from WPILib `Preferences`, which the modules already use for
turn-zero offsets, was rejected because Preferences are persistent and are
not AdvantageKit inputs, so a leftover value would survive a reboot and a
replay could not see it. A JSON file in the deploy directory as a live source
on the robot was rejected because it is the one path by which a file rather
than code could change a match; it survives as the replay override only.
Exposing only the raw NetworkTables values without logging the effective
config as an input was rejected because a competition log, recorded with
tuning off, would then replay with whatever constants the replaying tree
had. A Python re-implementation of the filter for offline work was rejected
on the team's standing rule; the lab runs the Java filter.

### Open questions

These need a robot, a mentor decision, or both.

1. CPU. Every-loop logging of `PoseObservation[]` and the per-observation
   struct rows are estimated at under 0.5 ms per loop on the roboRIO 2 but
   have not been measured; the `PROFILING_ENABLED` timers exist for this.
2. Why multi-tag never fires. It requires the PhotonVision multi-target
   toggle, a calibration at the running resolution, and two tags of the
   uploaded layout in one frame. The settings export is not in the repository.
3. Whether the front cameras should really sit behind the robot centre. The
   transforms are internally consistent (the front-left and rear-left
   cameras agree to 2 cm in Q27), but the names suggest otherwise.
4. The angular trust. Sending a very large angular standard deviation for
   single-tag observations hands heading to the gyro; whether the gyro's
   drift over a match is small enough for that needs a measurement.
5. Ground truth. Every metric the lab can print is self-consistency. A
   marked-spot procedure on the practice field, scored against the estimate,
   is the only way to know whether a change made the pose more accurate
   rather than merely more confident.
6. The yaw reference. A field-relative heading from the raw gyro plus a
   fixed offset captured at `setPose` removes the circularity, but the offset
   must be checked against the selected auto before each match.

---

## Appendix A. All confirmed findings, with remediation

One entry per finding, grouped by area: the location, the severity after the
refutation check, the defect in a sentence, and the remediation, incorporating
the refuter's corrections. Identifiers match the audit's raw output.

### Scoring

- **flat-tilt-height-sigmoids** (`VisionFilter.java:156`, high). Steepness
  1.0 per radian or per meter makes the pitch, roll, and height curves flat
  over the physical range. Fix: define every curve dimensionlessly,
  `1 - logistic(k * (x / tolerance - 1))` with `k = ln 19`, so a test scores
  0.95 at zero error, 0.5 at tolerance, and 0.05 at twice tolerance; then
  re-derive `MIN_SCORE` and pin the shape in tests.
- **score-ceiling-and-narrow-dynamic-range** (`VisionConstants.java:145`,
  high). Ceilings of 0.734 and 0.783 confine accepted single-camera scores to
  [0.65, 0.734]. Fix: follows from the curve fix; add a test that the
  single-tag ceiling at operating distance exceeds `MIN_SCORE` by a margin,
  and log the ceiling as a derived value.
- **effective-bar-is-distance-cutoff** (`VisionFilter.java:323`, high). With
  five tests at their plateaus the threshold reduces to a 5 m distance
  cutoff. Fix: after the curves, re-derive `MIN_SCORE` from stated per-test
  semantics; let distance set the standard deviation rather than the gate,
  with a separate, larger hard maximum; do not gate multi-tag by the
  single-tag distance curve.
- **hard-vetoes-inside-weighted-mean** (`VisionFilter.java:189`, medium). The
  two gate tests contribute only to the root and `moreThanZeroTags` never
  fails. Fix: evaluate gates first and short-circuit with a rejection reason;
  run the mean over the soft tests only; retune `MIN_SCORE` to that scale
  (0.505 is the current equivalent). Note that a gate weight of 0 silently
  disables it, since `0^0 = 1`.
- **root-normalization-couples-bar-to-weight-set** (`VisionFilter.java:314`,
  medium). `MIN_SCORE` changes meaning whenever the enabled set or a weight
  changes. Fix: keep the root (the refuter showed that thresholding the raw
  product is worse); document that `MIN_SCORE` must be re-derived when the
  set changes, and provide a helper that prints the ceiling and the
  equivalent per-test bar.
- **unambiguous-curve-too-permissive** (`VisionFilter.java:143`, medium). The
  ambiguity curve plateaus at 0.646 and admits ambiguity up to 0.37 at short
  range. Fix: steepness about 19.6 per unit (0.95, 0.5, 0.05 at 0, 0.15,
  0.30); consider a hard gate above 0.25 to 0.3; prefer disambiguation with
  the alternate pose over a penalty.
- **ambiguity-sentinel-and-zero** (`VisionFilter.java:143`, medium).
  PhotonVision's -1 sentinel scores 0.99 and exactly 0.0 scores as the best
  value. Fix: treat ambiguity below 0 as unknown (0.5, or reject) and exactly
  0.0 as unknown unless corroborated; add tests for both; log ambiguity
  histograms per camera.
- **nan-score-fails-open** (`Vision.java:226`, medium). A NaN score fails
  both comparisons and reaches the estimator with a NaN standard deviation.
  Fix: define acceptance once as `score >= minScore` (false for NaN), store it
  on the scored observation, and use it for logging and the gate; reject
  non-finite pose components at ingestion; guard the consumer against
  non-finite standard deviations.
- **within-boundaries-hard-margin** (`VisionFilter.java:183`, high). A hard
  veto at a frozen 0.468 m inset rejected 205 near-perfect observations in
  E4. Fix: gate on the physical field plus about 0.3 m of tolerance and add a
  soft inset term inside the mean; make the inset and its steepness config
  values and rebuild the rectangle when they change; log the result per
  observation.

### Estimator trust

- **kalman-gain-no-gate** (`Vision.java:242`, high). Every accepted
  observation moves the estimate about half way with no innovation gate.
  Fix: send a very large angular standard deviation whenever the gyro is
  connected; derive the linear one from distance squared and tag count so
  the per-observation gain stays near 0.05 to 0.2; add an innovation gate in
  `Drive.addVisionMeasurement` once the pose is initialized (about 1 m and 30
  degrees, re-seeding after N consecutive rejections); log the standard
  deviations and gains sent.
- **stddev-mapping-carries-no-information** (`Vision.java:242`, high). The
  standard deviation is 0.082 to 0.092 m for every single-camera observation
  regardless of distance. Fix: move the mapping into
  `VisionFilter.stdDevsFor(fused, config)`, compute it from physical
  predictors independent of the gate score, unit-test it, log it per
  observation, and delete or use `MAX_STD_DEV`.
- **repeated-single-updates-outpull-fused** (`Vision.java:246`, medium). Two
  or three frames from one camera per batch out-pull one fused measurement.
  Fix: average a camera's observations within the window, or scale its
  standard deviation by the square root of its frame count in the batch, or
  send one measurement per camera per batch; log the pull per batch.
- **miscalibrated-camera-passthrough** (`VisionFilter.java:454`, medium). A
  camera that never agrees with the others passes through at full trust.
  Fix: keep a per-camera exponentially weighted residual against the fused or
  estimator pose at each observation's timestamp; scale that camera's
  standard deviation by it and alert above a threshold; keep the same
  statistic per tag ID to catch a damaged field tag.

### Fusion

- **translation-only-clustering-yaw-average** (`VisionFilter.java:384`,
  medium). Clustering ignores heading, and disagreeing headings average to
  one nobody reported. Fix: add a yaw agreement test (10 to 15 degrees) to
  the pairwise merge, or compute the resultant length R and refuse to fuse
  below about 0.9 or inflate the fused angular standard deviation by 1/R;
  expose both; add fusion tests with 45 and 180 degree disagreement; consider
  not fusing yaw at all while every observation is single-tag.
- **correlation-boost-saturates** (`VisionFilter.java:445`, medium). The
  1.4x boost adds no discrimination over the reachable range. Fix: replace it
  with an information-based combination (average per camera, then combine
  with `1 / sqrt(sum 1/sigma^2)`) or a score-weighted mean; if a boost is
  kept, make it additive in log-odds so it cannot saturate.
- **batch-boundaries-split-correlated-pairs** (`Vision.java:224`, medium).
  Loop-count batches split about a fifth of fusable pairs. Fix: cluster on a
  sliding window keyed by observation timestamp, keeping observations newer
  than the newest minus the window across batches and emitting clusters
  whose newest member has aged out.
- **batch-latency-and-fusion-window** (`VisionFilter.java:384`, medium).
  Field-frame translations captured up to 150 ms apart are compared directly,
  so fusion switches off while driving. Fix: project each observation to a
  common reference time with the odometry delta before clustering, or gate
  on vision minus odometry at each observation's own time; log the fusion
  rate against chassis speed.
- **transitive-chaining-exceeds-thresholds** (`VisionFilter.java:391`, low).
  Union-find chaining lets a cluster exceed both thresholds, and a unit test
  enshrines it. Fix: require agreement with the centroid or all members when
  joining, or bound the cluster diameter, and change the test.
- **max-observations-fusion-fallback** (`VisionFilter.java:347`, low). A
  batch over 32 observations silently disables fusion. Fix: log the batch
  size and a `fusionSkipped` flag; size the arrays dynamically or raise and
  expose the cap; drop results older than about 0.3 s at ingestion with a
  counted drop.
- **cross-batch-out-of-order-discards-updates** (`Vision.java:234`, low). An
  older-stamped measurement in the next batch makes the estimator discard
  newer updates. Fix: track the last timestamp sent and drop older
  measurements, or delay a batch by the expected latency spread; use
  `Double.compare` in the sort.

### Ingestion and the IO layer

- **snapshot-replace-dup-and-loss** (`VisionThread.java:42`, medium). The
  mailbox handoff scores frames twice or drops them. Fix: a queue drained by
  `periodic`, or a generation number on the snapshot with already-processed
  snapshots skipped; keep a last-processed timestamp per camera; log
  PhotonVision's `sequenceID` and counters for duplicates and skips.
- **notifier-exception-freezes-vision** (`VisionThread.java:148`, medium).
  One exception ends the thread for all four cameras and leaves a frozen
  snapshot reporting connected. Fix: catch per camera, log the throwable,
  mark the snapshot faulted; add a snapshot timestamp and treat one older
  than 0.2 s as disconnected.
- **single-lock-all-cameras-stall** (`VisionThread.java:145`, low). The
  refuter found the lock inert; the serialization is the single Notifier
  thread. Fix: record tick duration and inter-tick gap as inputs; consider one
  Notifier per camera; combine with the queue handoff so a late tick loses
  nothing.
- **no-timestamp-sanity-guard** (`VisionIOPhotonVision.java:137`, medium).
  Results stamped 38 to 52 s in the future after a coprocessor reconnect are
  scored and sent. Fix: at ingestion drop and count results stamped more than
  20 ms ahead or 0.5 s behind the FPGA clock, with `timeSinceLastPong` over 2
  s, or with latency outside 0 to 250 ms; log latency, publish time, sync
  health, and `sequenceID` per observation; log the estimator's own
  rejections from a consumer wrapper.
- **velocity-negative-dt-free-pass** (`VisionFilter.java:222`, low). A
  negative `dt` earns a perfect velocity score. Fix: return the uncertain
  score for `dt < 0`; the ingestion bound above makes the case unreachable.
- **alternate-pnp-pose-discarded** (`VisionIOPhotonVision.java:126`,
  medium). The alternate PnP solution is dropped, so ambiguity is used as a
  gate that discards 29% to 42% of observations per match. Fix: compute
  robot poses from both transforms; when a heading reference exists and the
  candidate yaws differ by more than a few degrees, keep the one closer to
  the heading at capture, set a flag, and keep both poses in the record so
  other policies can be replayed; then reduce the ambiguity weight and
  re-derive `MIN_SCORE`. Note that high-ambiguity observations are mostly
  5.7 to 7.2 m from the tag, so disambiguation alone does not make them
  acceptable.
- **single-tag-uses-arbitrary-first-target** (`VisionIOPhotonVision.java:117`,
  medium). The first target in an unsorted list is used and the rest are
  discarded. Fix: emit one observation per in-layout target, with tag ID,
  area, and ambiguity, and let the filter decide, or at least choose the
  largest or least ambiguous; add every in-layout ID to `tagIds`; log the
  targets per result.
- **multitag-never-fires-cause-unknown** (`VisionIOPhotonVision.java:81`,
  medium). Every observation in 14 matches was single-tag and the repository
  cannot say why. Fix: log `targets.size()` and all IDs per result; check the
  multi-target toggle, uploaded layout, and calibration on each coprocessor;
  commit the PhotonVision settings export; if tags are never co-visible, tune
  for single-tag only.
- **pose-observation-record-too-thin** (`VisionIO.java:39`, medium). The
  record carries no tag ID, alternate pose, area, latency, or sequence
  number. Fix: extend it with those fields and the time-sync health, under
  the versioning below. Per-tag exclusion and duplicate detection are already
  possible from `TagIds` and timestamps.
- **poseobservation-record-layout-is-an-unversioned-replay-contract**
  (`VisionIO.java:39`, medium). Changing the record breaks replay of every
  existing log. Fix: because `LogTable` looks up by field key and checks only
  the struct prefix, renaming the record class alone does not protect old
  logs; log the new record under a new key (`PoseObservationsV2`) with a
  decoder that reads the old key into the old record, or add new data as
  parallel arrays; add a CI test that replays an archived log fragment.
- **multitag-avg-distance-inconsistent** (`VisionIOPhotonVision.java:91`,
  low). The multi-tag average distance and tag count iterate over different
  sets. Fix: average over the targets in `fiducialIDsUsed`, or report
  `targets.size()` separately.
- **latest-target-observation-stale** (`VisionIOPhotonVision.java:66`, low).
  The field is only written when a result arrives. Fix: reset it at the top
  of `updateInputs` or add a timestamp; complete the comment.
- **apriltag-layout-fms-frozen-on-notifier** (`Vision.java:314`, low). The
  FMS check runs once at first load and is cached. Fix: load the layout
  eagerly in the constructor on the main thread and re-evaluate the choice
  when FMS attaches, or make it an explicit deploy-time choice recorded in
  metadata; derive the arena rectangle from the loaded layout's field size.
- **field-layout-variant-hardcoded** (`VisionConstants.java:26`, low). The
  Andymark variant is hard-coded and the choice is not logged; the welded
  layout differs by a common 1.9 cm shift plus about 2 cm for tags 13 to 16
  and 29 to 32. Fix: select the variant from a logged input or deploy setting,
  alert it at boot, and derive the arena rectangle from the loaded layout.

### Logging and replay

- **per-test-scores-never-logged** (`Vision.java:251`, high). Per-test
  results are discarded and the summary scalars keep one value per batch.
  Fix: per loop and per camera, one struct row per scored observation
  (camera, timestamp, each test's score, total, accepted, cluster id),
  rejected ones included; per batch, the fused scores, camera counts,
  timestamps, poses, and standard deviations as arrays; retire the two
  scalars.
- **no-rejection-attribution-or-observation-key** (`Vision.java:186`, high).
  Nothing says why an observation was rejected, and rejected poses cannot be
  joined to their inputs. Fix: log with each row the contribution
  `w * ln(s)` per test and the margin to the threshold, plus a sequence id
  assigned at ingestion; log every observation's score so the histogram
  spans the threshold; add per-cycle accepted and rejected counts.
- **logging-divisor-replay** (`Vision.java:137`, high). Inputs are logged on
  alternate loops while scoring runs every loop. Fix: log every loop and
  measure it with the profiling hooks, or score only what is logged; assert
  the divisor in REPLAY or read it from metadata; add a replay-drift check on
  distinct accepted counts; re-run prior analyses before trusting them.
- **replay-odd-loops-see-empty-live-snapshot** (`Vision.java:130`, medium).
  In REPLAY the live thread's empty snapshot overwrites the inputs on odd
  loops. Fix: do not start `VisionThread`, or do not copy its snapshot, in
  REPLAY; better, let the IO layer signal new frames with a per-camera
  sequence number and call `processInputs` every loop.
- **no-parameter-values-in-log** (`Robot.java:147`, medium). A log cannot say
  which constants produced it. Fix: record build-time defaults with
  `Logger.recordMetadata` at construction; once tunable, rely on the logged
  config inputs; record a config hash as an output so a mid-session change is
  visible.
- **accepted-not-applied-no-feedback** (`Vision.java:187`, medium). Whether
  the estimator applied a measurement is never known. Fix: in
  `Drive.addVisionMeasurement` log the measurement's age, whether its
  timestamp lies inside the odometry buffer, the estimate's delta, and the
  pose and standard deviations sent, per batch.
- **replay-tunables-need-override-path** (`Constants.java:25`, medium).
  Replay needs a source edit and any NT value is frozen. Fix: an override
  applied after the logged values are restored and recorded as an output;
  select the mode by environment variable.
- **replay-used-different-build** (`VisionConstants.java:155`, medium). The
  E4 replay was built with a different divisor and enabled set than the
  robot ran. Fix: replay with the SHA in `RealMetadata` or fail; commit
  before events; record the enabled set and every constant in the log.
- **missing-input-keys-silently-default-then-crash-elsewhere**
  (`Drive.java:101`, medium). A missing input key defaults silently and the
  program dies 130 s later. Fix: keep input keys stable or alias the old
  module keys; a replay preflight that asserts expected keys exist and lists
  the missing ones; initialize `chassisSpeeds` and alert when no odometry
  sample arrives for N loops; make the replay task exit non-zero on a crash.
- **no-replay-fidelity-test-and-batch-scalars-move-the-wrong-way**
  (`Vision.java:251`, medium). Nothing asserts that an unchanged filter
  replays identically, and the batch scalars move against the real counts.
  Fix: the struct-array logging above, plus a fidelity check over a bundled
  log fragment asserting replayed per-observation decisions equal the
  recorded ones when the parameters match.
- **replay-is-whole-robot-resimulation-not-filter-experiment**
  (`Drive.java:418`, medium). A non-vision code change produced a 13 m
  divergence while the filter's decisions were identical. Fix: define the
  experiment's outputs at the filter level and treat `Drive/Pose` as
  derived; log the measurements actually applied so a shadow estimator can be
  run on them.
- **replay-outputs-have-no-provenance-or-label** (`Robot.java:250`, low). The
  output overwrites the previous run and records nothing about the
  experiment. Fix: select REPLAY by environment variable; accept a label and
  an override file at startup; write `<log>_<label>.wpilog` to a separate
  directory; record the label, the override contents, and the resolved
  parameters in metadata; a Gradle task that fails on a program crash.
- **replay-doc-sample-count-claim-is-dedup-artifact** (`Vision.java:139`,
  low). The "50 to 85% fewer samples" claim was a writer-dedup artefact.
  Fix: compare decoded rows, never entry counts (applied in the guide).
- **allocation-cpu-profile** (`VisionFilter.java:309`, low). The real CPU
  cost is serialization, once measured at 12 ms, which is why the divisor
  exists. Fix: log the periodic's phase timings as outputs every loop (branch
  `fix/profiling-to-wpilog` already does) and measure divisor 1 on the
  roboRIO; if serialization is the cost, log a compact per-observation struct
  instead of `Pose3d` arrays.
- **no-offline-multi-config-rescoring-harness** (`VISION_GUIDE.md:1317`,
  medium). Comparing two settings on one match takes a clone, two edits, and
  a full replay each. Fix: the `visionLab` harness of Part 3, decoding
  through AdvantageKit's public `LogTable` path (`RecordStruct` is
  package-private), with a ten-second fixture log and a golden test.

### Drive, seeding, and alerts

- **first-pose-bootstrap-unvetted** (`Drive.java:429`, medium). The first
  measurement while disabled seeds the estimator with no quality check. Fix:
  seed only from an observation meeting a stricter bar (two cameras, or N
  consistent observations within 0.2 m) passed through a richer consumer
  record; log the seed with its score; allow a re-seed while disabled when
  vision disagrees by more than 1 m.
- **seed-fires-in-fms-gap** (`Drive.java:428`, medium). The seed stays armed
  through auto and would fire between auto and teleop. Fix: gate it on
  `!poseInitialized`, require multi-camera or multi-frame agreement, skip it
  while FMS is attached between auto and teleop, and log `Drive/PoseSeeded`.
- **prematch-100pct-rejection-no-init** (`VisionConstants.java:73`, medium).
  Every pre-match observation was rejected in both matches. Fix: a
  disabled-period mode that accumulates far observations across frames and
  cameras, rejects the mirror cluster by consensus, and seeds from the
  cluster mean. The refuter notes the pre-match observations at VACHE were
  mostly wrong and the cameras disagreed by 6 to 8 m, so the answer is
  consensus, not looser thresholds.
- **uninitialized-pose-consumers** (`Robot.java:401`, medium). Nothing
  indicates that the pose is uninitialized. Fix: log `Drive/PoseInitialized`;
  alert while disabled with FMS attached and no accepted vision for 5 s; give
  the LED pose-seek a distinct no-pose pattern.
- **no-acceptance-watchdog-alert** (`Vision.java:153`, medium). A filter that
  rejects everything raises no alarm. Fix: a true per-camera pass rate over a
  2 s window, a warning when tags are seen but nothing is accepted for 5 s,
  and an error at auto init when the pose is uninitialized with tags in view;
  unit-test the window.
- **camera-alerts-by-index** (`Vision.java:105`, low). Alerts name cameras by
  index. Fix: add `name()` to `VisionIO`, use it in alert text and fault
  keys, and log the name-to-index map at boot.

### Tunability and constants

- **parameters-not-runtime-tunable** (`VisionFilter.java:139`, high).
  Weights are enum literals, steepness values are bare literals, and there is
  no config seam. Fix: the `VisionFilterConfig` record of Part 3, passed into
  `scoreObservation` and `fuseCorrelatedObservations`; weight 0 disables a
  test; afterward an optional shadow filter whose decisions are only logged.
- **unit-typed-constants-have-final-derived-twins**
  (`VisionConstants.java:73`, medium). Mutable unit-typed tolerances are
  shadowed by final doubles. Fix: collapse each pair to one source, keeping
  the unit-typed value as the default and converting at the point of use, or
  storing the double in the config; make everything not tunable `final`.
- **enabled-tests-aliases-default-set** (`Vision.java:79`, medium).
  `Vision.enabledTests` is the same mutable set the tests use. Fix: make
  `DEFAULT_ENABLED_TESTS` unmodifiable and give the config its own copy, or
  fold enabling into a weight of 0.
- **last-accepted-arrays-never-written** (`Vision.java:178`, medium). The
  velocity history is allocated and read but never written. Fix: write the
  arrays after the gate and before fusion for each accepted observation per
  camera (the writes existed in d6de05e and were lost in the merge), or
  delete them; consider the estimator pose at the observation's timestamp as
  the reference instead, since "last accepted" lets a bad pose become the
  reference.
- **gyro-yaw-sampled-at-scoring-time** (`Vision.java:180`, medium). The yaw
  reference is sampled late and is itself vision-corrected. Fix: a reference
  that is a function of timestamp, preferably the raw gyro at that time plus
  a fixed field offset captured at `setPose`; skip the test until the offset
  exists.
- **stale-constant-comments** (`VisionConstants.java:107`, medium). The
  comments describe a threshold of 0.6, a 1.3x boost, a majority rule, and
  unreachable scores. Fix: rewrite them against the current values; delete
  `MAX_STD_DEV` or use it as a clamp; move numeric claims into generated
  tables or tests.
- **precedents-not-replay-safe** (`Module.java:46`, low). The existing
  runtime-config precedents are not AdvantageKit inputs. Fix: base tunables
  on `LoggedNetworkNumber`, not `Preferences`, which persist and are not
  inputs; `Module`'s use is replay-safe only because its effect is captured
  in logged hardware inputs.

### Tests, simulation, and documentation

- **stale-failing-yaw-test-hidden-by-ignorefailures**
  (`VisionFilterTest.java:608`, medium). One test has failed on every build
  that ran it. Fix: score with the default set plus `yawConsistency`; make
  `ignoreFailures` conditional on deploy; have tests take a config with
  defaults.
- **tests-do-not-pin-curve-shape** (`VisionFilterTest.java:154`, medium).
  The suite checks orderings only. Fix: shape tests for every smooth test
  (0.9 or more at zero error, 0.45 to 0.55 at tolerance, 0.1 or less at
  twice tolerance), golden values (0.712, 0.775, 0.627), a typical single-tag
  observation at least 0.1 above `MIN_SCORE`, an impossible one below it,
  and a ceiling-margin invariant.
- **vision-periodic-untested** (`Vision.java:125`, medium). The batch step
  has no test. Fix: extract it into a pure method and test it; inject
  `VisionThread` instead of using the singleton.
- **velocity-test-passes-for-wrong-reason** (`VisionFilterTest.java:714`,
  medium). The test passes only because its pose is off the field. Fix: an
  in-field impossible pose and an explicit enabled set including
  `velocityConsistency`; tag the dormant groups.
- **test-file-hardcodes-2024-field** (`VisionFilterTest.java:26`, medium).
  The tests use 16.54 by 8.21 m. Fix: use the code's field constants; add
  far-corner and 1 mm inside and outside edge tests.
- **test-name-collision-and-three-meanings-of-test**
  (`VisionFilter.java:138`, medium). `Test` collides with JUnit's `@Test`.
  Fix: rename to `Criterion`, `evaluate`, `ScoringContext`,
  `ScoredObservation`, and `DEFAULT_CRITERIA`; update the guide's vocabulary
  in the same change.
- **sim-camera-model-does-not-match-real** (`VisionIOPhotonVisionSim.java:26`,
  medium). The sim runs 35 fps, 30 ms, and multi-tag; the robot runs 24 fps,
  50 ms, and single-tag. Fix: build the camera properties from the exported
  calibration, 24 fps and 50 plus or minus 10 ms latency, a sight range of
  about 8 m; pass a reduced layout to suppress multi-tag, since the sim has
  no flag for it.
- **sim-smoke-run-rejects-everything** (`Drive.java:84`, low). The sim robot
  starts at the origin, outside the arena, and 6 to 8 m from any tag. Fix:
  start it within about 4.5 m of visible tags and assert acceptances within
  N seconds.
- **sim-pose-read-from-notifier-and-4x-update**
  (`VisionIOPhotonVisionSim.java:70`, low). The shared sim is updated four
  times per tick from the Notifier thread. Fix: update it once per tick from
  the main thread, hand the pose over through an `AtomicReference`, and
  disable the video streams unless debugging.
- **camera-pass-rate-hardcoded-misnamed** (`Vision.java:64`, low). Four
  hard-coded filters average scores, not pass rate. Fix: size from
  `io.length`, rename or feed accept flags, log unconditionally or delete.
- **dead-api-and-unused-types** (`Vision.java:120`, low). `getTargetX`
  throws; the `type` field and `MEGATAG` enum are unused. Fix: delete or
  implement `getTargetX`; remove the field and enum; use `bestReprojError` as
  a test input if multi-tag ever fires, otherwise stop logging it.
- **vision-tests-md-stale** (`VISION_TESTS.md:93`, low). Four documents
  describe a filter that no longer exists. Fix: commit the staged deletions;
  replace "majority" in the `CORRELATION_BOOST_FACTOR` comment.
- **vision-guide-numeric-claims-check** (`VISION_GUIDE.md:832`, low). Two
  claims in the guide were wrong. Fix: applied in the guide's revision the
  same day.
- **guide-section-7-runtime-claim-hides-live-vs-frozen-split**
  (`VISION_GUIDE.md:642`, medium). The guide says nothing is runtime-tunable,
  although half the statics are live and unlogged. Fix: until the config
  refactor, a live-versus-frozen table in section 7 with a warning that
  runtime writes are unlogged; after it, one snapshot per loop from the
  config source.

## Appendix B. Refuted findings

- stalled-camera-reports-connected. The claim that a stalled stream is
  undetectable was refuted: PhotonLib's `isConnected` is a heartbeat with a
  0.5 s debounce, and a stalled stream is detected within that time.
- playground-output-invisible-and-not-preserved. The claim that Gradle
  swallows the playground's output was refuted: the GradleRIO plugin
  configures test logging to show standard streams, and the output appears.

## Appendix C. What was checked and found sound

- The weighted geometric mean is implemented correctly for its definition:
  order-independent, a zero vetoes, weights act as exponents.
- The circular mean for fused yaw is computed correctly.
- The transform composition in both the single-tag and multi-tag branches is
  correct.
- The union-find clustering is correct and leaves no state between calls.
- All four camera quaternions are normalized and decode to the angles in
  their comments; the left and right pairs are exact mirrors.
- Capture timestamps are in the roboRIO time base in steady state
  (observation age at logging 48 to 88 ms across three matches).
- Subsystem order is `Drive` then `Vision`, so the loop's odometry sample is
  in the buffer before its vision measurements arrive.
- Per-observation decisions replay byte-identically through the current code
  (4,293 of 4,293 accepted records in E8), so the filter is deterministic.
- The snapshot arrays are freshly allocated per update, so the immutability
  the thread relies on holds.
