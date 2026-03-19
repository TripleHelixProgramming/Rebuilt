# Vision Tests Branch

This branch improves how vision observations are filtered and scored.

## Overview

1. **VisionFilter extraction** - Scoring logic moved to separate class for testability
2. **Weighted test scoring** - Tests have weights; uses geometric mean
3. **TestContext refactoring** - Tests receive context object instead of just the observation
4. **Velocity consistency test** - Penalizes observations implying impossible movement
5. **Cross-camera correlation boost** - Rewards observations when multiple cameras agree
6. **Unit test suite** - 78 tests covering all scoring logic and edge cases

---

## VisionFilter Refactoring

The scoring and filtering logic has been extracted from `Vision.java` into `VisionFilter.java`. This separation:

- Makes the scoring logic **unit-testable** in isolation (no hardware dependencies)
- Keeps `Vision.java` focused on IO and coordination
- Allows the filter to be reused or swapped if needed

```
Vision.java          → IO, logging, pose estimator integration
VisionFilter.java    → Scoring tests, correlation boost, filtering logic
VisionConstants.java → Tunable thresholds
```

---

## Weighted Test Scoring

Each `VisionFilter.Test` has a weight determining its influence on the final score:

```java
withinBoundaries(1.0),    // Critical
moreThanZeroTags(1.0),    // Critical
velocityConsistency(0.9), // Important
unambiguous(0.8),         // Important
pitchError(0.7),          // Moderate
rollError(0.7),           // Moderate
heightError(0.7),         // Moderate
distanceToTags(0.5),      // Quality hint
```

Scores are combined using **weighted geometric mean**:

```
totalScore = (score1^w1 × score2^w2 × ...)^(1/sumOfWeights)
```

This gives the average score per test, weighted by importance.

---

## TestContext

Tests now receive a `TestContext` with the observation plus camera state:

```java
testContext
    .observation(observation)
    .cameraIndex(cameraIndex)
    .lastAcceptedPose(lastAcceptedPose[cameraIndex])
    .lastAcceptedTimestamp(lastAcceptedTimestamp[cameraIndex]);

for (var test : enabledTests) {
    testResults.put(test, test.test(testContext));
}
```

---

## Velocity Consistency Test

Compares each observation to the last accepted pose from the same camera. If the implied velocity exceeds `maxReasonableVelocityMps` (1.5× max drivetrain speed), the observation is penalized.

Uses a sigmoid for smooth falloff rather than a hard cutoff.

**Why not use actual odometry velocity?** Circular dependency (velocity estimate uses vision), and the robot might be carried (wheels not moving but robot is).

---

## Cross-Camera Correlation Boost

When multiple cameras report similar poses at similar times, boost their scores.

**Algorithm:**
1. Each observation starts in its own cluster (a Set)
2. Find observations from different cameras that agree (within 50ms and 15cm)
3. Merge clusters when observations agree
4. If a cluster has **majority** of reporting cameras, boost those scores by 1.3×

**Why majority?** If front cameras agree on pose A and back cameras agree on pose B, but A≠B, we have a conflict. Neither should be boosted.

| Scenario | Result |
|----------|--------|
| 4 agree | All boosted |
| 3 agree, 1 differs | The 3 boosted |
| 2v2 split | Nobody boosted |

---

## Configuration (VisionConstants.java)

```java
// Velocity consistency
velocityCheckTimeoutSeconds = 0.5
maxReasonableVelocityMps = drivetrainSpeedLimit * 1.5

// Cross-camera correlation
correlationTimeWindowSeconds = 0.050
correlationPoseThresholdMeters = 0.15
correlationBoostFactor = 1.3
```

---

## Unit Test Suite

The `VisionFilterTest` class provides comprehensive test coverage for the scoring logic. Run with:

```bash
./gradlew test --tests "frc.robot.subsystems.vision.VisionFilterTest"
```

**Test categories:**

| Category | Coverage |
|----------|----------|
| `Test.unambiguous` | Single/multi-tag ambiguity handling |
| `Test.pitchError` | Pitch tolerance, symmetry |
| `Test.rollError` | Roll tolerance |
| `Test.heightError` | Height tolerance, negative values |
| `Test.withinBoundaries` | Field boundary enforcement |
| `Test.moreThanZeroTags` | Zero-tag rejection |
| `Test.distanceToTags` | Distance penalty curve |
| `Test.velocityConsistency` | Impossible velocity detection |
| `scoreObservation` | Weighted geometric mean, test selection |
| `applyCorrelationBoost` | Majority rule, 2v2 splits, boost limits |
| `normalizedSigmoid` | Edge cases, steepness behavior |
| Integration | End-to-end scoring scenarios |
