// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Handles scoring and filtering of vision observations.
 *
 * <p>This class is separated from Vision to make the scoring logic testable in isolation.
 */
public class VisionFilter {

  // Arena boundary for withinBoundaries test
  private static final Rectangle2d arenaRectangle;

  static {
    double halfWidth = minRobotWidthHalfMeters;
    var cornerA = new Translation2d(halfWidth, halfWidth);
    var cornerB = new Translation2d(fieldXLenMeters - halfWidth, fieldYLenMeters - halfWidth);
    arenaRectangle = new Rectangle2d(cornerA, cornerB);
  }

  // Reusable data structures for cross-camera correlation (avoids allocations)
  private static final int MAX_OBSERVATIONS = 32;

  @SuppressWarnings("unchecked")
  private final Set<Integer>[] clusters = new HashSet[MAX_OBSERVATIONS];

  private final Map<Integer, Set<Integer>> clusterMap = new HashMap<>();
  private final Set<Integer> camerasInCluster = new HashSet<>();

  {
    for (int i = 0; i < MAX_OBSERVATIONS; i++) {
      clusters[i] = new HashSet<>();
    }
  }

  // Reusable context object for tests
  private final TestContext testContext = new TestContext();

  /** Observation paired with its camera index and test scores. */
  public record TestedObservation(
      PoseObservation observation,
      int cameraIndex,
      EnumMap<Test, Double> testResults,
      double score) {}

  /**
   * Fused observation from multiple cameras agreeing on a pose.
   *
   * @param pose The averaged 2D pose
   * @param timestamp The averaged timestamp
   * @param score The combined score (highest of contributing observations)
   * @param cameraCount Number of cameras that contributed to this fused pose
   */
  public record FusedObservation(Pose2d pose, double timestamp, double score, int cameraCount) {}

  /** Mutable context object for vision tests, reused to avoid allocations. */
  public static class TestContext {
    private PoseObservation observation;
    private int cameraIndex;
    private Pose2d lastAcceptedPose;
    private double lastAcceptedTimestamp;
    private Rotation2d gyroYaw;

    public TestContext observation(PoseObservation observation) {
      this.observation = observation;
      return this;
    }

    public TestContext cameraIndex(int cameraIndex) {
      this.cameraIndex = cameraIndex;
      return this;
    }

    public TestContext lastAcceptedPose(Pose2d lastAcceptedPose) {
      this.lastAcceptedPose = lastAcceptedPose;
      return this;
    }

    public TestContext lastAcceptedTimestamp(double lastAcceptedTimestamp) {
      this.lastAcceptedTimestamp = lastAcceptedTimestamp;
      return this;
    }

    public TestContext gyroYaw(Rotation2d gyroYaw) {
      this.gyroYaw = gyroYaw;
      return this;
    }

    public PoseObservation observation() {
      return observation;
    }

    public int cameraIndex() {
      return cameraIndex;
    }

    public Pose2d lastAcceptedPose() {
      return lastAcceptedPose;
    }

    public double lastAcceptedTimestamp() {
      return lastAcceptedTimestamp;
    }

    public Rotation2d gyroYaw() {
      return gyroYaw;
    }
  }

  /**
   * Vision tests that evaluate the quality of pose observations.
   *
   * <p>Each test returns a score between 0 and 1. Tests have weights that determine their impact on
   * the final score via weighted geometric mean.
   */
  public enum Test {
    unambiguous(0.8) {
      @Override
      public double test(TestContext ctx) {
        if (ctx.observation().tagCount() == 1) {
          return 1.0 - normalizedSigmoid(ctx.observation().ambiguity(), ambiguityTolerance, 4.0);
        }
        return 1.0;
      }
    },

    pitchError(0.7) {
      @Override
      public double test(TestContext ctx) {
        return 1.0
            - normalizedSigmoid(
                Math.abs(ctx.observation().pose().getRotation().getY()),
                pitchToleranceRadians,
                1.0);
      }
    },

    rollError(0.7) {
      @Override
      public double test(TestContext ctx) {
        return 1.0
            - normalizedSigmoid(
                Math.abs(ctx.observation().pose().getRotation().getX()), rollToleranceRadians, 1.0);
      }
    },

    heightError(0.7) {
      @Override
      public double test(TestContext ctx) {
        return 1.0
            - normalizedSigmoid(
                Math.abs(ctx.observation().pose().getZ()), elevationToleranceMeters, 1.0);
      }
    },

    withinBoundaries(1.0) {
      @Override
      public double test(TestContext ctx) {
        return arenaRectangle.contains(ctx.observation().pose().toPose2d().getTranslation())
            ? 1.0
            : 0.0;
      }
    },

    moreThanZeroTags(1.0) {
      @Override
      public double test(TestContext ctx) {
        return Math.min(ctx.observation().tagCount(), 1.0);
      }
    },

    distanceToTags(0.5) {
      @Override
      public double test(TestContext ctx) {
        return 1.0
            - normalizedSigmoid(
                ctx.observation().averageTagDistance(), tagDistanceToleranceMeters, 1.0);
      }
    },

    /**
     * Checks whether the implied velocity between this observation and the last accepted
     * observation from the same camera is physically reasonable.
     *
     * <p>When no history is available (first observation from this camera, or the last accepted
     * observation is older than velocityCheckTimeoutSeconds), the test returns
     * velocityUncertainScore instead of 1.0. This prevents a bad first-in-a-while pose from sailing
     * through with a perfect velocity score — it must earn acceptance from the other tests. The
     * correlation boost still allows startup convergence when multiple cameras agree.
     */
    velocityConsistency(0.9) {
      @Override
      public double test(TestContext ctx) {
        // No history from this camera — express uncertainty, not confidence
        if (ctx.lastAcceptedPose() == null) return velocityUncertainScore;

        double dt = ctx.observation().timestamp() - ctx.lastAcceptedTimestamp();
        if (dt <= 0.001) return 1.0; // Near-simultaneous, skip check
        // Stale history — treat the same as no history
        if (dt > velocityCheckTimeoutSeconds) return velocityUncertainScore;

        double distance =
            ctx.observation()
                .pose()
                .toPose2d()
                .getTranslation()
                .getDistance(ctx.lastAcceptedPose().getTranslation());
        double impliedVelocity = distance / dt;

        return 1.0 - normalizedSigmoid(impliedVelocity, maxReasonableVelocityMps, 2.0);
      }
    },

    /**
     * Checks whether the vision pose yaw matches the gyro yaw.
     *
     * <p>Ambiguous PnP solutions almost always have incorrect yaw because the ambiguity comes from
     * the tag appearing similar from different viewing angles, which inherently involves rotation.
     * The gyro provides reliable yaw tracking independent of vision, making this an effective check
     * for rejecting bad PnP solutions.
     */
    yawConsistency(1.0) {
      @Override
      public double test(TestContext ctx) {
        if (ctx.gyroYaw() == null) return 1.0; // No gyro data, skip check

        double visionYaw = ctx.observation().pose().toPose2d().getRotation().getRadians();
        double gyroYaw = ctx.gyroYaw().getRadians();
        double yawError = Math.abs(MathUtil.angleModulus(visionYaw - gyroYaw));

        return 1.0 - normalizedSigmoid(yawError, yawToleranceRadians, 4.0);
      }
    };

    private final double weight;

    Test(double weight) {
      this.weight = weight;
    }

    public double weight() {
      return weight;
    }

    public abstract double test(TestContext ctx);
  }

  /** Default enabled tests. */
  public static final EnumSet<Test> DEFAULT_ENABLED_TESTS =
      EnumSet.of(
          Test.moreThanZeroTags,
          Test.unambiguous,
          Test.pitchError,
          Test.rollError,
          Test.heightError,
          Test.withinBoundaries,
          Test.distanceToTags,
          // Test.velocityConsistency,
          Test.yawConsistency);

  /**
   * Scores an observation by running all enabled tests.
   *
   * @param observation The pose observation to score
   * @param cameraIndex Which camera produced this observation
   * @param lastAcceptedPose Last accepted pose from this camera (null if none)
   * @param lastAcceptedTimestamp Timestamp of last accepted pose
   * @param gyroYaw Current gyro yaw for yaw consistency check (null to skip check)
   * @param enabledTests Which tests to run
   * @return TestedObservation with scores
   */
  public TestedObservation scoreObservation(
      PoseObservation observation,
      int cameraIndex,
      Pose2d lastAcceptedPose,
      double lastAcceptedTimestamp,
      Rotation2d gyroYaw,
      EnumSet<Test> enabledTests) {

    testContext
        .observation(observation)
        .cameraIndex(cameraIndex)
        .lastAcceptedPose(lastAcceptedPose)
        .lastAcceptedTimestamp(lastAcceptedTimestamp)
        .gyroYaw(gyroYaw);

    var testResults = new EnumMap<Test, Double>(Test.class);
    for (var test : enabledTests) {
      testResults.put(test, test.test(testContext));
    }

    // Weighted geometric mean
    double weightedProduct = 1.0;
    double sumOfWeights = 0.0;
    for (var entry : testResults.entrySet()) {
      double score = entry.getValue();
      double weight = entry.getKey().weight();
      weightedProduct *= Math.pow(score, weight);
      sumOfWeights += weight;
    }
    double totalScore = Math.pow(weightedProduct, 1.0 / sumOfWeights);

    return new TestedObservation(observation, cameraIndex, testResults, totalScore);
  }

  /**
   * Fuses correlated observations from multiple cameras into single averaged poses.
   *
   * <p>When multiple cameras report similar poses at similar times, they are fused into a single
   * observation with an averaged position. This reduces jitter from multiple cameras providing
   * slightly different poses that all get accepted. Uncorrelated observations are returned as-is.
   *
   * @param observations List of tested observations to potentially fuse
   * @return List of fused observations (may be smaller than input if observations were merged)
   */
  public ArrayList<FusedObservation> fuseCorrelatedObservations(
      ArrayList<TestedObservation> observations) {
    var result = new ArrayList<FusedObservation>();
    int n = observations.size();

    if (n == 0) {
      return result;
    }

    if (n > MAX_OBSERVATIONS) {
      // Too many observations, fall back to individual processing
      for (var obs : observations) {
        result.add(
            new FusedObservation(
                obs.observation().pose().toPose2d(),
                obs.observation().timestamp(),
                obs.score(),
                1));
      }
      return result;
    }

    // Step 1: Build clusters of agreeing observations
    // Each observation starts in its own cluster
    clusterMap.clear();
    for (int i = 0; i < n; i++) {
      clusters[i].clear();
      clusters[i].add(i);
      clusterMap.put(i, clusters[i]);
    }

    // Step 2: Merge clusters when observations agree (same time window, similar pose, different
    // cameras)
    for (int i = 0; i < n; i++) {
      var obsA = observations.get(i);
      double timeA = obsA.observation().timestamp();
      var transA = obsA.observation().pose().toPose2d().getTranslation();

      for (int j = i + 1; j < n; j++) {
        var obsB = observations.get(j);
        if (obsA.cameraIndex() == obsB.cameraIndex()) continue;

        double timeB = obsB.observation().timestamp();
        if (Math.abs(timeA - timeB) > correlationTimeWindowSeconds) continue;

        var transB = obsB.observation().pose().toPose2d().getTranslation();
        if (transA.getDistance(transB) > correlationPoseThresholdMeters) continue;

        // Observations agree - merge their clusters
        var clusterA = clusterMap.get(i);
        var clusterB = clusterMap.get(j);
        if (clusterA != clusterB) {
          // Merge B into A
          clusterA.addAll(clusterB);
          // Update all members of B to point to merged cluster
          for (int member : clusterB) {
            clusterMap.put(member, clusterA);
          }
        }
      }
    }

    // Step 3: Track which observations have been processed
    var processed = new boolean[n];

    // Step 4: Create fused observations for each cluster
    for (int i = 0; i < n; i++) {
      if (processed[i]) continue;

      var cluster = clusterMap.get(i);

      // Count unique cameras in this cluster
      camerasInCluster.clear();
      for (int idx : cluster) {
        camerasInCluster.add(observations.get(idx).cameraIndex());
      }
      int cameraCount = camerasInCluster.size();

      if (cameraCount >= 2) {
        // Multiple cameras agree - compute weighted average pose
        Translation2d sumTranslation = Translation2d.kZero;
        double sumSin = 0, sumCos = 0, sumTime = 0;
        double sumWeight = 0;
        double maxScore = 0;

        for (int idx : cluster) {
          var obs = observations.get(idx);
          double weight = obs.score();
          var pose = obs.observation().pose().toPose2d();

          sumTranslation = sumTranslation.plus(pose.getTranslation().times(weight));
          // Circular mean for angles (no WPILib equivalent for weighted circular mean)
          sumSin += Math.sin(pose.getRotation().getRadians()) * weight;
          sumCos += Math.cos(pose.getRotation().getRadians()) * weight;
          sumTime += obs.observation().timestamp() * weight;
          sumWeight += weight;
          maxScore = Math.max(maxScore, obs.score());

          processed[idx] = true;
        }

        // Compute averages
        Translation2d avgTranslation = sumTranslation.div(sumWeight);
        double avgYaw = Math.atan2(sumSin / sumWeight, sumCos / sumWeight);
        double avgTime = sumTime / sumWeight;

        // Boost score for correlated observations
        double fusedScore = Math.min(1.0, maxScore * correlationBoostFactor);

        result.add(
            new FusedObservation(
                new Pose2d(avgTranslation, new Rotation2d(avgYaw)),
                avgTime,
                fusedScore,
                cameraCount));
      } else {
        // Single camera observation - return as-is
        var obs = observations.get(i);
        result.add(
            new FusedObservation(
                obs.observation().pose().toPose2d(),
                obs.observation().timestamp(),
                obs.score(),
                1));
        processed[i] = true;
      }
    }

    return result;
  }

  /** Normalized sigmoid function. Output is between 0 and 1. */
  public static double normalizedSigmoid(double x, double midpoint, double steepness) {
    double exponent = -steepness * (x - midpoint);
    return 1.0 / (1.0 + Math.exp(exponent));
  }
}
