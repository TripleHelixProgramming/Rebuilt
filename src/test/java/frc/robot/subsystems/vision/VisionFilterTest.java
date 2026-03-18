package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.vision.VisionFilter.FusedObservation;
import frc.robot.subsystems.vision.VisionFilter.Test;
import frc.robot.subsystems.vision.VisionFilter.TestContext;
import frc.robot.subsystems.vision.VisionFilter.TestedObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.ArrayList;
import java.util.EnumSet;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;

class VisionFilterTest {

  private VisionFilter filter;

  // Field dimensions (approximate, for boundary tests)
  private static final double FIELD_X = 16.54; // meters
  private static final double FIELD_Y = 8.21; // meters
  private static final double ROBOT_HALF_WIDTH = 0.47; // ~18.5 inches

  @BeforeEach
  void setUp() {
    filter = new VisionFilter();
  }

  // ==================== Helper Methods ====================

  /** Creates a basic valid observation at the given position. */
  private PoseObservation makeObservation(double x, double y, double timestamp) {
    return new PoseObservation(
        timestamp,
        new Pose3d(x, y, 0, new Rotation3d()),
        0.01, // low ambiguity
        1, // 1 tag
        3.0, // average tag distance
        PoseObservationType.MEGATAG_1);
  }

  /** Creates an observation with full control over all parameters. */
  private PoseObservation makeObservation(
      double x,
      double y,
      double z,
      double roll,
      double pitch,
      double yaw,
      double timestamp,
      int tagCount,
      double ambiguity,
      double tagDistance) {
    return new PoseObservation(
        timestamp,
        new Pose3d(x, y, z, new Rotation3d(roll, pitch, yaw)),
        ambiguity,
        tagCount,
        tagDistance,
        PoseObservationType.MEGATAG_1);
  }

  /** Creates a TestedObservation for correlation boost tests. */
  private TestedObservation makeTestedObs(
      double x, double y, double timestamp, int cameraIndex, double score) {
    return new TestedObservation(makeObservation(x, y, timestamp), cameraIndex, null, score);
  }

  /** Scores an observation with default tests enabled. */
  private TestedObservation score(PoseObservation obs) {
    return filter.scoreObservation(obs, 0, null, 0.0, null, VisionFilter.DEFAULT_ENABLED_TESTS);
  }

  /** Scores with velocity tracking from a previous pose. */
  private TestedObservation scoreWithHistory(
      PoseObservation obs, Pose2d lastPose, double lastTimestamp) {
    return filter.scoreObservation(
        obs, 0, lastPose, lastTimestamp, null, VisionFilter.DEFAULT_ENABLED_TESTS);
  }

  /** Scores with a specific gyro yaw for yaw consistency testing. */
  private TestedObservation scoreWithGyroYaw(PoseObservation obs, Rotation2d gyroYaw) {
    return filter.scoreObservation(obs, 0, null, 0.0, gyroYaw, VisionFilter.DEFAULT_ENABLED_TESTS);
  }

  // ==================== Individual Test Tests ====================

  @Nested
  @DisplayName("Test.unambiguous")
  class UnambiguousTests {

    @org.junit.jupiter.api.Test
    @DisplayName("Single tag with low ambiguity scores higher than high ambiguity")
    void lowAmbiguitySingleTag() {
      var lowAmb =
          new TestContext().observation(makeObservation(8, 4, 0, 0, 0, 0, 0.0, 1, 0.01, 3.0));
      var highAmb =
          new TestContext().observation(makeObservation(8, 4, 0, 0, 0, 0, 0.0, 1, 0.3, 3.0));
      double lowResult = Test.unambiguous.test(lowAmb);
      double highResult = Test.unambiguous.test(highAmb);
      assertTrue(
          lowResult > highResult,
          "Low ambiguity should score higher: " + lowResult + " vs " + highResult);
      assertTrue(lowResult > 0.5, "Low ambiguity should be > 0.5, got " + lowResult);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Single tag with high ambiguity scores low")
    void highAmbiguitySingleTag() {
      var ctx = new TestContext().observation(makeObservation(8, 4, 0, 0, 0, 0, 0.0, 1, 0.5, 3.0));
      double result = Test.unambiguous.test(ctx);
      assertTrue(result < 0.5, "High ambiguity should score < 0.5, got " + result);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Multiple tags ignore ambiguity (always 1.0)")
    void multipleTagsIgnoreAmbiguity() {
      var ctx = new TestContext().observation(makeObservation(8, 4, 0, 0, 0, 0, 0.0, 2, 0.99, 3.0));
      double result = Test.unambiguous.test(ctx);
      assertEquals(1.0, result, 0.001, "Multiple tags should always return 1.0");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Zero tags still returns 1.0 (ambiguity irrelevant)")
    void zeroTagsReturnsOne() {
      var ctx = new TestContext().observation(makeObservation(8, 4, 0, 0, 0, 0, 0.0, 0, 0.5, 3.0));
      double result = Test.unambiguous.test(ctx);
      assertEquals(1.0, result, 0.001, "Zero tags should return 1.0 (caught by moreThanZeroTags)");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Ambiguity at tolerance threshold")
    void ambiguityAtThreshold() {
      double tolerance = VisionConstants.ambiguityTolerance; // 0.15
      var ctx =
          new TestContext().observation(makeObservation(8, 4, 0, 0, 0, 0, 0.0, 1, tolerance, 3.0));
      double result = Test.unambiguous.test(ctx);
      // At midpoint of sigmoid, should be around 0.5
      assertTrue(result > 0.3 && result < 0.7, "At threshold should be ~0.5, got " + result);
    }
  }

  @Nested
  @DisplayName("Test.pitchError")
  class PitchErrorTests {

    @org.junit.jupiter.api.Test
    @DisplayName("Zero pitch scores higher than large pitch")
    void zeroPitchScoresHigher() {
      var zeroPitch =
          new TestContext().observation(makeObservation(8, 4, 0, 0, 0, 0, 0.0, 1, 0.01, 3.0));
      var largePitch =
          new TestContext()
              .observation(makeObservation(8, 4, 0, 0, Math.toRadians(30), 0, 0.0, 1, 0.01, 3.0));
      double zeroResult = Test.pitchError.test(zeroPitch);
      double largeResult = Test.pitchError.test(largePitch);
      assertTrue(
          zeroResult > largeResult,
          "Zero pitch should score higher: " + zeroResult + " vs " + largeResult);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Large positive pitch scores low")
    void largePositivePitch() {
      var ctx =
          new TestContext()
              .observation(makeObservation(8, 4, 0, 0, Math.toRadians(30), 0, 0.0, 1, 0.01, 3.0));
      double result = Test.pitchError.test(ctx);
      assertTrue(result < 0.5, "30 degree pitch should score < 0.5, got " + result);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Negative pitch penalized same as positive (absolute value)")
    void negativePitchSymmetric() {
      var posPitch =
          new TestContext()
              .observation(makeObservation(8, 4, 0, 0, Math.toRadians(15), 0, 0.0, 1, 0.01, 3.0));
      var negPitch =
          new TestContext()
              .observation(makeObservation(8, 4, 0, 0, Math.toRadians(-15), 0, 0.0, 1, 0.01, 3.0));
      double posResult = Test.pitchError.test(posPitch);
      double negResult = Test.pitchError.test(negPitch);
      assertEquals(
          posResult, negResult, 0.001, "Positive and negative pitch should score the same");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Small pitch scores better than large pitch")
    void smallPitchScoresBetter() {
      var smallPitch =
          new TestContext()
              .observation(makeObservation(8, 4, 0, 0, Math.toRadians(2), 0, 0.0, 1, 0.01, 3.0));
      var largePitch =
          new TestContext()
              .observation(makeObservation(8, 4, 0, 0, Math.toRadians(20), 0, 0.0, 1, 0.01, 3.0));
      double smallResult = Test.pitchError.test(smallPitch);
      double largeResult = Test.pitchError.test(largePitch);
      assertTrue(
          smallResult > largeResult,
          "Small pitch should score better: " + smallResult + " vs " + largeResult);
    }
  }

  @Nested
  @DisplayName("Test.rollError")
  class RollErrorTests {

    @org.junit.jupiter.api.Test
    @DisplayName("Zero roll scores higher than large roll")
    void zeroRollScoresHigher() {
      var zeroRoll =
          new TestContext().observation(makeObservation(8, 4, 0, 0, 0, 0, 0.0, 1, 0.01, 3.0));
      var largeRoll =
          new TestContext()
              .observation(makeObservation(8, 4, 0, Math.toRadians(25), 0, 0, 0.0, 1, 0.01, 3.0));
      double zeroResult = Test.rollError.test(zeroRoll);
      double largeResult = Test.rollError.test(largeRoll);
      assertTrue(
          zeroResult > largeResult,
          "Zero roll should score higher: " + zeroResult + " vs " + largeResult);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Large roll scores low")
    void largeRoll() {
      var ctx =
          new TestContext()
              .observation(makeObservation(8, 4, 0, Math.toRadians(25), 0, 0, 0.0, 1, 0.01, 3.0));
      double result = Test.rollError.test(ctx);
      assertTrue(result < 0.5, "25 degree roll should score < 0.5, got " + result);
    }
  }

  @Nested
  @DisplayName("Test.heightError")
  class HeightErrorTests {

    @org.junit.jupiter.api.Test
    @DisplayName("Zero height scores higher than large height")
    void zeroHeightScoresHigher() {
      var zeroHeight =
          new TestContext().observation(makeObservation(8, 4, 0, 0, 0, 0, 0.0, 1, 0.01, 3.0));
      var largeHeight =
          new TestContext().observation(makeObservation(8, 4, 1.0, 0, 0, 0, 0.0, 1, 0.01, 3.0));
      double zeroResult = Test.heightError.test(zeroHeight);
      double largeResult = Test.heightError.test(largeHeight);
      assertTrue(
          zeroResult > largeResult,
          "Zero height should score higher: " + zeroResult + " vs " + largeResult);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Large positive height scores low")
    void largePositiveHeight() {
      var ctx =
          new TestContext().observation(makeObservation(8, 4, 1.0, 0, 0, 0, 0.0, 1, 0.01, 3.0));
      double result = Test.heightError.test(ctx);
      assertTrue(result < 0.5, "1m height should score < 0.5, got " + result);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Negative height (underground) scores low")
    void negativeHeight() {
      var ctx =
          new TestContext().observation(makeObservation(8, 4, -0.5, 0, 0, 0, 0.0, 1, 0.01, 3.0));
      double result = Test.heightError.test(ctx);
      assertTrue(result < 0.5, "Negative height should score low, got " + result);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Small height scores better than large height")
    void smallHeightScoresBetter() {
      var smallHeight =
          new TestContext().observation(makeObservation(8, 4, 0.1, 0, 0, 0, 0.0, 1, 0.01, 3.0));
      var largeHeight =
          new TestContext().observation(makeObservation(8, 4, 0.8, 0, 0, 0, 0.0, 1, 0.01, 3.0));
      double smallResult = Test.heightError.test(smallHeight);
      double largeResult = Test.heightError.test(largeHeight);
      assertTrue(
          smallResult > largeResult,
          "Small height should score better: " + smallResult + " vs " + largeResult);
    }
  }

  @Nested
  @DisplayName("Test.withinBoundaries")
  class WithinBoundariesTests {

    @org.junit.jupiter.api.Test
    @DisplayName("Center of field scores 1.0")
    void centerOfField() {
      var ctx = new TestContext().observation(makeObservation(FIELD_X / 2, FIELD_Y / 2, 0.0));
      double result = Test.withinBoundaries.test(ctx);
      assertEquals(1.0, result, 0.001);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Just inside boundary scores 1.0")
    void justInsideBoundary() {
      double margin = ROBOT_HALF_WIDTH + 0.1;
      var ctx = new TestContext().observation(makeObservation(margin, margin, 0.0));
      double result = Test.withinBoundaries.test(ctx);
      assertEquals(1.0, result, 0.001);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Outside X boundary (negative) scores 0")
    void outsideNegativeX() {
      var ctx = new TestContext().observation(makeObservation(-1.0, 4.0, 0.0));
      double result = Test.withinBoundaries.test(ctx);
      assertEquals(0.0, result, 0.001);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Outside X boundary (positive) scores 0")
    void outsidePositiveX() {
      var ctx = new TestContext().observation(makeObservation(FIELD_X + 1.0, 4.0, 0.0));
      double result = Test.withinBoundaries.test(ctx);
      assertEquals(0.0, result, 0.001);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Outside Y boundary (negative) scores 0")
    void outsideNegativeY() {
      var ctx = new TestContext().observation(makeObservation(8.0, -1.0, 0.0));
      double result = Test.withinBoundaries.test(ctx);
      assertEquals(0.0, result, 0.001);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Outside Y boundary (positive) scores 0")
    void outsidePositiveY() {
      var ctx = new TestContext().observation(makeObservation(8.0, FIELD_Y + 1.0, 0.0));
      double result = Test.withinBoundaries.test(ctx);
      assertEquals(0.0, result, 0.001);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Corner outside scores 0")
    void cornerOutside() {
      var ctx = new TestContext().observation(makeObservation(-5.0, -5.0, 0.0));
      double result = Test.withinBoundaries.test(ctx);
      assertEquals(0.0, result, 0.001);
    }
  }

  @Nested
  @DisplayName("Test.moreThanZeroTags")
  class MoreThanZeroTagsTests {

    @org.junit.jupiter.api.Test
    @DisplayName("Zero tags scores 0")
    void zeroTags() {
      var ctx = new TestContext().observation(makeObservation(8, 4, 0, 0, 0, 0, 0.0, 0, 0.01, 3.0));
      double result = Test.moreThanZeroTags.test(ctx);
      assertEquals(0.0, result, 0.001);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("One tag scores 1.0")
    void oneTag() {
      var ctx = new TestContext().observation(makeObservation(8, 4, 0, 0, 0, 0, 0.0, 1, 0.01, 3.0));
      double result = Test.moreThanZeroTags.test(ctx);
      assertEquals(1.0, result, 0.001);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Multiple tags scores 1.0")
    void multipleTags() {
      var ctx = new TestContext().observation(makeObservation(8, 4, 0, 0, 0, 0, 0.0, 5, 0.01, 3.0));
      double result = Test.moreThanZeroTags.test(ctx);
      assertEquals(1.0, result, 0.001);
    }
  }

  @Nested
  @DisplayName("Test.distanceToTags")
  class DistanceToTagsTests {

    @org.junit.jupiter.api.Test
    @DisplayName("Close tags score well")
    void closeTags() {
      var ctx = new TestContext().observation(makeObservation(8, 4, 0, 0, 0, 0, 0.0, 1, 0.01, 1.0));
      double result = Test.distanceToTags.test(ctx);
      assertTrue(result > 0.8, "1m distance should score well, got " + result);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Far tags score poorly")
    void farTags() {
      var ctx = new TestContext().observation(makeObservation(8, 4, 0, 0, 0, 0, 0.0, 1, 0.01, 8.0));
      double result = Test.distanceToTags.test(ctx);
      assertTrue(result < 0.3, "8m distance should score poorly, got " + result);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Moderate distance at tolerance")
    void moderateDistance() {
      double tolerance = VisionConstants.tagDistanceToleranceMeters; // 4.0
      var ctx =
          new TestContext().observation(makeObservation(8, 4, 0, 0, 0, 0, 0.0, 1, 0.01, tolerance));
      double result = Test.distanceToTags.test(ctx);
      assertTrue(result > 0.3 && result < 0.7, "At tolerance should be ~0.5, got " + result);
    }
  }

  @Nested
  @DisplayName("Test.velocityConsistency")
  class VelocityConsistencyTests {

    @org.junit.jupiter.api.Test
    @DisplayName("No previous pose returns uncertain score")
    void noPreviousPose() {
      var ctx =
          new TestContext()
              .observation(makeObservation(8, 4, 0.0))
              .lastAcceptedPose(null)
              .lastAcceptedTimestamp(0.0);
      double result = Test.velocityConsistency.test(ctx);
      assertEquals(
          VisionConstants.velocityUncertainScore,
          result,
          0.001,
          "No history should return uncertain score, not a perfect pass");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Reasonable velocity scores well")
    void reasonableVelocity() {
      var lastPose = new Pose2d(8.0, 4.0, new Rotation2d());
      var ctx =
          new TestContext()
              .observation(makeObservation(8.5, 4.0, 0.1)) // 0.5m in 0.1s = 5 m/s
              .lastAcceptedPose(lastPose)
              .lastAcceptedTimestamp(0.0);
      double result = Test.velocityConsistency.test(ctx);
      assertTrue(result > 0.5, "5 m/s should be acceptable, got " + result);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Impossible velocity scores poorly")
    void impossibleVelocity() {
      var lastPose = new Pose2d(8.0, 4.0, new Rotation2d());
      var ctx =
          new TestContext()
              .observation(makeObservation(18.0, 4.0, 0.1)) // 10m in 0.1s = 100 m/s
              .lastAcceptedPose(lastPose)
              .lastAcceptedTimestamp(0.0);
      double result = Test.velocityConsistency.test(ctx);
      assertTrue(result < 0.2, "100 m/s should be rejected, got " + result);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Very small dt returns 1.0 (avoid division issues)")
    void verySmallDt() {
      var lastPose = new Pose2d(8.0, 4.0, new Rotation2d());
      var ctx =
          new TestContext()
              .observation(makeObservation(18.0, 4.0, 0.0005)) // dt < 0.001
              .lastAcceptedPose(lastPose)
              .lastAcceptedTimestamp(0.0);
      double result = Test.velocityConsistency.test(ctx);
      assertEquals(1.0, result, 0.001, "Very small dt should return 1.0");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Long timeout returns uncertain score (stale data)")
    void longTimeout() {
      var lastPose = new Pose2d(8.0, 4.0, new Rotation2d());
      var ctx =
          new TestContext()
              .observation(makeObservation(18.0, 4.0, 1.0)) // 1 second later
              .lastAcceptedPose(lastPose)
              .lastAcceptedTimestamp(0.0);
      double result = Test.velocityConsistency.test(ctx);
      assertEquals(
          VisionConstants.velocityUncertainScore,
          result,
          0.001,
          "Stale timestamp should return uncertain score, not a perfect pass");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Stationary robot scores perfectly")
    void stationaryRobot() {
      var lastPose = new Pose2d(8.0, 4.0, new Rotation2d());
      var ctx =
          new TestContext()
              .observation(makeObservation(8.0, 4.0, 0.05))
              .lastAcceptedPose(lastPose)
              .lastAcceptedTimestamp(0.0);
      double result = Test.velocityConsistency.test(ctx);
      assertTrue(result > 0.95, "Stationary should score ~1.0, got " + result);
    }
  }

  @Nested
  @DisplayName("Test.yawConsistency")
  class YawConsistencyTests {

    @org.junit.jupiter.api.Test
    @DisplayName("No gyro data returns 1.0 (skip check)")
    void noGyroData() {
      var ctx =
          new TestContext()
              .observation(makeObservation(8, 4, 0, 0, 0, Math.toRadians(45), 0.0, 1, 0.01, 3.0))
              .gyroYaw(null);
      double result = Test.yawConsistency.test(ctx);
      assertEquals(1.0, result, 0.001, "No gyro should return 1.0 (skip check)");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Matching yaw scores higher than small error")
    void matchingYawScoresHigher() {
      double yaw = Math.toRadians(45);
      var matchingCtx =
          new TestContext()
              .observation(makeObservation(8, 4, 0, 0, 0, yaw, 0.0, 1, 0.01, 3.0))
              .gyroYaw(new Rotation2d(yaw));
      var smallErrorCtx =
          new TestContext()
              .observation(makeObservation(8, 4, 0, 0, 0, yaw, 0.0, 1, 0.01, 3.0))
              .gyroYaw(new Rotation2d(yaw + Math.toRadians(5)));

      double matchingResult = Test.yawConsistency.test(matchingCtx);
      double smallErrorResult = Test.yawConsistency.test(smallErrorCtx);

      assertTrue(
          matchingResult > smallErrorResult,
          "Matching yaw should score higher: " + matchingResult + " vs " + smallErrorResult);
      assertTrue(matchingResult > 0.5, "Matching yaw should score > 0.5, got " + matchingResult);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Small yaw error scores higher than large error")
    void smallYawErrorScoresHigher() {
      var smallErrorCtx =
          new TestContext()
              .observation(makeObservation(8, 4, 0, 0, 0, Math.toRadians(45), 0.0, 1, 0.01, 3.0))
              .gyroYaw(new Rotation2d(Math.toRadians(50))); // 5 degree error
      var largeErrorCtx =
          new TestContext()
              .observation(makeObservation(8, 4, 0, 0, 0, Math.toRadians(45), 0.0, 1, 0.01, 3.0))
              .gyroYaw(new Rotation2d(Math.toRadians(75))); // 30 degree error

      double smallResult = Test.yawConsistency.test(smallErrorCtx);
      double largeResult = Test.yawConsistency.test(largeErrorCtx);

      assertTrue(
          smallResult > largeResult,
          "Small error should score higher: " + smallResult + " vs " + largeResult);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Large yaw error scores poorly")
    void largeYawError() {
      var ctx =
          new TestContext()
              .observation(makeObservation(8, 4, 0, 0, 0, Math.toRadians(45), 0.0, 1, 0.01, 3.0))
              .gyroYaw(new Rotation2d(Math.toRadians(90))); // 45 degree error
      double result = Test.yawConsistency.test(ctx);
      assertTrue(result < 0.2, "45 degree error should score < 0.2, got " + result);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("180 degree error (flipped) scores near zero")
    void flippedYaw() {
      var ctx =
          new TestContext()
              .observation(makeObservation(8, 4, 0, 0, 0, Math.toRadians(0), 0.0, 1, 0.01, 3.0))
              .gyroYaw(new Rotation2d(Math.toRadians(180)));
      double result = Test.yawConsistency.test(ctx);
      assertTrue(result < 0.1, "180 degree error should score < 0.1, got " + result);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Handles angle wrapping correctly")
    void angleWrapping() {
      // Vision at -170 degrees, gyro at 170 degrees = 20 degree difference (not 340)
      var wrappedCtx =
          new TestContext()
              .observation(makeObservation(8, 4, 0, 0, 0, Math.toRadians(-170), 0.0, 1, 0.01, 3.0))
              .gyroYaw(new Rotation2d(Math.toRadians(170)));
      // Compare against a clearly large error
      var largeErrorCtx =
          new TestContext()
              .observation(makeObservation(8, 4, 0, 0, 0, Math.toRadians(0), 0.0, 1, 0.01, 3.0))
              .gyroYaw(new Rotation2d(Math.toRadians(90))); // 90 degree error

      double wrappedResult = Test.yawConsistency.test(wrappedCtx);
      double largeErrorResult = Test.yawConsistency.test(largeErrorCtx);

      assertTrue(
          wrappedResult > largeErrorResult,
          "20 degree wrapped error should score higher than 90 degree: "
              + wrappedResult
              + " vs "
              + largeErrorResult);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Rejects ambiguous PnP solution with wrong yaw")
    void rejectsAmbiguousPnP() {
      // Simulate a bad PnP solution that has ~45 degree yaw error
      // Test via scoreWithGyroYaw - score will be affected by velocityUncertainScore
      // but the yaw error should still pull the total score down significantly
      var badObs = makeObservation(8, 4, 0, 0, 0, Math.toRadians(126), 0.0, 1, 0.01, 3.0);
      var goodObs = makeObservation(8, 4, 0, 0, 0, Math.toRadians(171), 0.0, 1, 0.01, 3.0);

      var testedBad = scoreWithGyroYaw(badObs, new Rotation2d(Math.toRadians(171)));
      var testedGood = scoreWithGyroYaw(goodObs, new Rotation2d(Math.toRadians(171)));

      // Bad PnP with 45 degree yaw error should score significantly lower than good one
      assertTrue(
          testedGood.score() > testedBad.score(),
          "Good pose should score higher than bad PnP: "
              + testedGood.score()
              + " vs "
              + testedBad.score());
      // The bad pose should score below minScore threshold
      assertTrue(
          testedBad.score() < VisionConstants.minScore,
          "Bad PnP with 45 degree yaw error should score below minScore, got " + testedBad.score());
    }
  }

  // ==================== ScoreObservation Tests ====================

  @Nested
  @DisplayName("scoreObservation")
  class ScoreObservationTests {

    @org.junit.jupiter.api.Test
    @DisplayName("Valid observation in field center scores high")
    void validObservationScoresHigh() {
      var obs = makeObservation(8.0, 4.0, 0.0);
      var tested = score(obs);
      assertTrue(
          tested.score() > 0.5, "Valid observation should score > 0.5, got " + tested.score());
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Observation outside field scores zero")
    void outsideFieldScoresZero() {
      var obs = makeObservation(-5.0, -5.0, 0.0);
      var tested = score(obs);
      assertEquals(0.0, tested.score(), 0.001);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Zero tags scores zero")
    void zeroTagsScoresZero() {
      var obs = makeObservation(8, 4, 0, 0, 0, 0, 0.0, 0, 0.01, 3.0);
      var tested = score(obs);
      assertEquals(0.0, tested.score(), 0.001);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("High ambiguity reduces score")
    void highAmbiguityReducesScore() {
      var lowAmb = makeObservation(8, 4, 0, 0, 0, 0, 0.0, 1, 0.01, 3.0);
      var highAmb = makeObservation(8, 4, 0, 0, 0, 0, 0.0, 1, 0.5, 3.0);

      var testedLow = score(lowAmb);
      var testedHigh = score(highAmb);

      assertTrue(
          testedLow.score() > testedHigh.score(),
          "Low ambiguity should score higher: " + testedLow.score() + " vs " + testedHigh.score());
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Large pitch error reduces score")
    void largePitchReducesScore() {
      var noPitch = makeObservation(8, 4, 0, 0, 0, 0, 0.0, 1, 0.01, 3.0);
      var bigPitch = makeObservation(8, 4, 0, 0, Math.toRadians(20), 0, 0.0, 1, 0.01, 3.0);

      var testedNo = score(noPitch);
      var testedBig = score(bigPitch);

      assertTrue(
          testedNo.score() > testedBig.score(),
          "No pitch should score higher: " + testedNo.score() + " vs " + testedBig.score());
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Score includes all test results")
    void scoreIncludesAllTestResults() {
      var obs = makeObservation(8.0, 4.0, 0.0);
      var tested = score(obs);

      assertNotNull(tested.testResults());
      assertEquals(VisionFilter.DEFAULT_ENABLED_TESTS.size(), tested.testResults().size());
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Preserves observation and camera index")
    void preservesObservationAndCameraIndex() {
      var obs = makeObservation(8.0, 4.0, 0.0);
      var tested =
          filter.scoreObservation(obs, 3, null, 0.0, null, VisionFilter.DEFAULT_ENABLED_TESTS);

      assertSame(obs, tested.observation());
      assertEquals(3, tested.cameraIndex());
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Velocity check uses previous pose")
    void velocityCheckUsesPreviousPose() {
      var lastPose = new Pose2d(8.0, 4.0, new Rotation2d());

      // Reasonable movement
      var reasonable = makeObservation(8.5, 4.0, 0.1);
      var testedReasonable = scoreWithHistory(reasonable, lastPose, 0.0);

      // Impossible movement
      var impossible = makeObservation(18.0, 4.0, 0.1);
      var testedImpossible = scoreWithHistory(impossible, lastPose, 0.0);

      assertTrue(
          testedReasonable.score() > testedImpossible.score(),
          "Reasonable velocity should score higher");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Can disable specific tests")
    void canDisableSpecificTests() {
      var obs = makeObservation(-5.0, -5.0, 0.0); // Outside field

      // With boundary check - should be 0
      var enabledAll = VisionFilter.DEFAULT_ENABLED_TESTS;
      var testedAll = filter.scoreObservation(obs, 0, null, 0.0, null, enabledAll);
      assertEquals(0.0, testedAll.score(), 0.001);

      // Without boundary check - should be non-zero
      var enabledNoBoundary = EnumSet.copyOf(VisionFilter.DEFAULT_ENABLED_TESTS);
      enabledNoBoundary.remove(Test.withinBoundaries);
      var testedNoBoundary = filter.scoreObservation(obs, 0, null, 0.0, null, enabledNoBoundary);
      assertTrue(testedNoBoundary.score() > 0, "Without boundary check should score > 0");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Multiple failures compound (geometric mean)")
    void multipleFailuresCompound() {
      // Good observation
      var good = makeObservation(8, 4, 0, 0, 0, 0, 0.0, 1, 0.01, 2.0);

      // Bad observation: high ambiguity + bad pitch + high height
      var bad = makeObservation(8, 4, 0.5, 0, Math.toRadians(15), 0, 0.0, 1, 0.4, 6.0);

      var testedGood = score(good);
      var testedBad = score(bad);

      assertTrue(
          testedGood.score() > testedBad.score(),
          "Good should score higher than bad: " + testedGood.score() + " vs " + testedBad.score());
      // The geometric mean ensures multiple failures have compounding effect
      assertTrue(
          testedGood.score() > testedBad.score() * 1.2,
          "Multiple bad factors should have significant impact");
    }
  }

  // ==================== Fuse Correlated Observations Tests ====================

  @Nested
  @DisplayName("fuseCorrelatedObservations")
  class FuseCorrelatedObservationsTests {

    @org.junit.jupiter.api.Test
    @DisplayName("Empty list returns empty")
    void emptyList() {
      var observations = new ArrayList<TestedObservation>();
      var result = filter.fuseCorrelatedObservations(observations);
      assertTrue(result.isEmpty());
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Single observation returns single fused with cameraCount=1")
    void singleObservation() {
      var observations = new ArrayList<TestedObservation>();
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.5));

      var result = filter.fuseCorrelatedObservations(observations);

      assertEquals(1, result.size());
      assertEquals(1, result.get(0).cameraCount());
      assertEquals(0.5, result.get(0).score(), 0.001);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Two agreeing cameras fuse into one observation")
    void twoAgreeingCamerasFuse() {
      var observations = new ArrayList<TestedObservation>();
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.5));
      observations.add(makeTestedObs(8.1, 4.1, 0.01, 1, 0.6));

      var result = filter.fuseCorrelatedObservations(observations);

      assertEquals(1, result.size(), "Two agreeing cameras should fuse into one");
      assertEquals(2, result.get(0).cameraCount());
      // Score should be boosted (max * boostFactor, capped at 1.0)
      assertTrue(result.get(0).score() > 0.6, "Fused score should be boosted");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Fused pose is weighted average of inputs")
    void fusedPoseIsWeightedAverage() {
      var observations = new ArrayList<TestedObservation>();
      // Camera 0: x=8.0, score=0.6 (within correlation threshold of ~0.15m)
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.6));
      // Camera 1: x=8.1, score=0.4 (0.1m apart, within threshold)
      observations.add(makeTestedObs(8.1, 4.0, 0.01, 1, 0.4));

      var result = filter.fuseCorrelatedObservations(observations);

      assertEquals(1, result.size(), "Should fuse into one observation");
      // Weighted average: (8.0*0.6 + 8.1*0.4) / (0.6+0.4) = 8.04
      assertEquals(8.04, result.get(0).pose().getX(), 0.01);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Same camera multiple observations not fused")
    void sameCameraNotFused() {
      var observations = new ArrayList<TestedObservation>();
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.5));
      observations.add(makeTestedObs(8.05, 4.05, 0.01, 0, 0.5)); // Same camera

      var result = filter.fuseCorrelatedObservations(observations);

      assertEquals(2, result.size(), "Same camera observations should not fuse");
      assertEquals(1, result.get(0).cameraCount());
      assertEquals(1, result.get(1).cameraCount());
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Cameras too far apart in time not fused")
    void tooFarInTimeNotFused() {
      var observations = new ArrayList<TestedObservation>();
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.5));
      observations.add(makeTestedObs(8.05, 4.05, 0.2, 1, 0.5)); // 200ms apart (> 150ms threshold)

      var result = filter.fuseCorrelatedObservations(observations);

      assertEquals(2, result.size(), "Observations too far apart in time should not fuse");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Cameras too far apart in position not fused")
    void tooFarInPositionNotFused() {
      var observations = new ArrayList<TestedObservation>();
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.5));
      observations.add(makeTestedObs(9.0, 5.0, 0.01, 1, 0.5)); // 1.4m apart

      var result = filter.fuseCorrelatedObservations(observations);

      assertEquals(2, result.size(), "Observations too far apart should not fuse");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("2v2 split creates two fused observations")
    void twoVsTwoSplitCreatesTwoClusters() {
      var observations = new ArrayList<TestedObservation>();
      // Cameras 0,1 agree on position A
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.5));
      observations.add(makeTestedObs(8.05, 4.05, 0.01, 1, 0.5));
      // Cameras 2,3 agree on position B
      observations.add(makeTestedObs(10.0, 6.0, 0.0, 2, 0.5));
      observations.add(makeTestedObs(10.05, 6.05, 0.01, 3, 0.5));

      var result = filter.fuseCorrelatedObservations(observations);

      assertEquals(2, result.size(), "2v2 split should create 2 fused observations");
      // Both should have 2 cameras each
      for (var fused : result) {
        assertEquals(2, fused.cameraCount());
      }
    }

    @org.junit.jupiter.api.Test
    @DisplayName("3v1: three cameras fuse, outlier separate")
    void threeVsOneFuses() {
      var observations = new ArrayList<TestedObservation>();
      // Cameras 0,1,2 agree
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.5));
      observations.add(makeTestedObs(8.05, 4.05, 0.01, 1, 0.5));
      observations.add(makeTestedObs(8.03, 4.03, 0.02, 2, 0.5));
      // Camera 3 disagrees
      observations.add(makeTestedObs(10.0, 6.0, 0.0, 3, 0.5));

      var result = filter.fuseCorrelatedObservations(observations);

      assertEquals(2, result.size(), "Should have 2 fused observations");
      // Find the fused one with 3 cameras and the single one
      FusedObservation fusedCluster = null, singleObs = null;
      for (var f : result) {
        if (f.cameraCount() == 3) fusedCluster = f;
        if (f.cameraCount() == 1) singleObs = f;
      }
      assertNotNull(fusedCluster, "Should have a 3-camera cluster");
      assertNotNull(singleObs, "Should have a single camera observation");
      assertTrue(
          fusedCluster.score() > singleObs.score(), "Fused cluster should have higher score");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("4 cameras all agree fuse into one")
    void fourAllAgreeFuse() {
      var observations = new ArrayList<TestedObservation>();
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.5));
      observations.add(makeTestedObs(8.05, 4.05, 0.01, 1, 0.5));
      observations.add(makeTestedObs(8.03, 4.03, 0.02, 2, 0.5));
      observations.add(makeTestedObs(8.07, 4.02, 0.03, 3, 0.5));

      var result = filter.fuseCorrelatedObservations(observations);

      assertEquals(1, result.size(), "4 agreeing cameras should fuse into one");
      assertEquals(4, result.get(0).cameraCount());
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Fused score caps at 1.0")
    void fusedScoreCapsAtOne() {
      var observations = new ArrayList<TestedObservation>();
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.9));
      observations.add(makeTestedObs(8.05, 4.05, 0.01, 1, 0.9));

      var result = filter.fuseCorrelatedObservations(observations);

      assertTrue(result.get(0).score() <= 1.0, "Score should not exceed 1.0");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Edge of time window: exactly at threshold fuses")
    void edgeOfTimeWindowFuses() {
      double threshold = VisionConstants.correlationTimeWindowSeconds;
      var observations = new ArrayList<TestedObservation>();
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.5));
      observations.add(makeTestedObs(8.05, 4.05, threshold, 1, 0.5)); // Exactly at threshold

      var result = filter.fuseCorrelatedObservations(observations);

      // At exactly threshold, should still be within window (<=)
      assertEquals(1, result.size(), "At time threshold should still fuse");
      assertEquals(2, result.get(0).cameraCount());
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Edge of position threshold: exactly at threshold not fused")
    void edgeOfPositionThresholdNotFused() {
      double threshold = VisionConstants.correlationPoseThresholdMeters;
      var observations = new ArrayList<TestedObservation>();
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.5));
      // Exactly at threshold distance
      observations.add(makeTestedObs(8.0 + threshold, 4.0, 0.01, 1, 0.5));

      var result = filter.fuseCorrelatedObservations(observations);

      // At exactly threshold, should NOT fuse (uses >)
      assertEquals(2, result.size(), "At position threshold should not fuse");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Just under position threshold fuses")
    void justUnderPositionThresholdFuses() {
      double threshold = VisionConstants.correlationPoseThresholdMeters;
      var observations = new ArrayList<TestedObservation>();
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.5));
      observations.add(makeTestedObs(8.0 + threshold - 0.01, 4.0, 0.01, 1, 0.5));

      var result = filter.fuseCorrelatedObservations(observations);

      assertEquals(1, result.size(), "Just under threshold should fuse");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Transitive clustering: A-B and B-C agree, all fuse")
    void transitiveClusteringFuses() {
      double dist = VisionConstants.correlationPoseThresholdMeters - 0.01;
      var observations = new ArrayList<TestedObservation>();
      // A at origin
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.5));
      // B close to A
      observations.add(makeTestedObs(8.0 + dist, 4.0, 0.01, 1, 0.5));
      // C close to B, but farther from A
      observations.add(makeTestedObs(8.0 + dist * 1.5, 4.0, 0.02, 2, 0.5));

      var result = filter.fuseCorrelatedObservations(observations);

      // All should be in same cluster via transitivity
      assertEquals(1, result.size(), "Transitive clustering should fuse all");
      assertEquals(3, result.get(0).cameraCount());
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Exceeding MAX_OBSERVATIONS returns unfused")
    void exceedingMaxObservationsReturnsUnfused() {
      var observations = new ArrayList<TestedObservation>();
      for (int i = 0; i < 40; i++) { // More than MAX_OBSERVATIONS (32)
        observations.add(makeTestedObs(8.0, 4.0, 0.0, i % 4, 0.5));
      }

      var result = filter.fuseCorrelatedObservations(observations);

      // Should return individual observations without fusion
      assertEquals(40, result.size());
      for (var fused : result) {
        assertEquals(1, fused.cameraCount());
        assertEquals(0.5, fused.score(), 0.001);
      }
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Multiple observations from same camera in cluster still fuses")
    void multipleSameCameraInClusterFuses() {
      var observations = new ArrayList<TestedObservation>();
      // Camera 0: two observations at same position
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.5));
      observations.add(makeTestedObs(8.01, 4.01, 0.01, 0, 0.5));
      // Camera 1: one observation
      observations.add(makeTestedObs(8.02, 4.02, 0.02, 1, 0.5));

      var result = filter.fuseCorrelatedObservations(observations);

      // Should fuse since there are 2 distinct cameras
      assertEquals(1, result.size());
      assertEquals(2, result.get(0).cameraCount());
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Fused timestamp is weighted average")
    void fusedTimestampIsWeightedAverage() {
      var observations = new ArrayList<TestedObservation>();
      // Camera 0: time=0.0, score=0.6
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.6));
      // Camera 1: time=0.04, score=0.4
      observations.add(makeTestedObs(8.05, 4.0, 0.04, 1, 0.4));

      var result = filter.fuseCorrelatedObservations(observations);

      // Weighted average: (0.0*0.6 + 0.04*0.4) / (0.6+0.4) = 0.016
      assertEquals(0.016, result.get(0).timestamp(), 0.001);
    }
  }

  // ==================== NormalizedSigmoid Tests ====================

  @Nested
  @DisplayName("normalizedSigmoid")
  class NormalizedSigmoidTests {

    @org.junit.jupiter.api.Test
    @DisplayName("At midpoint returns 0.5")
    void atMidpointReturnsHalf() {
      assertEquals(0.5, VisionFilter.normalizedSigmoid(5.0, 5.0, 1.0), 0.001);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Far below midpoint returns near 0")
    void farBelowMidpointReturnsNearZero() {
      double result = VisionFilter.normalizedSigmoid(0.0, 5.0, 1.0);
      assertTrue(result < 0.01);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Far above midpoint returns near 1")
    void farAboveMidpointReturnsNearOne() {
      double result = VisionFilter.normalizedSigmoid(10.0, 5.0, 1.0);
      assertTrue(result > 0.99);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Higher steepness = sharper transition")
    void higherSteepnessSharper() {
      double shallowBelow = VisionFilter.normalizedSigmoid(4.0, 5.0, 1.0);
      double steepBelow = VisionFilter.normalizedSigmoid(4.0, 5.0, 4.0);

      assertTrue(
          steepBelow < shallowBelow,
          "Steeper should be lower below midpoint: steep="
              + steepBelow
              + " shallow="
              + shallowBelow);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Zero steepness = 0.5 everywhere")
    void zeroSteepness() {
      assertEquals(0.5, VisionFilter.normalizedSigmoid(0.0, 5.0, 0.0), 0.001);
      assertEquals(0.5, VisionFilter.normalizedSigmoid(5.0, 5.0, 0.0), 0.001);
      assertEquals(0.5, VisionFilter.normalizedSigmoid(10.0, 5.0, 0.0), 0.001);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Negative steepness inverts curve")
    void negativeSteepness() {
      double positive = VisionFilter.normalizedSigmoid(7.0, 5.0, 1.0);
      double negative = VisionFilter.normalizedSigmoid(7.0, 5.0, -1.0);

      assertTrue(positive > 0.5);
      assertTrue(negative < 0.5);
      assertEquals(1.0, positive + negative, 0.001, "Should be symmetric");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Output always between 0 and 1")
    void outputAlwaysBounded() {
      for (double x = -100; x <= 100; x += 10) {
        for (double mid = -10; mid <= 10; mid += 5) {
          for (double steep = 0.1; steep <= 10; steep += 2) {
            double result = VisionFilter.normalizedSigmoid(x, mid, steep);
            assertTrue(
                result >= 0.0 && result <= 1.0,
                "Result should be in [0,1]: "
                    + result
                    + " for x="
                    + x
                    + " mid="
                    + mid
                    + " steep="
                    + steep);
          }
        }
      }
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Handles extreme values without overflow")
    void handlesExtremeValues() {
      assertDoesNotThrow(() -> VisionFilter.normalizedSigmoid(1e10, 0.0, 1.0));
      assertDoesNotThrow(() -> VisionFilter.normalizedSigmoid(-1e10, 0.0, 1.0));

      double veryHigh = VisionFilter.normalizedSigmoid(1e10, 0.0, 1.0);
      double veryLow = VisionFilter.normalizedSigmoid(-1e10, 0.0, 1.0);

      assertEquals(1.0, veryHigh, 0.001);
      assertEquals(0.0, veryLow, 0.001);
    }
  }

  // ==================== TestContext Tests ====================

  @Nested
  @DisplayName("TestContext")
  class TestContextTests {

    @org.junit.jupiter.api.Test
    @DisplayName("Builder pattern returns same instance")
    void builderPatternReturnsSameInstance() {
      var ctx = new TestContext();
      var result = ctx.observation(makeObservation(0, 0, 0));
      assertSame(ctx, result);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("All setters and getters work")
    void allSettersAndGettersWork() {
      var obs = makeObservation(8.0, 4.0, 1.0);
      var pose = new Pose2d(1, 2, new Rotation2d());

      var ctx =
          new TestContext()
              .observation(obs)
              .cameraIndex(3)
              .lastAcceptedPose(pose)
              .lastAcceptedTimestamp(0.5);

      assertSame(obs, ctx.observation());
      assertEquals(3, ctx.cameraIndex());
      assertSame(pose, ctx.lastAcceptedPose());
      assertEquals(0.5, ctx.lastAcceptedTimestamp(), 0.001);
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Can set null values")
    void canSetNullValues() {
      var ctx = new TestContext().observation(null).lastAcceptedPose(null);

      assertNull(ctx.observation());
      assertNull(ctx.lastAcceptedPose());
    }
  }

  // ==================== Integration / Edge Case Tests ====================

  @Nested
  @DisplayName("Integration and Edge Cases")
  class IntegrationTests {

    @org.junit.jupiter.api.Test
    @DisplayName("Perfect observation scores well")
    void perfectObservationScoresWell() {
      // Center of field, no errors, low ambiguity, close tags, multiple tags
      var obs = makeObservation(8, 4, 0, 0, 0, 0, 0.0, 3, 0.01, 2.0);
      var tested = score(obs);

      // Even "perfect" observations don't score 1.0 due to sigmoid gradients and the
      // velocityUncertainScore penalty (score() passes null history). But they should
      // still score well above minScore — a good observation needs to be accepted even
      // when it's the first one from a camera.
      assertTrue(
          tested.score() > VisionConstants.minScore,
          "Perfect observation should score above minScore, got " + tested.score());
      assertTrue(
          tested.score() > 0.6, "Perfect observation should score > 0.6, got " + tested.score());
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Worst valid observation still passes if in bounds")
    void worstValidObservation() {
      // Just inside field, max ambiguity, far tags, some tilt
      var obs =
          makeObservation(
              ROBOT_HALF_WIDTH + 0.1,
              ROBOT_HALF_WIDTH + 0.1,
              0.2,
              Math.toRadians(4),
              Math.toRadians(4),
              0,
              0.0,
              1,
              0.14,
              5.0);
      var tested = score(obs);

      assertTrue(tested.score() > 0, "Should have non-zero score, got " + tested.score());
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Filter is reusable (no state leakage)")
    void filterIsReusable() {
      var obs1 = makeObservation(8.0, 4.0, 0.0);
      var obs2 = makeObservation(8.5, 4.5, 0.1);

      var tested1a = score(obs1);
      var tested2 = score(obs2);
      var tested1b = score(obs1);

      assertEquals(
          tested1a.score(),
          tested1b.score(),
          0.001,
          "Same observation should get same score on repeated calls");
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Fusion is deterministic")
    void fusionIsDeterministic() {
      var observations = new ArrayList<TestedObservation>();
      observations.add(makeTestedObs(8.0, 4.0, 0.0, 0, 0.5));
      observations.add(makeTestedObs(8.05, 4.05, 0.01, 1, 0.5));

      var result1 = filter.fuseCorrelatedObservations(observations);
      var result2 = filter.fuseCorrelatedObservations(observations);

      // Same input should produce same output
      assertEquals(result1.size(), result2.size());
      assertEquals(result1.get(0).score(), result2.get(0).score(), 0.001);
      assertEquals(result1.get(0).cameraCount(), result2.get(0).cameraCount());
    }

    @org.junit.jupiter.api.Test
    @DisplayName("All default tests are applied")
    void allDefaultTestsApplied() {
      var obs = makeObservation(8.0, 4.0, 0.0);
      var tested =
          filter.scoreObservation(obs, 0, null, 0.0, null, VisionFilter.DEFAULT_ENABLED_TESTS);

      for (var test : VisionFilter.DEFAULT_ENABLED_TESTS) {
        assertTrue(
            tested.testResults().containsKey(test),
            "Test " + test.name() + " should be in results");
      }
    }

    @org.junit.jupiter.api.Test
    @DisplayName("Empty test set produces NaN (edge case)")
    void emptyTestSetEdgeCase() {
      var obs = makeObservation(8.0, 4.0, 0.0);
      var tested = filter.scoreObservation(obs, 0, null, 0.0, null, EnumSet.noneOf(Test.class));

      // With no tests, geometric mean is undefined (0^0 or similar)
      // Document actual behavior
      assertTrue(
          Double.isNaN(tested.score()) || tested.score() == 1.0,
          "Empty test set should produce NaN or 1.0");
    }
  }
}
