package frc.robot.subsystems.leds;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

class LEDControllerTest {

  // Pose-seek colors, robot-relative. White on any series means within tolerance.
  private static final Color FORWARD = Color.kGreen; // POSE_X
  private static final Color BACKWARD = Color.kRed; // POSE_X
  private static final Color LEFT = Color.kRed; // POSE_Y
  private static final Color RIGHT = Color.kGreen; // POSE_Y
  private static final Color ON_TARGET = Color.kWhite;

  private static LEDController leds;

  @BeforeAll
  static void setUp() {
    assertTrue(HAL.initialize(500, 0));
    leds = LEDController.getInstance();
  }

  // ==================== Helper Methods ====================

  /** Asserts that every LED in the series shows the expected color. */
  private static void assertSeriesColor(LEDSeries series, Color expected) {
    var expectedHex = new Color8Bit(expected).toHexString();
    for (int i = 0; i < series.getLength(); i++) {
      assertEquals(
          expectedHex, series.getLED8Bit(i).toHexString(), series.name() + " LED " + i + " color");
    }
  }

  /** Returns the pose reached by moving from the given pose in its own frame. */
  private static Pose2d offset(Pose2d pose, double ahead, double left, double degreesCCW) {
    return pose.transformBy(new Transform2d(ahead, left, Rotation2d.fromDegrees(degreesCCW)));
  }

  // ==================== Pose Seek ====================

  @Nested
  @DisplayName("displayPoseSeek")
  class PoseSeekTests {

    /**
     * Auto start poses face 0, +/-90 and 180 degrees. At each heading, and at one oblique heading,
     * the target is placed 1 m from the robot in each robot-relative direction.
     */
    static Stream<Arguments> robotRelativeTargets() {
      Stream.Builder<Arguments> cases = Stream.builder();
      for (double heading : new double[] {0.0, 90.0, -90.0, 180.0, 30.0}) {
        cases.add(Arguments.of(heading, 1.0, 0.0, FORWARD, ON_TARGET));
        cases.add(Arguments.of(heading, -1.0, 0.0, BACKWARD, ON_TARGET));
        cases.add(Arguments.of(heading, 0.0, 1.0, ON_TARGET, LEFT));
        cases.add(Arguments.of(heading, 0.0, -1.0, ON_TARGET, RIGHT));
      }
      return cases.build();
    }

    @ParameterizedTest(name = "heading {0} deg, target {1} m ahead and {2} m left")
    @MethodSource("robotRelativeTargets")
    @DisplayName("Translation lights point toward the target in the robot's frame")
    void translationLightsUseRobotFrame(
        double headingDegrees, double ahead, double left, Color expectedX, Color expectedY) {
      var robot = new Pose2d(4.0, 3.0, Rotation2d.fromDegrees(headingDegrees));

      leds.displayPoseSeek(robot, offset(robot, ahead, left, 0.0));

      assertSeriesColor(LEDSeries.POSE_X, expectedX);
      assertSeriesColor(LEDSeries.POSE_Y, expectedY);
      assertSeriesColor(LEDSeries.POSE_ROTATION, ON_TARGET);
    }

    @Test
    @DisplayName("Rotation lights: X green and Y red to rotate counterclockwise")
    void counterclockwise() {
      var robot = new Pose2d(4.0, 3.0, Rotation2d.fromDegrees(-90.0));

      leds.displayPoseSeek(robot, offset(robot, 0.0, 0.0, 10.0));

      assertSeriesColor(LEDSeries.POSE_ROTATION_X, Color.kGreen);
      assertSeriesColor(LEDSeries.POSE_ROTATION_Y, Color.kRed);
    }

    @Test
    @DisplayName("Rotation lights: X red and Y green to rotate clockwise")
    void clockwise() {
      var robot = new Pose2d(4.0, 3.0, Rotation2d.fromDegrees(-90.0));

      leds.displayPoseSeek(robot, offset(robot, 0.0, 0.0, -10.0));

      assertSeriesColor(LEDSeries.POSE_ROTATION_X, Color.kRed);
      assertSeriesColor(LEDSeries.POSE_ROTATION_Y, Color.kGreen);
    }
  }
}
