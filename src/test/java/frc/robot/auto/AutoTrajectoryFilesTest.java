package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.*;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.hal.HAL;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.regex.Pattern;
import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * Checks that every trajectory an auto routine asks for can be loaded from src/main/deploy/choreo.
 * ChoreoLib replaces a missing trajectory with an empty one, and resetOdometry() then kills the
 * routine, so a misspelled name compiles and deploys but the auto does nothing.
 */
class AutoTrajectoryFilesTest {

  private static final Path AUTO_SOURCES = Path.of("src/main/java/frc/robot/auto");
  private static final Pattern TRAJECTORY_CALL = Pattern.compile("\\.trajectory\\(\"([^\"]+)\"");

  @BeforeAll
  static void setUp() {
    assertTrue(HAL.initialize(500, 0));
  }

  /** Returns (source file, trajectory name) for every routine.trajectory("...") call. */
  static Stream<Arguments> trajectoryReferences() throws IOException {
    List<Arguments> references = new ArrayList<>();
    try (var files = Files.list(AUTO_SOURCES)) {
      for (var file : files.filter(f -> f.toString().endsWith(".java")).sorted().toList()) {
        var matcher = TRAJECTORY_CALL.matcher(Files.readString(file));
        while (matcher.find()) {
          references.add(Arguments.of(file.getFileName().toString(), matcher.group(1)));
        }
      }
    }
    assertFalse(references.isEmpty(), "No trajectory references found in " + AUTO_SOURCES);
    return references.stream();
  }

  @ParameterizedTest(name = "{0}: {1}")
  @MethodSource("trajectoryReferences")
  void trajectoryLoadsWithSamples(String sourceFile, String trajectoryName) {
    Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory(trajectoryName);

    assertTrue(
        trajectory.isPresent(),
        sourceFile + " loads a trajectory with no file: " + trajectoryName + ".traj");
    assertFalse(trajectory.get().samples().isEmpty(), trajectoryName + ".traj has no samples");
  }
}
