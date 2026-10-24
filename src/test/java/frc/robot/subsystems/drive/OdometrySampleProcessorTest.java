package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Feeds OdometrySampleProcessor samples from a simulated robot whose true pose is known, including
 * the failures seen in VACHE and October logs, and integrates the output the way the pose estimator
 * does.
 */
class OdometrySampleProcessorTest {
  private static final Translation2d[] MODULES = DriveConstants.MODULE_TRANSLATIONS;
  private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULES);
  private static final double DT = 0.004; // 250 Hz odometry
  private static final boolean[] ALL_VALID = {true, true, true, true};

  /** The real robot: its pose, each module's position, and the time. */
  private static final class Robot {
    Pose2d pose = Pose2d.kZero;
    final double[] distances = new double[4];
    final Rotation2d[] angles = {
      Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero
    };
    double time = 100.0;

    /** Moves for one sample period at robot-relative speeds. */
    void step(double vx, double vy, double omega) {
      var states = KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(vx, vy, omega));
      for (int i = 0; i < 4; i++) {
        distances[i] += states[i].speedMetersPerSecond * DT;
        if (states[i].speedMetersPerSecond > 1e-9) {
          angles[i] = states[i].angle;
        }
      }
      pose = pose.exp(new Twist2d(vx * DT, vy * DT, omega * DT));
      time += DT;
    }

    SwerveModulePosition[] positions() {
      var positions = new SwerveModulePosition[4];
      for (int i = 0; i < 4; i++) {
        positions[i] = new SwerveModulePosition(distances[i], angles[i]);
      }
      return positions;
    }
  }

  /** Odometry as Drive did it before: raw positions, rotation from the gyro or raw wheels. */
  private static final class OldOdometry {
    final SwerveDriveOdometry odometry;
    SwerveModulePosition[] last = new Robot().positions();
    Rotation2d rotation = Rotation2d.kZero;

    OldOdometry(Pose2d start) {
      odometry = new SwerveDriveOdometry(KINEMATICS, Rotation2d.kZero, last, start);
    }

    void update(SwerveModulePosition[] raw, Rotation2d gyro) {
      if (gyro != null) {
        rotation = gyro;
      } else {
        var deltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
          deltas[i] =
              new SwerveModulePosition(
                  raw[i].distanceMeters - last[i].distanceMeters, raw[i].angle);
        }
        rotation = rotation.plus(new Rotation2d(KINEMATICS.toTwist2d(deltas).dtheta));
      }
      last = raw;
      odometry.update(rotation, raw);
    }
  }

  private Robot robot;
  private OdometrySampleProcessor processor;
  private SwerveDriveOdometry odometry; // Integrates processed samples like the estimator
  private OldOdometry naive; // Integrates the raw samples, as Drive did before

  @BeforeEach
  void setUp() {
    robot = new Robot();
    processor = new OdometrySampleProcessor(MODULES);
    odometry =
        new SwerveDriveOdometry(
            KINEMATICS, Rotation2d.kZero, processor.lastPositions(), robot.pose);
    naive = new OldOdometry(robot.pose);
  }

  /** Processes one sample stamped at the robot's time, with the gyro if given. */
  private OdometrySampleProcessor.Sample feed(
      SwerveModulePosition[] raw, boolean[] valid, boolean gyroConnected) {
    Rotation2d gyro = gyroConnected ? robot.pose.getRotation() : null;
    var sample = processor.process(robot.time, robot.time + 0.01, raw, valid, gyro);
    odometry.update(sample.gyroRotation(), sample.modulePositions());
    naive.update(raw, gyro);
    return sample;
  }

  private double translationError(SwerveDriveOdometry odometry) {
    return odometry.getPoseMeters().getTranslation().getDistance(robot.pose.getTranslation());
  }

  private double translationError(OldOdometry old) {
    return translationError(old.odometry);
  }

  @Test
  @DisplayName("Healthy samples pass through unchanged")
  void healthySamplesPassThrough() {
    for (int k = 0; k < 500; k++) {
      robot.step(2.0, 0.5, 1.0);
      var raw = robot.positions();
      var sample = feed(raw, ALL_VALID, true);
      assertEquals(robot.time, sample.timestamp(), 0.0);
      for (int i = 0; i < 4; i++) {
        assertEquals(raw[i].distanceMeters, sample.modulePositions()[i].distanceMeters, 0.0);
        assertEquals(raw[i].angle, sample.modulePositions()[i].angle);
      }
    }
    assertEquals(0, processor.rebaseCount());
    assertEquals(0, processor.substitutedTimestampCount());
    assertEquals(4, processor.validModuleCount());
    assertEquals(0.0, translationError(odometry), 1e-9);
  }

  @Test
  @DisplayName("A reset to zero is absorbed, not driven (VACHE Q27)")
  void resetToZeroIsAbsorbed() {
    for (int k = 0; k < 250; k++) {
      robot.step(2.0, 0.0, 0.0);
      feed(robot.positions(), ALL_VALID, true);
    }
    // Every device reports 0.0 before the connection flags notice; the robot has stopped
    var reset = robot.positions();
    for (int i = 0; i < 4; i++) {
      reset[i] = new SwerveModulePosition(0.0, Rotation2d.kZero);
    }
    for (int k = 0; k < 100; k++) {
      robot.step(0.0, 0.0, 0.0);
      feed(reset, ALL_VALID, true);
    }
    assertEquals(4, processor.rebaseCount());
    assertEquals(0.0, translationError(odometry), 0.01);
    assertTrue(translationError(naive) > 1.9, "Raw samples move the pose 2 m backward");
  }

  @Test
  @DisplayName("A module that stopped reporting is filled in from the others (VACHE Q32)")
  void frozenModuleIsFilledIn() {
    SwerveModulePosition held = null;
    for (int k = 0; k < 500; k++) {
      robot.step(3.0, 0.0, 0.0);
      var raw = robot.positions();
      if (k >= 100) {
        held = held == null ? raw[0] : held; // Stops reporting and holds its last value
        raw[0] = held;
      }
      feed(raw, k >= 100 ? new boolean[] {false, true, true, true} : ALL_VALID, true);
    }
    assertEquals(3, processor.validModuleCount());
    assertEquals(0.0, translationError(odometry), 0.02 * 6.0);
    assertTrue(translationError(naive) > 1.0, "A held wheel loses a quarter of the travel");
  }

  @Test
  @DisplayName("A module with a dead turn signal is left out (October 15)")
  void deadTurnSignalIsLeftOut() {
    boolean[] valid = {true, false, true, true};
    for (int k = 0; k < 500; k++) {
      robot.step(2.12, 2.12, 0.0); // 45 degrees
      var raw = robot.positions();
      raw[1] = new SwerveModulePosition(raw[1].distanceMeters, Rotation2d.kZero); // Angle stuck
      feed(raw, valid, true);
    }
    assertEquals(0.0, translationError(odometry), 0.02 * 6.0);
    assertTrue(translationError(naive) > 0.5, "The stuck angle bends the path");
  }

  @Test
  @DisplayName("A module that comes back at a new position causes no jump")
  void returningModuleCausesNoJump() {
    for (int k = 0; k < 600; k++) {
      robot.step(2.0, 0.0, 0.5);
      var raw = robot.positions();
      boolean out = k >= 100 && k < 225; // Silent for half a second
      if (k >= 225) {
        raw[2] = new SwerveModulePosition(raw[2].distanceMeters - 5.93, raw[2].angle);
      }
      feed(raw, out ? new boolean[] {true, true, false, true} : ALL_VALID, true);
    }
    assertEquals(0.0, translationError(odometry), 0.02);
  }

  @Test
  @DisplayName("Real motion across a 0.7 s stall is kept")
  void motionAcrossStallIsKept() {
    for (int k = 0; k < 100; k++) {
      robot.step(4.4, 0.0, 0.0);
      feed(robot.positions(), ALL_VALID, true);
    }
    for (int k = 0; k < 175; k++) {
      robot.step(4.4, 0.0, 0.0); // No samples arrive
    }
    feed(robot.positions(), ALL_VALID, true); // One sample carrying 3.1 m of travel
    assertEquals(0, processor.rebaseCount());
    assertEquals(0.0, translationError(odometry), 0.05);
  }

  @Test
  @DisplayName("Bad timestamps are replaced and the sequence stays strictly increasing")
  void badTimestampsAreReplaced() {
    double t = 200.0;
    double now = t + 0.02;
    double[] stamps = {
      t,
      Double.NaN,
      t + 0.004,
      Double.POSITIVE_INFINITY,
      t + 0.008,
      Double.NEGATIVE_INFINITY,
      t - 600.0, // Frozen far in the past (VACHE Q27)
      t + 2.7e7, // Garbage far in the future (VACHE session 19)
      t + 0.012 - 0.05, // Steps backward
      t + 0.016,
      t + 0.016 // Repeats
    };
    double previous = Double.NEGATIVE_INFINITY;
    for (double stamp : stamps) {
      var sample = processor.process(stamp, now, robot.positions(), ALL_VALID, Rotation2d.kZero);
      assertTrue(sample.timestamp() > previous, "Strictly increasing after " + previous);
      assertTrue(sample.timestamp() <= now + 0.1, "Not in the future");
      previous = sample.timestamp();
    }
    assertEquals(5, processor.substitutedTimestampCount());
  }

  @Test
  @DisplayName("Lagging timestamps keep the motion between samples (October 15, session 70)")
  void laggingTimestampsKeepMotion() {
    // A dead signal slowed the thread to a sample every 16 ms and dragged every timestamp 25 s
    // back, so all are replaced with the loop time. Each loop delivers four samples at once.
    // The first sample comes with the robot at rest, as at boot.
    var first =
        processor.process(
            robot.time - 25.0, robot.time, robot.positions(), ALL_VALID, Rotation2d.kZero);
    odometry.update(first.gyroRotation(), first.modulePositions());
    for (int loop = 0; loop < 100; loop++) {
      var batch = new SwerveModulePosition[4][];
      var stamps = new double[4];
      var headings = new Rotation2d[4];
      for (int s = 0; s < 4; s++) {
        for (int k = 0; k < 4; k++) {
          robot.step(4.0, 0.0, 0.0); // 6.4 cm per sample, more than the step margin
        }
        batch[s] = robot.positions();
        stamps[s] = robot.time - 25.0;
        headings[s] = robot.pose.getRotation();
      }
      double now = robot.time + 0.005;
      for (int s = 0; s < 4; s++) {
        var sample = processor.process(stamps[s], now, batch[s], ALL_VALID, headings[s]);
        odometry.update(sample.gyroRotation(), sample.modulePositions());
      }
    }
    assertEquals(401, processor.substitutedTimestampCount());
    assertEquals(0, processor.rebaseCount(), "No real motion is mistaken for a reset");
    assertEquals(0.0, translationError(odometry), 0.02);
  }

  @Test
  @DisplayName("Samples from a 0.7 s loop stall keep their timestamps")
  void stallSamplesKeepTimestamps() {
    double now = 300.0;
    for (int k = 0; k < 12; k++) {
      double stamp = k < 3 ? now - 0.72 + k * 0.004 : now - 0.04 + k * 0.004;
      var sample = processor.process(stamp, now, robot.positions(), ALL_VALID, Rotation2d.kZero);
      assertEquals(stamp, sample.timestamp(), 0.0);
    }
    assertEquals(0, processor.substitutedTimestampCount());
  }

  @Test
  @DisplayName("Without a gyro, a held module adds no false rotation")
  void noFalseRotationWithoutGyro() {
    boolean[] valid = {true, true, true, false};
    for (int k = 0; k < 500; k++) {
      robot.step(3.0, 0.0, 0.0);
      var raw = robot.positions();
      raw[3] = new SwerveModulePosition(0.0, Rotation2d.kZero);
      feed(raw, valid, false);
    }
    assertEquals(0.0, processor.gyroRotation().getRadians(), 1e-9);
    assertEquals(0.0, translationError(odometry), 0.02 * 6.0);
    assertTrue(
        Math.abs(naive.odometry.getPoseMeters().getRotation().getRadians()) > 0.5,
        "Raw samples integrate false rotation from the held wheel");
  }

  @Test
  @DisplayName("With every module out, the pose holds position and follows the gyro")
  void allModulesOut() {
    boolean[] none = {false, false, false, false};
    var frozen = robot.positions();
    for (int k = 0; k < 250; k++) {
      robot.step(0.0, 0.0, 1.0); // Spinning in place
      feed(frozen, none, true);
    }
    assertEquals(0, processor.validModuleCount());
    assertEquals(0.0, translationError(odometry), 0.01);
    assertEquals(
        robot.pose.getRotation().getRadians(),
        odometry.getPoseMeters().getRotation().getRadians(),
        1e-9);
  }

  @Test
  @DisplayName("Chassis speeds leave out a module that stopped reporting")
  void chassisSpeedsSkipInvalidModule() {
    robot.step(2.0, 0.0, 0.0);
    feed(robot.positions(), new boolean[] {false, true, true, true}, true);
    var states = KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(2.0, 0.0, 0.0));
    states[0] = new SwerveModuleState(-1.76, Rotation2d.kZero); // Frozen velocity
    for (Double gyroRate : new Double[] {0.0, null}) {
      var speeds = processor.chassisSpeeds(states, gyroRate);
      assertEquals(2.0, speeds.vxMetersPerSecond, 1e-9);
      assertEquals(0.0, speeds.vyMetersPerSecond, 1e-9);
      assertEquals(0.0, speeds.omegaRadiansPerSecond, 1e-9);
    }
  }

  @Test
  @DisplayName("Modules back from a short outage keep the travel they counted")
  void shortOutageKeepsTravel() {
    SwerveModulePosition[] held = null;
    for (int k = 0; k < 250; k++) {
      robot.step(4.0, 0.0, 0.0);
      var raw = robot.positions();
      boolean out = k >= 100 && k < 125; // 0.1 s with every signal stale
      if (out) {
        held = held == null ? raw : held; // Stale samples repeat the last good value
        raw = held;
      }
      feed(raw, out ? new boolean[4] : ALL_VALID, true);
    }
    assertEquals(0, processor.rebaseCount(), "The 0.4 m counted during the outage is kept");
    assertEquals(0.0, translationError(odometry), 0.02);
  }

  @Test
  @DisplayName("The first sample after the gyro reconnects adds no translation")
  void gyroReconnectAddsNoTranslation() {
    boolean[] valid = {true, false, true, true};
    var still = robot.positions();
    for (int k = 0; k < 50; k++) {
      robot.step(0.0, 0.0, 0.0);
      processor.process(robot.time, robot.time, still, valid, null); // Gyro out
    }
    // The gyro returns reading 0.5 rad from the wheel-integrated heading
    var sample =
        processor.process(robot.time, robot.time, still, valid, Rotation2d.fromRadians(0.5));
    odometry.update(sample.gyroRotation(), sample.modulePositions());
    assertEquals(0.0, translationError(odometry), 1e-9);
  }

  @Test
  @DisplayName("A garbage future timestamp doesn't loosen the wheel step bound")
  void futureTimestampKeepsBoundTight() {
    for (int k = 0; k < 100; k++) {
      robot.step(1.0, 0.0, 0.0);
      feed(robot.positions(), ALL_VALID, true);
    }
    var garbage = robot.positions();
    for (int i = 0; i < 4; i++) {
      garbage[i] = new SwerveModulePosition(garbage[i].distanceMeters + 3.0, garbage[i].angle);
    }
    var sample =
        processor.process(robot.time + 2.7e7, robot.time, garbage, ALL_VALID, Rotation2d.kZero);
    odometry.update(sample.gyroRotation(), sample.modulePositions());
    assertEquals(4, processor.rebaseCount(), "The 3 m jump is absorbed");
    assertEquals(0.0, translationError(odometry), 0.01);
  }

  @Test
  @DisplayName("A tiny estimated move keeps the module's angle")
  void tinyEstimatedMoveKeepsAngle() {
    boolean[] valid = {false, true, true, true};
    for (int k = 0; k < 10; k++) {
      robot.step(2.0, 2.0, 0.0); // 45 degrees, so the angle to keep isn't zero
      feed(robot.positions(), ALL_VALID, true);
    }
    var angle = processor.lastPositions()[0].angle;
    assertEquals(45.0, angle.getDegrees(), 1e-6);
    var creep = robot.positions();
    for (int i = 1; i < 4; i++) {
      creep[i] = new SwerveModulePosition(creep[i].distanceMeters + 1e-7, Rotation2d.kCW_90deg);
    }
    var sample = feed(creep, valid, true);
    assertEquals(angle, sample.modulePositions()[0].angle);
  }

  @Test
  @DisplayName("A pose reset seeds valid modules from their current positions")
  void resetSeedsFromCurrentPositions() {
    boolean[] valid = {true, true, false, true};
    for (int k = 0; k < 50; k++) {
      robot.step(2.0, 0.0, 0.0);
      feed(robot.positions(), valid, true);
    }
    var estimated = processor.lastPositions()[2];
    robot.step(2.0, 0.0, 0.0); // The current reading is newer than the last sample
    var seeded = processor.resetPositions(robot.positions());
    assertEquals(robot.distances[0], seeded[0].distanceMeters, 1e-12);
    assertEquals(estimated.distanceMeters, seeded[2].distanceMeters, 0.0);
    assertEquals(0, processor.rebaseCount());
  }

  @Test
  @DisplayName("Module layout is symmetric about the robot center")
  void moduleLayoutIsSymmetric() {
    // Filling in a module from the others relies on this (see OdometrySampleProcessor)
    double sumX = 0.0;
    double sumY = 0.0;
    for (var translation : MODULES) {
      sumX += translation.getX();
      sumY += translation.getY();
    }
    assertEquals(0.0, sumX, 1e-9);
    assertEquals(0.0, sumY, 1e-9);
  }
}
