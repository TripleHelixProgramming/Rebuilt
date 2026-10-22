package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Checks how the odometry thread decides which Phoenix signals are fresh and what timestamp a
 * sample gets, and how ModuleIOTalonFX turns stale values into invalid samples. These run only on
 * the robot, so replays of match logs never exercise them.
 */
class PhoenixOdometryThreadTest {
  private static final boolean[] ALL_VALID = filled(8, true);

  private static boolean[] filled(int length, boolean value) {
    var array = new boolean[length];
    Arrays.fill(array, value);
    return array;
  }

  private static double[] ages(double... ages) {
    return ages;
  }

  private static boolean[] fresh(double[] ages, boolean[] timestampValid) {
    var fresh = new boolean[ages.length];
    PhoenixOdometryThread.markFresh(ages, timestampValid, fresh);
    return fresh;
  }

  @Test
  @DisplayName("Healthy signals are fresh, even when the thread was held up")
  void healthySignalsAreFresh() {
    assertArrayEquals(
        filled(8, true),
        fresh(ages(0.004, 0.003, 0.004, 0.005, 0.004, 0.003, 0.004, 0.004), ALL_VALID));
    // A 76 ms delay in the thread ages every signal equally
    assertArrayEquals(
        filled(8, true),
        fresh(ages(0.080, 0.079, 0.080, 0.081, 0.080, 0.079, 0.080, 0.080), ALL_VALID));
  }

  @Test
  @DisplayName("A device that stopped updating is stale")
  void laggingDeviceIsStale() {
    var fresh = fresh(ages(0.004, 0.004, 0.004, 0.004, 0.004, 0.004, 0.004, 0.120), ALL_VALID);
    assertArrayEquals(new boolean[] {true, true, true, true, true, true, true, false}, fresh);
  }

  @Test
  @DisplayName("A timestamp in the future is stale and doesn't make healthy signals stale")
  void futureTimestampIsStale() {
    // VACHE session 19 had timestamps up to 2.7e7 s ahead, i.e. hugely negative ages
    var fresh = fresh(ages(0.004, 0.004, 0.004, 0.004, 0.004, 0.004, 0.004, -2.2e8), ALL_VALID);
    assertArrayEquals(new boolean[] {true, true, true, true, true, true, true, false}, fresh);
  }

  @Test
  @DisplayName("Every signal is stale when the whole bus stops")
  void deadBusIsStale() {
    // When the CANivore left USB at VACHE, every age grew to minutes
    assertArrayEquals(
        filled(8, false),
        fresh(ages(585.0, 585.0, 585.0, 585.0, 585.0, 585.0, 585.0, 585.0), ALL_VALID));
  }

  @Test
  @DisplayName("A signal with an invalid timestamp is stale")
  void invalidTimestampIsStale() {
    var timestampValid = filled(8, true);
    timestampValid[3] = false;
    var fresh = fresh(ages(0.004, 0.004, 0.004, 0.004, 0.004, 0.004, 0.004, 0.004), timestampValid);
    assertFalse(fresh[3]);
    assertTrue(fresh[0] && fresh[7]);
  }

  @Test
  @DisplayName("The timestamp leaves out stale signals and never runs ahead of the FPGA clock")
  void timestampUsesFreshSignalsOnly() {
    var ages = ages(0.004, 0.004, 0.004, 0.004, 0.004, 0.004, 0.004, 100.0);
    assertEquals(
        50.0 - 0.004,
        PhoenixOdometryThread.sampleTimestamp(
            50.0, ages, fresh(ages, ALL_VALID), Double.NEGATIVE_INFINITY),
        1e-12);
    assertEquals(
        50.0,
        PhoenixOdometryThread.sampleTimestamp(50.0, ages, new boolean[8], Double.NEGATIVE_INFINITY),
        0.0);
    // Slightly negative ages (clock jitter) can't put the timestamp in the future
    var early = ages(-0.01, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01);
    assertEquals(
        50.0,
        PhoenixOdometryThread.sampleTimestamp(
            50.0, early, fresh(early, ALL_VALID), Double.NEGATIVE_INFINITY),
        0.0);
  }

  @Test
  @DisplayName("One garbage timestamp doesn't push later timestamps into the future")
  void garbageTimestampDoesNotStick() {
    double previous = Double.NEGATIVE_INFINITY;
    var samples =
        List.of(
            ages(0.004, 0.004, 0.004, 0.004, 0.004, 0.004, 0.004, 0.004),
            ages(0.004, 0.004, 0.004, 0.004, 0.004, 0.004, 0.004, -2.2e8),
            ages(0.004, 0.004, 0.004, 0.004, 0.004, 0.004, 0.004, 0.004));
    for (int k = 0; k < samples.size(); k++) {
      double fpga = 100.0 + 0.004 * k;
      var ages = samples.get(k);
      double timestamp =
          PhoenixOdometryThread.sampleTimestamp(fpga, ages, fresh(ages, ALL_VALID), previous);
      assertEquals(fpga - 0.004, timestamp, 1e-9, "Sample " + k);
      previous = timestamp;
    }
  }

  @Test
  @DisplayName("Stale (NaN) values become invalid samples that repeat the last good value")
  void staleValuesBecomeInvalidSamples() {
    var queue = new ArrayDeque<Double>(List.of(1.0, Double.NaN, 2.0, Double.NaN));
    var values = new double[4];
    var valid = new boolean[4];
    double lastGood = ModuleIOTalonFX.drainOdometryQueue(queue, values, valid, 0.5);
    assertArrayEquals(new double[] {1.0, 1.0, 2.0, 2.0}, values);
    assertArrayEquals(new boolean[] {true, false, true, false}, valid);
    assertEquals(2.0, lastGood);

    var startsStale = new ArrayDeque<Double>(List.of(Double.NaN, 3.0));
    var values2 = new double[2];
    var valid2 = new boolean[2];
    ModuleIOTalonFX.drainOdometryQueue(startsStale, values2, valid2, lastGood);
    assertArrayEquals(new double[] {2.0, 3.0}, values2);
    assertArrayEquals(new boolean[] {false, true}, valid2);
  }
}
