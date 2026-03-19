package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.util.KernelLogMonitor.EventPattern;
import frc.robot.util.KernelLogMonitor.KernelEvent;
import java.lang.reflect.Method;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

/**
 * Comprehensive unit tests for KernelLogMonitor.
 *
 * <p>Note: Some tests use reflection to test private methods and access internal state, as this is
 * a singleton with system-level integration that's difficult to fully mock.
 */
@DisplayName("KernelLogMonitor Tests")
class KernelLogMonitorTest {

  @Nested
  @DisplayName("EventPattern Enum Tests")
  class EventPatternTests {

    @Test
    @DisplayName("USB_DISCONNECT pattern should match USB disconnect messages")
    void testUsbDisconnectPattern() {
      assertTrue(EventPattern.USB_DISCONNECT.matches("usb 1-1: USB disconnect, device number 5"));
      assertTrue(
          EventPattern.USB_DISCONNECT.matches(
              "kern  :err   : [12345.678] usb 1-1: USB disconnect"));
      assertTrue(EventPattern.USB_DISCONNECT.matches("USB DISCONNECT event detected"));
      assertFalse(EventPattern.USB_DISCONNECT.matches("can0: error detected"));
    }

    @Test
    @DisplayName("USB_RESET pattern should match USB reset messages")
    void testUsbResetPattern() {
      assertTrue(EventPattern.USB_RESET.matches("usb 1-1: reset high-speed USB device"));
      assertTrue(EventPattern.USB_RESET.matches("USB Reset occurred on port 2"));
      assertFalse(EventPattern.USB_RESET.matches("usb disconnect"));
    }

    @Test
    @DisplayName("I2C_ERROR pattern should match I2C error messages")
    void testI2cErrorPattern() {
      assertTrue(EventPattern.I2C_ERROR.matches("i2c i2c-0: error reading from device"));
      assertTrue(EventPattern.I2C_ERROR.matches("I2C ERROR: timeout"));
      assertTrue(EventPattern.I2C_ERROR.matches("i2c error detected on bus 1"));
      assertFalse(EventPattern.I2C_ERROR.matches("can error"));
    }

    @Test
    @DisplayName("CAN_ERROR pattern should match CAN bus error messages")
    void testCanErrorPattern() {
      assertTrue(EventPattern.CAN_ERROR.matches("can0: error-passive state"));
      assertTrue(EventPattern.CAN_ERROR.matches("CAN ERROR: bus off"));
      assertTrue(EventPattern.CAN_ERROR.matches("can bus error detected"));
      assertFalse(EventPattern.CAN_ERROR.matches("usb error"));
    }

    @Test
    @DisplayName("POWER_EVENT pattern should match power/voltage/brownout events")
    void testPowerEventPattern() {
      assertTrue(EventPattern.POWER_EVENT.matches("voltage drop detected"));
      assertTrue(EventPattern.POWER_EVENT.matches("power supply unstable"));
      assertTrue(EventPattern.POWER_EVENT.matches("brownout detected"));
      assertTrue(EventPattern.POWER_EVENT.matches("VOLTAGE below threshold"));
      assertTrue(EventPattern.POWER_EVENT.matches("POWER loss imminent"));
      assertFalse(EventPattern.POWER_EVENT.matches("can error"));
    }

    @Test
    @DisplayName("findMatch should return first matching pattern")
    void testFindMatchReturnsFirstMatch() {
      var result = EventPattern.findMatch("usb 1-1: USB disconnect");
      assertEquals(EventPattern.USB_DISCONNECT, result);

      result = EventPattern.findMatch("i2c error on bus 0");
      assertEquals(EventPattern.I2C_ERROR, result);

      result = EventPattern.findMatch("voltage drop detected");
      assertEquals(EventPattern.POWER_EVENT, result);
    }

    @Test
    @DisplayName("findMatch should return null for non-matching lines")
    void testFindMatchReturnsNullForNoMatch() {
      var result = EventPattern.findMatch("normal kernel log message");
      assertNull(result);

      result = EventPattern.findMatch("some random text");
      assertNull(result);
    }

    @Test
    @DisplayName("Pattern matching should be case-insensitive")
    void testPatternsAreCaseInsensitive() {
      assertTrue(EventPattern.USB_DISCONNECT.matches("USB DISCONNECT"));
      assertTrue(EventPattern.USB_DISCONNECT.matches("usb disconnect"));
      assertTrue(EventPattern.USB_DISCONNECT.matches("Usb DiScOnNeCt"));

      assertTrue(EventPattern.POWER_EVENT.matches("VOLTAGE"));
      assertTrue(EventPattern.POWER_EVENT.matches("voltage"));
      assertTrue(EventPattern.POWER_EVENT.matches("VoLtAgE"));
    }

    @Test
    @DisplayName("All enum values should be accessible")
    void testAllEnumValuesAccessible() {
      var values = EventPattern.values();
      assertEquals(5, values.length);
      assertNotNull(EventPattern.valueOf("USB_DISCONNECT"));
      assertNotNull(EventPattern.valueOf("USB_RESET"));
      assertNotNull(EventPattern.valueOf("I2C_ERROR"));
      assertNotNull(EventPattern.valueOf("CAN_ERROR"));
      assertNotNull(EventPattern.valueOf("POWER_EVENT"));
    }
  }

  @Nested
  @DisplayName("KernelEvent Record Tests")
  class KernelEventTests {

    @Test
    @DisplayName("KernelEvent should be created with all fields")
    void testKernelEventCreation() {
      var event = new KernelEvent("12345.678", "test message", "USB_DISCONNECT");

      assertEquals("12345.678", event.timestamp());
      assertEquals("test message", event.message());
      assertEquals("USB_DISCONNECT", event.eventType());
    }

    @Test
    @DisplayName("KernelEvent records with same values should be equal")
    void testKernelEventEquality() {
      var event1 = new KernelEvent("12345.678", "test message", "USB_DISCONNECT");
      var event2 = new KernelEvent("12345.678", "test message", "USB_DISCONNECT");

      assertEquals(event1, event2);
      assertEquals(event1.hashCode(), event2.hashCode());
    }

    @Test
    @DisplayName("KernelEvent records with different values should not be equal")
    void testKernelEventInequality() {
      var event1 = new KernelEvent("12345.678", "test message", "USB_DISCONNECT");
      var event2 = new KernelEvent("12345.679", "test message", "USB_DISCONNECT");
      var event3 = new KernelEvent("12345.678", "different message", "USB_DISCONNECT");
      var event4 = new KernelEvent("12345.678", "test message", "CAN_ERROR");

      assertNotEquals(event1, event2);
      assertNotEquals(event1, event3);
      assertNotEquals(event1, event4);
    }

    @Test
    @DisplayName("KernelEvent toString should include all fields")
    void testKernelEventToString() {
      var event = new KernelEvent("12345.678", "test message", "USB_DISCONNECT");
      var str = event.toString();

      assertTrue(str.contains("12345.678"));
      assertTrue(str.contains("test message"));
      assertTrue(str.contains("USB_DISCONNECT"));
    }
  }

  @Nested
  @DisplayName("Timestamp Extraction Tests")
  class TimestampExtractionTests {

    private Method extractTimestampMethod;

    @BeforeEach
    void setUp() throws Exception {
      // Use reflection to access private extractTimestamp method
      extractTimestampMethod =
          KernelLogMonitor.class.getDeclaredMethod("extractTimestamp", String.class);
      extractTimestampMethod.setAccessible(true);
    }

    @Test
    @DisplayName("Should extract timestamp from standard dmesg format")
    void testExtractTimestampStandard() throws Exception {
      var monitor = createMonitorInstance();
      var result =
          (String)
              extractTimestampMethod.invoke(
                  monitor, "kern  :warn  : [12345.678901] usb disconnect");

      assertEquals("12345.678901", result);
    }

    @Test
    @DisplayName("Should extract timestamp with different facility/level")
    void testExtractTimestampDifferentFormat() throws Exception {
      var monitor = createMonitorInstance();
      var result =
          (String)
              extractTimestampMethod.invoke(monitor, "user  :err   : [999.123456] error message");

      assertEquals("999.123456", result);
    }

    @Test
    @DisplayName("Should return 'unknown' when timestamp is missing")
    void testExtractTimestampMissing() throws Exception {
      var monitor = createMonitorInstance();
      var result = (String) extractTimestampMethod.invoke(monitor, "no timestamp here");

      assertEquals("unknown", result);
    }

    @Test
    @DisplayName("Should return 'unknown' for malformed timestamp")
    void testExtractTimestampMalformed() throws Exception {
      var monitor = createMonitorInstance();
      var result = (String) extractTimestampMethod.invoke(monitor, "kern :warn : [incomplete");

      assertEquals("unknown", result);
    }

    @Test
    @DisplayName("Should handle empty brackets")
    void testExtractTimestampEmptyBrackets() throws Exception {
      var monitor = createMonitorInstance();
      var result = (String) extractTimestampMethod.invoke(monitor, "kern :warn : [] message");

      assertEquals("", result);
    }
  }

  @Nested
  @DisplayName("Generic Event Categorization Tests")
  class GenericEventCategorizationTests {

    private Method categorizeGenericEventMethod;

    @BeforeEach
    void setUp() throws Exception {
      categorizeGenericEventMethod =
          KernelLogMonitor.class.getDeclaredMethod("categorizeGenericEvent", String.class);
      categorizeGenericEventMethod.setAccessible(true);
    }

    @ParameterizedTest
    @ValueSource(
        strings = {
          "kern  :err  : [123.456] some error",
          "user  :error: [789.012] another error",
          "some line with :err  : in it"
        })
    @DisplayName("Should categorize error level messages as KERNEL_ERROR")
    void testCategorizeAsKernelError(String line) throws Exception {
      var monitor = createMonitorInstance();
      var result = (String) categorizeGenericEventMethod.invoke(monitor, line);

      assertEquals("KERNEL_ERROR", result);
    }

    @ParameterizedTest
    @ValueSource(
        strings = {
          "kern  :warn : [123.456] warning message",
          "user  :warning: [789.012] another warning",
          "line with :warn : in it"
        })
    @DisplayName("Should categorize warning level messages as KERNEL_WARNING")
    void testCategorizeAsKernelWarning(String line) throws Exception {
      var monitor = createMonitorInstance();
      var result = (String) categorizeGenericEventMethod.invoke(monitor, line);

      assertEquals("KERNEL_WARNING", result);
    }

    @Test
    @DisplayName("Should categorize unknown level as KERNEL_EVENT")
    void testCategorizeAsKernelEvent() throws Exception {
      var monitor = createMonitorInstance();
      var result = (String) categorizeGenericEventMethod.invoke(monitor, "unknown format message");

      assertEquals("KERNEL_EVENT", result);
    }
  }

  @Nested
  @DisplayName("Counter Increment Tests")
  class CounterIncrementTests {

    private Method incrementCounterMethod;

    @BeforeEach
    void setUp() throws Exception {
      incrementCounterMethod =
          KernelLogMonitor.class.getDeclaredMethod(
              "incrementCounterForPattern", EventPattern.class);
      incrementCounterMethod.setAccessible(true);
    }

    @Test
    @DisplayName("USB_DISCONNECT should increment USB counter")
    void testIncrementUsbDisconnect() throws Exception {
      var monitor = createMonitorInstance();
      var usbCounterBefore = getCounterValue(monitor, "usbEvents");

      incrementCounterMethod.invoke(monitor, EventPattern.USB_DISCONNECT);

      var usbCounterAfter = getCounterValue(monitor, "usbEvents");
      assertEquals(usbCounterBefore + 1, usbCounterAfter);
    }

    @Test
    @DisplayName("USB_RESET should increment USB counter")
    void testIncrementUsbReset() throws Exception {
      var monitor = createMonitorInstance();
      var usbCounterBefore = getCounterValue(monitor, "usbEvents");

      incrementCounterMethod.invoke(monitor, EventPattern.USB_RESET);

      var usbCounterAfter = getCounterValue(monitor, "usbEvents");
      assertEquals(usbCounterBefore + 1, usbCounterAfter);
    }

    @Test
    @DisplayName("I2C_ERROR should increment I2C counter")
    void testIncrementI2cError() throws Exception {
      var monitor = createMonitorInstance();
      var i2cCounterBefore = getCounterValue(monitor, "i2cEvents");

      incrementCounterMethod.invoke(monitor, EventPattern.I2C_ERROR);

      var i2cCounterAfter = getCounterValue(monitor, "i2cEvents");
      assertEquals(i2cCounterBefore + 1, i2cCounterAfter);
    }

    @Test
    @DisplayName("CAN_ERROR should increment CAN counter")
    void testIncrementCanError() throws Exception {
      var monitor = createMonitorInstance();
      var canCounterBefore = getCounterValue(monitor, "canEvents");

      incrementCounterMethod.invoke(monitor, EventPattern.CAN_ERROR);

      var canCounterAfter = getCounterValue(monitor, "canEvents");
      assertEquals(canCounterBefore + 1, canCounterAfter);
    }

    @Test
    @DisplayName("POWER_EVENT should increment power counter")
    void testIncrementPowerEvent() throws Exception {
      var monitor = createMonitorInstance();
      var powerCounterBefore = getCounterValue(monitor, "powerEvents");

      incrementCounterMethod.invoke(monitor, EventPattern.POWER_EVENT);

      var powerCounterAfter = getCounterValue(monitor, "powerEvents");
      assertEquals(powerCounterBefore + 1, powerCounterAfter);
    }
  }

  @Nested
  @DisplayName("Queue Behavior Tests")
  class QueueBehaviorTests {

    @Test
    @DisplayName("Event queue should have correct maximum capacity")
    void testQueueCapacity() throws Exception {
      var monitor = createMonitorInstance();
      var queue = getEventQueue(monitor);

      // The remaining capacity should be MAX_QUEUE_SIZE (5000) since queue is empty
      assertEquals(5000, queue.remainingCapacity());
    }

    @Test
    @DisplayName("Should be able to queue events up to capacity")
    void testQueueUpToCapacity() throws Exception {
      var monitor = createMonitorInstance();
      var queue = getEventQueue(monitor);

      // Add a few events
      for (int i = 0; i < 10; i++) {
        var event = new KernelEvent("123.456", "test message " + i, "TEST_EVENT");
        assertTrue(queue.offer(event));
      }

      assertEquals(10, queue.size());
    }

    @Test
    @DisplayName("Queue should reject events when full")
    void testQueueRejectsWhenFull() throws Exception {
      var monitor = createMonitorInstance();
      var queue = getEventQueue(monitor);

      // Fill the queue to capacity
      for (int i = 0; i < 5000; i++) {
        var event = new KernelEvent("123.456", "test message " + i, "TEST_EVENT");
        queue.offer(event);
      }

      // Try to add one more - should fail
      var extraEvent = new KernelEvent("123.456", "extra message", "TEST_EVENT");
      assertFalse(queue.offer(extraEvent));
    }
  }

  @Nested
  @DisplayName("Statistics Tests")
  class StatisticsTests {

    @Test
    @DisplayName("All statistics counters should start at zero")
    void testCountersStartAtZero() throws Exception {
      var monitor = createMonitorInstance();

      assertEquals(0, getCounterValue(monitor, "totalEventsDetected"));
      assertEquals(0, getCounterValue(monitor, "usbEvents"));
      assertEquals(0, getCounterValue(monitor, "i2cEvents"));
      assertEquals(0, getCounterValue(monitor, "canEvents"));
      assertEquals(0, getCounterValue(monitor, "powerEvents"));
      assertEquals(0, getCounterValue(monitor, "otherEvents"));
    }

    @Test
    @DisplayName("Counters should be thread-safe AtomicIntegers")
    void testCountersAreAtomic() throws Exception {
      var monitor = createMonitorInstance();

      var totalField = KernelLogMonitor.class.getDeclaredField("totalEventsDetected");
      totalField.setAccessible(true);
      var counter = totalField.get(monitor);

      assertTrue(counter instanceof AtomicInteger);
    }
  }

  @Nested
  @DisplayName("Integration Tests")
  class IntegrationTests {

    @ParameterizedTest
    @CsvSource({
      "kern :err : [123.456] usb 1-1: USB disconnect, USB_DISCONNECT",
      "kern :warn : [789.012] i2c error on bus 0, I2C_ERROR",
      "user :err : [456.789] can0: error-passive, CAN_ERROR",
      "kern :warn : [111.222] voltage drop detected, POWER_EVENT",
      "kern :err : [333.444] usb reset high-speed, USB_RESET"
    })
    @DisplayName("Should correctly identify event types from real kernel log formats")
    void testEventTypeIdentification(String logLine, String expectedEventType) {
      var matchedPattern = EventPattern.findMatch(logLine);
      assertNotNull(matchedPattern);
      assertEquals(expectedEventType, matchedPattern.name());
    }
  }

  // Helper methods

  /**
   * Creates a monitor instance using reflection to bypass singleton restrictions. This creates a
   * new instance for testing without affecting the singleton.
   */
  private KernelLogMonitor createMonitorInstance() throws Exception {
    var constructor = KernelLogMonitor.class.getDeclaredConstructor();
    constructor.setAccessible(true);
    return constructor.newInstance();
  }

  /** Gets the value of an AtomicInteger counter field using reflection. */
  private int getCounterValue(KernelLogMonitor monitor, String fieldName) throws Exception {
    var field = KernelLogMonitor.class.getDeclaredField(fieldName);
    field.setAccessible(true);
    var atomicInt = (AtomicInteger) field.get(monitor);
    return atomicInt.get();
  }

  /** Gets the event queue using reflection. */
  @SuppressWarnings("unchecked")
  private BlockingQueue<KernelEvent> getEventQueue(KernelLogMonitor monitor) throws Exception {
    var field = KernelLogMonitor.class.getDeclaredField("eventQueue");
    field.setAccessible(true);
    return (BlockingQueue<KernelEvent>) field.get(monitor);
  }
}
