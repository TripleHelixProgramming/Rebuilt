package frc.robot.util;

import edu.wpi.first.wpilibj.RobotBase;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.regex.Pattern;
import org.littletonrobotics.junction.Logger;

/**
 * Monitors RoboRIO kernel logs for unexpected events (USB disconnects, ESD events, hardware errors)
 * and publishes them to NetworkTables via AdvantageKit Logger.
 *
 * <p>Uses a long-running dmesg process in follow mode with a blocking reader thread for efficiency.
 * This is a singleton that runs as a daemon thread for the lifetime of the robot program.
 */
public class KernelLogMonitor {
  private static final int MAX_QUEUE_SIZE = 5000;
  private static final int MAX_EVENTS_PER_CYCLE = 10;
  private static final double QUEUE_WARNING_THRESHOLD = 0.8;

  private static KernelLogMonitor instance;

  /** Event patterns to detect specific hardware issues. */
  public enum EventPattern {
    USB_DISCONNECT(Pattern.compile("usb.*disconnect", Pattern.CASE_INSENSITIVE)),
    USB_RESET(Pattern.compile("usb.*reset", Pattern.CASE_INSENSITIVE)),
    I2C_ERROR(Pattern.compile("i2c.*error", Pattern.CASE_INSENSITIVE)),
    CAN_ERROR(Pattern.compile("can.*error", Pattern.CASE_INSENSITIVE)),
    POWER_EVENT(Pattern.compile("(voltage|power|brownout)", Pattern.CASE_INSENSITIVE));

    private final Pattern pattern;

    EventPattern(Pattern pattern) {
      this.pattern = pattern;
    }

    /**
     * Tests if this pattern matches the given line.
     *
     * @param line the log line to test
     * @return true if the pattern matches
     */
    public boolean matches(String line) {
      return pattern.matcher(line).find();
    }

    /**
     * Finds the first matching event pattern for the given line.
     *
     * @param line the log line to test
     * @return the matching EventPattern, or null if no match
     */
    public static EventPattern findMatch(String line) {
      return Arrays.stream(values())
          .filter(pattern -> pattern.matches(line))
          .findFirst()
          .orElse(null);
    }
  }

  /** Kernel event record containing timestamp, message, and event type. */
  public record KernelEvent(String timestamp, String message, String eventType) {}

  private final BlockingQueue<KernelEvent> eventQueue = new LinkedBlockingQueue<>(MAX_QUEUE_SIZE);
  private Process dmesgProcess;
  private ExecutorService executor;
  private volatile boolean running = false;

  // Statistics - using AtomicInteger for thread-safe access
  private final AtomicInteger totalEventsDetected = new AtomicInteger(0);
  private final AtomicInteger usbEvents = new AtomicInteger(0);
  private final AtomicInteger i2cEvents = new AtomicInteger(0);
  private final AtomicInteger canEvents = new AtomicInteger(0);
  private final AtomicInteger powerEvents = new AtomicInteger(0);
  private final AtomicInteger otherEvents = new AtomicInteger(0);

  /** Private constructor for singleton pattern. Use getInstance() instead. */
  private KernelLogMonitor() {}

  /**
   * Gets the singleton instance of KernelLogMonitor. Automatically starts monitoring on first call
   * if running on real robot.
   *
   * @return the singleton instance
   */
  public static synchronized KernelLogMonitor getInstance() {
    if (instance == null) {
      instance = new KernelLogMonitor();
      instance.start();
    }
    return instance;
  }

  /**
   * Starts monitoring kernel logs. Should only be called in REAL mode. Called automatically by
   * getInstance().
   */
  private void start() {
    if (!RobotBase.isReal()) {
      System.out.println(
          "[KernelLogMonitor] Not starting - not running on real robot (simulation or replay"
              + " mode)");
      return;
    }

    if (running) {
      System.out.println("[KernelLogMonitor] Already running");
      return;
    }

    try {
      // Start dmesg in follow mode with boot-time timestamps
      // -w: wait for new messages (follow mode)
      // -x: decode facility and level (to distinguish error vs warning)
      // --level=err,warn: only errors and warnings (reduces noise)
      // Note: Using boot time (seconds since boot) instead of -T (wall-clock time)
      //       because RoboRIO often has incorrect wall-clock time until DS connects
      var pb = new ProcessBuilder("dmesg", "-w", "-x", "--level=err,warn");
      pb.redirectErrorStream(true); // Merge stderr into stdout
      dmesgProcess = pb.start();

      // Create daemon thread factory for executor
      ThreadFactory daemonThreadFactory =
          runnable -> {
            var thread = new Thread(runnable, "KernelLogMonitor");
            thread.setDaemon(true); // Allow JVM to exit even if thread is running
            return thread;
          };

      // Create single-threaded executor with daemon threads
      executor = Executors.newSingleThreadExecutor(daemonThreadFactory);
      running = true; // Set before submitting to avoid race condition

      // Submit the log monitoring task
      executor.submit(
          () -> {
            try (var reader =
                new BufferedReader(new InputStreamReader(dmesgProcess.getInputStream()))) {
              System.out.println("[KernelLogMonitor] Started monitoring kernel logs");

              // Blocking read - thread sleeps until new line arrives
              String line;
              while (running && (line = reader.readLine()) != null) {
                processLogLine(line);
              }
            } catch (IOException e) {
              if (running) {
                System.err.println(
                    "[KernelLogMonitor] Error reading kernel logs: " + e.getMessage());
              }
            }
            System.out.println("[KernelLogMonitor] Reader thread stopped");
          });

    } catch (IOException e) {
      System.err.println("[KernelLogMonitor] Failed to start dmesg process: " + e.getMessage());
    }
  }

  /**
   * No-op method. Monitor is designed to run as a daemon thread for the lifetime of the JVM.
   * Resources are automatically cleaned up when the JVM exits.
   *
   * @deprecated Monitor runs for entire program lifetime. No need to stop.
   */
  @Deprecated
  public void stop() {
    // Intentionally empty - daemon thread will be cleaned up when JVM exits
    System.out.println(
        "[KernelLogMonitor] stop() called but monitor runs for program lifetime (daemon thread)");
  }

  /**
   * Processes a single log line, checking for patterns of interest.
   *
   * @param line Raw log line from dmesg (with -x flag, includes facility and level)
   */
  private void processLogLine(String line) {
    var matchedPattern = EventPattern.findMatch(line);

    String eventType;
    if (matchedPattern != null) {
      eventType = matchedPattern.name();
      incrementCounterForPattern(matchedPattern);
    } else {
      // No specific pattern matched - determine type based on severity level
      eventType = categorizeGenericEvent(line);
      otherEvents.incrementAndGet();
    }

    // Queue the event
    var timestamp = extractTimestamp(line);
    var event = new KernelEvent(timestamp, line.trim(), eventType);

    // Use offer which returns false if queue is full (bounded queue)
    if (!eventQueue.offer(event)) {
      System.err.println(
          "[KernelLogMonitor] Event queue full (max " + MAX_QUEUE_SIZE + "), dropping event");
      return; // Don't increment counter if we couldn't queue
    }

    totalEventsDetected.incrementAndGet();

    // Also print to console for immediate visibility
    var truncatedLine = line.substring(0, Math.min(100, line.length()));
    System.out.println("[KernelLogMonitor] Detected " + eventType + ": " + truncatedLine);
  }

  /**
   * Increments the appropriate counter for a matched event pattern.
   *
   * @param pattern the matched event pattern
   */
  private void incrementCounterForPattern(EventPattern pattern) {
    switch (pattern) {
      case USB_DISCONNECT, USB_RESET -> usbEvents.incrementAndGet();
      case I2C_ERROR -> i2cEvents.incrementAndGet();
      case CAN_ERROR -> canEvents.incrementAndGet();
      case POWER_EVENT -> powerEvents.incrementAndGet();
    }
  }

  /**
   * Categorizes a generic kernel event based on severity level in the log line.
   *
   * @param line the log line from dmesg
   * @return the event type string
   */
  private String categorizeGenericEvent(String line) {
    // With -x flag, line format is like: "facility :level : [timestamp] message"
    if (line.contains(":err  :") || line.contains(":error:")) {
      return "KERNEL_ERROR";
    } else if (line.contains(":warn :") || line.contains(":warning:")) {
      return "KERNEL_WARNING";
    } else {
      // Fallback - couldn't parse level, but we know it's err or warn from filter
      return "KERNEL_EVENT";
    }
  }

  /**
   * Extracts timestamp from dmesg line (format with -x: "facility,level,[seconds] message")
   *
   * @param line The log line
   * @return The extracted timestamp (seconds since boot) or "unknown" if not found
   */
  private String extractTimestamp(String line) {
    // With -x flag, format is like: "kern  :warn  : [12345.678901] message"
    // Find the timestamp in brackets (seconds since boot)
    var startBracket = line.indexOf('[');
    if (startBracket >= 0) {
      var endBracket = line.indexOf(']', startBracket);
      if (endBracket > 0) {
        return line.substring(startBracket + 1, endBracket);
      }
    }
    return "unknown";
  }

  /**
   * Publishes queued events to AdvantageKit Logger (which auto-publishes to NetworkTables). Should
   * be called from the main robot thread's periodic method.
   */
  public void publishToLogger() {
    var queueSize = eventQueue.size();

    if (queueSize == 0) {
      // Still publish statistics even if no events
      publishStatistics(queueSize);
      return;
    }

    // Drain the queue and publish all events
    var recentEvents = new StringBuilder();
    var eventsPublished = 0;

    KernelEvent event;
    while ((event = eventQueue.poll()) != null && eventsPublished < MAX_EVENTS_PER_CYCLE) {
      recentEvents
          .append('[')
          .append(event.timestamp())
          .append("] ")
          .append(event.eventType())
          .append(": ")
          .append(event.message())
          .append('\n');
      eventsPublished++;
    }

    if (eventsPublished > 0) {
      // Publish to NetworkTables via Logger (automatically handled by NT4Publisher)
      Logger.recordOutput("Kernel/RecentEvents", recentEvents.toString());
    }

    // Update queue size after draining some events
    queueSize = eventQueue.size();
    publishStatistics(queueSize);

    // Warn if queue is getting full (> 80% capacity)
    if (queueSize > MAX_QUEUE_SIZE * QUEUE_WARNING_THRESHOLD) {
      Logger.recordOutput("Kernel/QueueNearFull", true);
      System.err.println(
          "[KernelLogMonitor] WARNING: Event queue near capacity ("
              + queueSize
              + "/"
              + MAX_QUEUE_SIZE
              + ")");
    } else {
      Logger.recordOutput("Kernel/QueueNearFull", false);
    }
  }

  /**
   * Publishes statistics to Logger.
   *
   * @param currentQueueSize The current size of the event queue
   */
  private void publishStatistics(int currentQueueSize) {
    Logger.recordOutput("Kernel/EventsPendingPublish", currentQueueSize);
    Logger.recordOutput("Kernel/Stats/TotalEvents", totalEventsDetected.get());
    Logger.recordOutput("Kernel/Stats/USBEvents", usbEvents.get());
    Logger.recordOutput("Kernel/Stats/I2CEvents", i2cEvents.get());
    Logger.recordOutput("Kernel/Stats/CANEvents", canEvents.get());
    Logger.recordOutput("Kernel/Stats/PowerEvents", powerEvents.get());
    Logger.recordOutput("Kernel/Stats/OtherEvents", otherEvents.get());
    Logger.recordOutput("Kernel/Stats/MonitorRunning", running);
  }

  /** Returns true if the monitor is currently running. */
  public boolean isRunning() {
    return running;
  }
}
