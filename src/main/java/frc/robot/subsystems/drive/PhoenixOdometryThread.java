// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
  private final Lock signalsLock =
      new ReentrantLock(); // Prevents conflicts when registering signals
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();
  private double[] signalAges = new double[0];
  private boolean[] signalTimestampValid = new boolean[0];
  private boolean[] signalFresh = new boolean[0];
  private double[] signalValues = new double[0];
  private double lastTimestamp = Double.NEGATIVE_INFINITY;
  private volatile long droppedSamples = 0; // Written only by this thread

  private static boolean isCANFD = new CANBus(DRIVETRAIN_CONSTANTS.CANBusName).isNetworkFD();
  private static PhoenixOdometryThread instance = null;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    if (timestampQueues.size() > 0) {
      super.start();
    }
  }

  /**
   * Registers a Phoenix signal to be read from the thread. The queue receives NaN for samples taken
   * while the signal was stale.
   */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(ODOMETRY_QUEUE_CAPACITY);
    signalsLock.lock();
    Drive.ODOMETRY_LOCK.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
      System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
      newSignals[phoenixSignals.length] = signal;
      phoenixSignals = newSignals;
      phoenixQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Drive.ODOMETRY_LOCK.unlock();
    }
    return queue;
  }

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(ODOMETRY_QUEUE_CAPACITY);
    signalsLock.lock();
    Drive.ODOMETRY_LOCK.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Drive.ODOMETRY_LOCK.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(ODOMETRY_QUEUE_CAPACITY);
    Drive.ODOMETRY_LOCK.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.ODOMETRY_LOCK.unlock();
    }
    return queue;
  }

  /** Returns how many samples were lost because a queue was full. */
  public long getDroppedSamples() {
    return droppedSamples;
  }

  /**
   * Marks which signals are fresh. A signal is stale if its timestamp is invalid or in the future,
   * if it lags the freshest signal by {@link DriveConstants#ODOMETRY_SIGNAL_MAX_AGE_SECONDS} or
   * more (its device stopped updating), or if even the freshest signal is {@link
   * DriveConstants#ODOMETRY_BUS_MAX_AGE_SECONDS} old (the bus stopped). Comparing with the freshest
   * signal keeps a delay in this thread, which ages every signal equally, from making healthy
   * signals stale.
   *
   * @param ages age of each signal's last update, in seconds
   * @param timestampValid whether each signal's timestamp is valid
   * @param fresh receives whether each signal is fresh
   */
  static void markFresh(double[] ages, boolean[] timestampValid, boolean[] fresh) {
    double freshestAge = Double.POSITIVE_INFINITY;
    for (int i = 0; i < ages.length; i++) {
      if (plausibleAge(ages[i], timestampValid[i])) {
        freshestAge = Math.min(freshestAge, ages[i]);
      }
    }
    for (int i = 0; i < ages.length; i++) {
      fresh[i] =
          plausibleAge(ages[i], timestampValid[i])
              && ages[i] - freshestAge < ODOMETRY_SIGNAL_MAX_AGE_SECONDS
              && freshestAge < ODOMETRY_BUS_MAX_AGE_SECONDS;
    }
  }

  /** A timestamp in the future is garbage; VACHE session 19 had ones 2.7e7 s ahead. */
  private static boolean plausibleAge(double age, boolean timestampValid) {
    return timestampValid && age > -ODOMETRY_SIGNAL_MAX_AGE_SECONDS;
  }

  /**
   * Returns the timestamp for a sample: the FPGA time minus the mean age of the fresh signals, or
   * the FPGA time if none is fresh. It is never after the FPGA time or more than {@link
   * DriveConstants#ODOMETRY_BUS_MAX_AGE_SECONDS} before it, and it is kept strictly after the
   * previous timestamp.
   *
   * @param fpgaSeconds FPGA time when the signals were read
   * @param signalAges age of each Phoenix signal's last update, in seconds
   * @param fresh whether each signal was fresh
   * @param previousTimestamp the previous sample's timestamp
   */
  static double sampleTimestamp(
      double fpgaSeconds, double[] signalAges, boolean[] fresh, double previousTimestamp) {
    double totalAge = 0.0;
    int freshCount = 0;
    for (int i = 0; i < signalAges.length; i++) {
      if (fresh[i]) {
        totalAge += signalAges[i];
        freshCount++;
      }
    }
    double timestamp = freshCount > 0 ? fpgaSeconds - totalAge / freshCount : fpgaSeconds;
    timestamp = MathUtil.clamp(timestamp, fpgaSeconds - ODOMETRY_BUS_MAX_AGE_SECONDS, fpgaSeconds);
    return Math.max(timestamp, previousTimestamp + 1e-6);
  }

  @Override
  public void run() {
    while (true) {
      BaseStatusSignal[] signals;
      double timestamp;
      signalsLock.lock();
      try {
        // Wait for updates from all signals
        try {
          if (isCANFD && phoenixSignals.length > 0) {
            BaseStatusSignal.waitForAll(2.0 / Drive.ODOMETRY_FREQUENCY, phoenixSignals);
          } else {
            // "waitForAll" does not support blocking on multiple signals with a bus
            // that is not CAN FD, regardless of Pro licensing. No reasoning for this
            // behavior is provided by the documentation.
            Thread.sleep((long) (1000.0 / Drive.ODOMETRY_FREQUENCY));
            if (phoenixSignals.length > 0) BaseStatusSignal.refreshAll(phoenixSignals);
          }
        } catch (InterruptedException e) {
          e.printStackTrace();
        }

        // Read the signals right away, so time spent waiting for ODOMETRY_LOCK doesn't age them.
        // A stale signal's value is replaced with NaN and its age is left out of the timestamp:
        // a dead device's age grows without bound (when the CANivore left USB at VACHE, ages
        // reached minutes). Sample timestamp is current FPGA time minus the average age of the
        // fresh signals; default timestamps from Phoenix are NOT compatible with FPGA timestamps.
        signals = phoenixSignals;
        double fpgaSeconds = RobotController.getFPGATime() / 1e6;
        if (signalAges.length != signals.length) {
          signalAges = new double[signals.length];
          signalTimestampValid = new boolean[signals.length];
          signalFresh = new boolean[signals.length];
          signalValues = new double[signals.length];
        }
        for (int i = 0; i < signals.length; i++) {
          var signalTimestamp = signals[i].getTimestamp();
          signalAges[i] = signalTimestamp.getLatency();
          signalTimestampValid[i] = signalTimestamp.isValid();
        }
        markFresh(signalAges, signalTimestampValid, signalFresh);
        for (int i = 0; i < signals.length; i++) {
          signalValues[i] = signalFresh[i] ? signals[i].getValueAsDouble() : Double.NaN;
        }
        timestamp = sampleTimestamp(fpgaSeconds, signalAges, signalFresh, lastTimestamp);
        lastTimestamp = timestamp;
      } finally {
        signalsLock.unlock();
      }

      // Save new data to queues
      Drive.ODOMETRY_LOCK.lock();
      try {
        // Add new samples to queues; a full queue loses the sample, which is counted
        boolean dropped = false;
        for (int i = 0; i < signals.length; i++) {
          dropped |= !phoenixQueues.get(i).offer(signalValues[i]);
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          dropped |= !genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          dropped |= !timestampQueues.get(i).offer(timestamp);
        }
        if (dropped) {
          droppedSamples++;
        }
      } finally {
        Drive.ODOMETRY_LOCK.unlock();
      }
    }
  }
}
