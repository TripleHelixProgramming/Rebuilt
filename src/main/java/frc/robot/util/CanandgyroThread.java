package frc.robot.util;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Provides an interface for asynchronously reading Canandgyro inputs on a background thread,
 * avoiding blocking CAN calls on the main robot loop.
 */
public class CanandgyroThread {
  private static CanandgyroThread instance;
  private static final double UPDATE_FREQUENCY_HZ = 100.0; // 10ms update rate

  private final ReentrantLock lock = new ReentrantLock();
  private final Notifier notifier;
  private final List<GyroInputs> registeredInputs = new ArrayList<>();

  /** Cached inputs for a single Canandgyro device. */
  public static class GyroInputs {
    private final Canandgyro gyro;

    // Cached values (volatile for thread safety)
    private volatile boolean connected = false;
    private volatile boolean calibrating = true;
    private volatile double yaw = 0.0;
    private volatile double angularVelocityYaw = 0.0;
    private volatile double timestamp = 0.0;

    public GyroInputs(Canandgyro gyro) {
      this.gyro = gyro;
    }

    /** Updates all cached values. Called from background thread. */
    private void update() {
      connected = gyro.isConnected();
      calibrating = gyro.isCalibrating();
      yaw = gyro.getYaw();
      angularVelocityYaw = gyro.getAngularVelocityYaw();
      timestamp = RobotController.getFPGATime() / 1e6;
    }

    // Getters for cached values (called from main thread)
    public boolean isConnected() {
      return connected;
    }

    public boolean isCalibrating() {
      return calibrating;
    }

    public double getYaw() {
      return yaw;
    }

    public double getAngularVelocityYaw() {
      return angularVelocityYaw;
    }

    public double getTimestamp() {
      return timestamp;
    }
  }

  /** Returns the singleton instance, creating it if necessary. */
  public static synchronized CanandgyroThread getInstance() {
    if (instance == null) {
      instance = new CanandgyroThread();
    }
    return instance;
  }

  private CanandgyroThread() {
    notifier = new Notifier(this::periodic);
    notifier.setName("CanandgyroThread");
  }

  /** Starts the background thread. */
  public void start() {
    notifier.startPeriodic(1.0 / UPDATE_FREQUENCY_HZ);
  }

  /**
   * Registers a Canandgyro device for background reading.
   *
   * @param gyro The Canandgyro device
   * @return GyroInputs object to read cached values from
   */
  public GyroInputs registerCanandgyro(Canandgyro gyro) {
    GyroInputs inputs = new GyroInputs(gyro);
    lock.lock();
    try {
      registeredInputs.add(inputs);
    } finally {
      lock.unlock();
    }
    return inputs;
  }

  /** Called periodically by the Notifier to update all registered inputs. */
  private void periodic() {
    lock.lock();
    try {
      for (GyroInputs inputs : registeredInputs) {
        inputs.update();
      }
    } finally {
      lock.unlock();
    }
  }
}
