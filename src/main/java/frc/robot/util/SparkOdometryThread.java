package frc.robot.util;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading SparkMax/SparkFlex inputs on a background
 * thread, avoiding blocking CAN calls on the main robot loop.
 */
public class SparkOdometryThread {
  private static SparkOdometryThread instance;
  private static final double UPDATE_FREQUENCY_HZ = 100.0; // 10ms update rate

  private final ReentrantLock lock = new ReentrantLock();
  private final Notifier notifier;
  private final List<SparkInputs> registeredInputs = new ArrayList<>();

  /** Cached inputs for a single SparkMax/SparkFlex device. */
  public static class SparkInputs {
    private final SparkBase spark;
    private final DoubleSupplier positionSupplier;
    private final DoubleSupplier velocitySupplier;
    private final DoubleSupplier[] additionalSuppliers;

    // Cached values (volatile for thread safety)
    private volatile double position = 0.0;
    private volatile double velocity = 0.0;
    private volatile double appliedVolts = 0.0;
    private volatile double outputCurrent = 0.0;
    private volatile double[] additionalValues;
    private volatile double timestamp = 0.0;
    private volatile boolean connected = true;

    /**
     * Creates SparkInputs for a device.
     *
     * @param spark The SparkBase device
     * @param positionSupplier Supplier for encoder position
     * @param velocitySupplier Supplier for encoder velocity
     * @param additionalSuppliers Optional additional values to read
     */
    public SparkInputs(
        SparkBase spark,
        DoubleSupplier positionSupplier,
        DoubleSupplier velocitySupplier,
        DoubleSupplier... additionalSuppliers) {
      this.spark = spark;
      this.positionSupplier = positionSupplier;
      this.velocitySupplier = velocitySupplier;
      this.additionalSuppliers = additionalSuppliers;
      this.additionalValues = new double[additionalSuppliers.length];
    }

    /** Updates all cached values. Called from background thread. */
    private void update() {
      boolean ok = true;

      double pos = positionSupplier.getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) ok = false;

      double vel = velocitySupplier.getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) ok = false;

      double output = spark.getAppliedOutput();
      if (spark.getLastError() != REVLibError.kOk) ok = false;

      double voltage = spark.getBusVoltage();
      if (spark.getLastError() != REVLibError.kOk) ok = false;

      double volts = output * voltage;

      double current = spark.getOutputCurrent();
      if (spark.getLastError() != REVLibError.kOk) ok = false;

      double[] additional = new double[additionalSuppliers.length];
      for (int i = 0; i < additionalSuppliers.length; i++) {
        additional[i] = additionalSuppliers[i].getAsDouble();
        if (spark.getLastError() != REVLibError.kOk) ok = false;
      }

      // Only update cached values if all reads succeeded
      if (ok) {
        position = pos;
        velocity = vel;
        appliedVolts = volts;
        outputCurrent = current;
        additionalValues = additional;
        timestamp = RobotController.getFPGATime() / 1e6;
      }
      connected = ok;
    }

    // Getters for cached values (called from main thread)
    public double getPosition() {
      return position;
    }

    public double getVelocity() {
      return velocity;
    }

    public double getAppliedVolts() {
      return appliedVolts;
    }

    public double getOutputCurrent() {
      return outputCurrent;
    }

    public double getAdditionalValue(int index) {
      return additionalValues[index];
    }

    public double getTimestamp() {
      return timestamp;
    }

    public boolean isConnected() {
      return connected;
    }
  }

  /** Returns the singleton instance, creating it if necessary. */
  public static synchronized SparkOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkOdometryThread();
    }
    return instance;
  }

  private SparkOdometryThread() {
    notifier = new Notifier(this::periodic);
    notifier.setName("SparkOdometryThread");
  }

  /** Starts the background thread. Call after all devices are registered. */
  public void start() {
    if (!registeredInputs.isEmpty()) {
      notifier.startPeriodic(1.0 / UPDATE_FREQUENCY_HZ);
    }
  }

  /**
   * Registers a SparkMax/SparkFlex device for background reading.
   *
   * @param spark The SparkBase device
   * @param encoder The relative encoder
   * @param additionalSuppliers Optional additional suppliers (e.g., absolute encoder position)
   * @return SparkInputs object to read cached values from
   */
  public SparkInputs registerSpark(
      SparkBase spark, RelativeEncoder encoder, DoubleSupplier... additionalSuppliers) {
    SparkInputs inputs =
        new SparkInputs(spark, encoder::getPosition, encoder::getVelocity, additionalSuppliers);
    lock.lock();
    try {
      registeredInputs.add(inputs);
    } finally {
      lock.unlock();
    }
    return inputs;
  }

  /**
   * Registers a SparkMax/SparkFlex device with an absolute encoder for background reading.
   *
   * @param spark The SparkBase device
   * @param encoder The absolute encoder
   * @param additionalSuppliers Optional additional suppliers
   * @return SparkInputs object to read cached values from
   */
  public SparkInputs registerSpark(
      SparkBase spark, AbsoluteEncoder encoder, DoubleSupplier... additionalSuppliers) {
    SparkInputs inputs =
        new SparkInputs(spark, encoder::getPosition, encoder::getVelocity, additionalSuppliers);
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
      for (SparkInputs inputs : registeredInputs) {
        inputs.update();
      }
    } finally {
      lock.unlock();
    }
  }
}
