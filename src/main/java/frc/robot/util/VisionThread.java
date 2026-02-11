package frc.robot.util;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Provides an interface for asynchronously reading vision inputs on a background thread, avoiding
 * blocking network calls on the main robot loop.
 */
public class VisionThread {
  private static VisionThread instance;
  private static final double UPDATE_FREQUENCY_HZ = 50.0; // 20ms update rate

  private final ReentrantLock lock = new ReentrantLock();
  private final Notifier notifier;
  private final List<VisionInputs> registeredInputs = new ArrayList<>();

  /** Cached inputs for a single VisionIO device. */
  public static class VisionInputs {
    private final VisionIO io;
    private final VisionIOInputs workingInputs = new VisionIOInputs();

    // Cached snapshot (volatile reference for thread safety)
    private volatile VisionIOInputsSnapshot snapshot = new VisionIOInputsSnapshot();

    public VisionInputs(VisionIO io) {
      this.io = io;
    }

    /** Updates cached values. Called from background thread. */
    private void update() {
      // Call the potentially-blocking updateInputs
      io.updateInputs(workingInputs);

      // Create an immutable snapshot of the current state
      snapshot =
          new VisionIOInputsSnapshot(
              workingInputs.connected,
              workingInputs.latestTargetObservation,
              workingInputs.poseObservations.clone(),
              workingInputs.tagIds.clone());
    }

    /** Returns the cached snapshot. Safe to call from main thread. */
    public VisionIOInputsSnapshot getSnapshot() {
      return snapshot;
    }
  }

  /** Immutable snapshot of VisionIOInputs for thread-safe reading. */
  public static class VisionIOInputsSnapshot {
    public final boolean connected;
    public final TargetObservation latestTargetObservation;
    public final PoseObservation[] poseObservations;
    public final int[] tagIds;

    public VisionIOInputsSnapshot() {
      this.connected = false;
      this.latestTargetObservation =
          new TargetObservation(
              edu.wpi.first.math.geometry.Rotation2d.kZero,
              edu.wpi.first.math.geometry.Rotation2d.kZero,
              edu.wpi.first.math.geometry.Rotation2d.kZero,
              0,
              -1,
              -1);
      this.poseObservations = new PoseObservation[0];
      this.tagIds = new int[0];
    }

    public VisionIOInputsSnapshot(
        boolean connected,
        TargetObservation latestTargetObservation,
        PoseObservation[] poseObservations,
        int[] tagIds) {
      this.connected = connected;
      this.latestTargetObservation = latestTargetObservation;
      this.poseObservations = poseObservations;
      this.tagIds = tagIds;
    }

    /** Copies this snapshot's values into a VisionIOInputs object for AdvantageKit logging. */
    public void copyTo(VisionIOInputs inputs) {
      inputs.connected = connected;
      inputs.latestTargetObservation = latestTargetObservation;
      inputs.poseObservations = poseObservations;
      inputs.tagIds = tagIds;
    }
  }

  /** Returns the singleton instance, creating it if necessary. */
  public static synchronized VisionThread getInstance() {
    if (instance == null) {
      instance = new VisionThread();
    }
    return instance;
  }

  private VisionThread() {
    notifier = new Notifier(this::periodic);
    notifier.setName("VisionThread");
  }

  /** Starts the background thread. Call after all devices are registered. */
  public void start() {
    if (!registeredInputs.isEmpty()) {
      notifier.startPeriodic(1.0 / UPDATE_FREQUENCY_HZ);
    }
  }

  /**
   * Registers a VisionIO device for background reading.
   *
   * @param io The VisionIO device
   * @return VisionInputs object to read cached values from
   */
  public VisionInputs registerVisionIO(VisionIO io) {
    VisionInputs inputs = new VisionInputs(io);
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
      for (VisionInputs inputs : registeredInputs) {
        inputs.update();
      }
    } finally {
      lock.unlock();
    }
  }
}
