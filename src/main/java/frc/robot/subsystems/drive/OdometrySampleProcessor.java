package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.HashMap;
import java.util.Map;

/**
 * Prepares raw odometry samples for the pose estimator, so a misbehaving drive module or a bad
 * timestamp degrades odometry instead of corrupting the pose.
 *
 * <p>Match logs show three failures, each handled here:
 *
 * <ul>
 *   <li>A module whose signals are stale (its devices dropped off the CAN bus) is given the motion
 *       that the other modules and the gyro imply. Treating it as a wheel that stopped would bias
 *       the translation and, without a gyro, add false rotation.
 *   <li>A wheel position that jumps further than a wheel can travel (a device reset to zero) is
 *       absorbed into a per-module offset. A module that starts reporting again keeps the travel it
 *       counted while out, if that travel was possible in the time it was out.
 *   <li>A timestamp that is not finite, more than {@link
 *       DriveConstants#ODOMETRY_MAX_SAMPLE_AGE_SECONDS} old, or in the future is replaced with the
 *       loop time. Timestamps passed on are strictly increasing, so the estimator's history stays
 *       in order and vision corrections keep applying.
 * </ul>
 *
 * <p>When every module is valid and the timestamps are sane, samples pass through unchanged.
 *
 * <p>Synthesizing a module's motion relies on the module layout being symmetric about the robot
 * center: the estimator's own least-squares fit over all modules then returns exactly the
 * translation that the valid modules imply.
 */
final class OdometrySampleProcessor {
  /** A processed sample, ready for SwerveDrivePoseEstimator.updateWithTime. */
  record Sample(
      double timestamp, Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {}

  private static final double MIN_TIMESTAMP_STEP_SECONDS = 1e-6;

  private final Translation2d[] moduleTranslations;
  private final SwerveDriveKinematics kinematics;
  private final Map<Integer, SwerveDriveKinematics> subsetKinematics = new HashMap<>();

  private final double[] offsetsMeters;
  private final SwerveModulePosition[] lastPositions; // What the estimator last consumed
  private final boolean[] lastValid;
  private double lastTimestamp = Double.NEGATIVE_INFINITY;
  private double lastRawTimestamp = Double.NaN;
  private final double[] lastValidRawMeters; // Each module's raw position when last valid
  private final double[] lastValidRawTimestamps;
  private Rotation2d gyroRotation = Rotation2d.kZero;
  private boolean hadGyro = false;

  private int validModuleCount;
  private long rebaseCount = 0;
  private long substitutedTimestampCount = 0;

  OdometrySampleProcessor(Translation2d[] moduleTranslations) {
    this.moduleTranslations = moduleTranslations;
    kinematics = new SwerveDriveKinematics(moduleTranslations);
    int count = moduleTranslations.length;
    offsetsMeters = new double[count];
    lastPositions = new SwerveModulePosition[count];
    lastValid = new boolean[count];
    lastValidRawMeters = new double[count];
    lastValidRawTimestamps = new double[count];
    for (int i = 0; i < count; i++) {
      lastPositions[i] = new SwerveModulePosition();
      lastValid[i] = true;
      lastValidRawTimestamps[i] = Double.NaN;
    }
    validModuleCount = count;
  }

  /**
   * Processes one odometry sample.
   *
   * @param timestamp the sample's timestamp from the odometry thread, in seconds
   * @param now the current loop timestamp, in seconds
   * @param rawPositions each module's measured position
   * @param valid whether each module's sample can be trusted
   * @param gyroYaw the gyro yaw for this sample, or null if the gyro is disconnected
   * @return the timestamp, rotation, and module positions to give the estimator
   */
  Sample process(
      double timestamp,
      double now,
      SwerveModulePosition[] rawPositions,
      boolean[] valid,
      Rotation2d gyroYaw) {
    int count = moduleTranslations.length;

    // Replace a timestamp that can't be right, then keep the sequence strictly increasing
    double stamp = timestamp;
    if (!(stamp >= now - ODOMETRY_MAX_SAMPLE_AGE_SECONDS
        && stamp <= now + ODOMETRY_MAX_SAMPLE_LEAD_SECONDS)) {
      stamp = now;
      substitutedTimestampCount++;
    }
    stamp = Math.max(stamp, lastTimestamp + MIN_TIMESTAMP_STEP_SECONDS);
    lastTimestamp = stamp;

    // Bound wheel steps by the spacing of the raw timestamps. Timestamps that lag (one stale
    // signal drags them back) are replaced above, and replacements in one loop are microseconds
    // apart, but the raw ones keep the true spacing. A raw timestamp that isn't finite or is in
    // the future says nothing about elapsed time, so its sample gets the tightest bound.
    boolean rawUsable =
        Double.isFinite(timestamp) && timestamp <= now + ODOMETRY_MAX_SAMPLE_LEAD_SECONDS;
    double maxStepMeters =
        maxWheelStep(rawUsable ? timestamp - lastRawTimestamp : 0.0); // NaN on the first sample
    if (rawUsable) {
      lastRawTimestamp = timestamp;
    }

    // Valid modules: apply offsets, absorbing resets
    var positions = new SwerveModulePosition[count];
    var usable = new boolean[count];
    var steps = new double[count];
    int validCount = 0;
    for (int i = 0; i < count; i++) {
      var raw = rawPositions[i];
      usable[i] = valid[i] && Double.isFinite(raw.distanceMeters);
      if (!usable[i]) {
        continue;
      }
      validCount++;
      double last = lastPositions[i].distanceMeters;
      double distance = raw.distanceMeters + offsetsMeters[i];
      boolean possible;
      if (lastValid[i]) {
        possible = Math.abs(distance - last) <= maxStepMeters;
      } else {
        // Back from an outage: its raw travel since it was last valid must have been possible in
        // the time since. The estimator then takes the difference between that travel and what
        // was estimated for the module while it was out.
        double outageSeconds =
            rawUsable ? timestamp - lastValidRawTimestamps[i] : 0.0; // NaN if never valid
        possible =
            Math.abs(raw.distanceMeters - lastValidRawMeters[i]) <= maxWheelStep(outageSeconds);
      }
      if (!possible) { // A reset, not motion: move the offset so the wheel doesn't jump
        offsetsMeters[i] += last - distance;
        distance = last;
        rebaseCount++;
      }
      positions[i] = new SwerveModulePosition(distance, raw.angle);
      steps[i] = distance - last;
      lastValidRawMeters[i] = raw.distanceMeters;
      if (rawUsable) {
        lastValidRawTimestamps[i] = timestamp;
      }
    }
    validModuleCount = validCount;

    // Rotation from the gyro if connected, else from the valid wheels; translation from the valid
    // wheels, needed only to fill in modules that aren't valid
    double dTheta = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    if (gyroYaw != null) {
      // On the first sample, or the first after the gyro reconnects, its yaw jumps from the
      // wheel-integrated heading; that is not rotation for the estimated modules
      dTheta = hadGyro ? gyroYaw.minus(gyroRotation).getRadians() : 0.0;
      gyroRotation = gyroYaw;
      if (validCount < count && validCount > 0) {
        for (int i = 0; i < count; i++) {
          if (usable[i]) {
            dx += steps[i] * positions[i].angle.getCos() + dTheta * moduleTranslations[i].getY();
            dy += steps[i] * positions[i].angle.getSin() - dTheta * moduleTranslations[i].getX();
          }
        }
        dx /= validCount;
        dy /= validCount;
      }
    } else if (validCount >= 2) {
      // The fit the estimator itself would make, over the valid modules only
      Twist2d twist = kinematicsFor(usable).toTwist2d(validDeltas(usable, steps, positions));
      dTheta = twist.dtheta;
      dx = twist.dx;
      dy = twist.dy;
      gyroRotation = gyroRotation.plus(new Rotation2d(dTheta));
    } else if (validCount == 1) {
      // One wheel can't give rotation; hold the heading and take its translation
      for (int i = 0; i < count; i++) {
        if (usable[i]) {
          dx = steps[i] * positions[i].angle.getCos();
          dy = steps[i] * positions[i].angle.getSin();
        }
      }
    }
    hadGyro = gyroYaw != null;

    // Invalid modules move as a rigid body with the valid ones
    for (int i = 0; i < count; i++) {
      if (usable[i]) {
        continue;
      }
      double moveX = dx - dTheta * moduleTranslations[i].getY();
      double moveY = dy + dTheta * moduleTranslations[i].getX();
      double length = Math.hypot(moveX, moveY);
      Rotation2d angle =
          length > 1e-6 ? Rotation2d.fromRadians(Math.atan2(moveY, moveX)) : lastPositions[i].angle;
      positions[i] = new SwerveModulePosition(lastPositions[i].distanceMeters + length, angle);
    }

    for (int i = 0; i < count; i++) {
      lastPositions[i] = positions[i];
      lastValid[i] = usable[i];
    }
    return new Sample(stamp, gyroRotation, positions.clone());
  }

  /**
   * Returns the robot-relative chassis speeds from the module states, leaving out modules whose
   * last sample was invalid. With every module valid this is the plain kinematics result.
   *
   * @param states each module's measured state
   * @param gyroRateRadPerSec the gyro's yaw rate, or null if the gyro is disconnected
   */
  ChassisSpeeds chassisSpeeds(SwerveModuleState[] states, Double gyroRateRadPerSec) {
    int count = moduleTranslations.length;
    if (validModuleCount == count) {
      return kinematics.toChassisSpeeds(states);
    }
    if (gyroRateRadPerSec != null) {
      double omega = gyroRateRadPerSec;
      double vx = 0.0;
      double vy = 0.0;
      for (int i = 0; i < count; i++) {
        if (lastValid[i]) {
          vx += states[i].speedMetersPerSecond * states[i].angle.getCos();
          vx += omega * moduleTranslations[i].getY();
          vy += states[i].speedMetersPerSecond * states[i].angle.getSin();
          vy -= omega * moduleTranslations[i].getX();
        }
      }
      return validModuleCount == 0
          ? new ChassisSpeeds(0.0, 0.0, omega)
          : new ChassisSpeeds(vx / validModuleCount, vy / validModuleCount, omega);
    }
    if (validModuleCount >= 2) {
      var validStates = new SwerveModuleState[validModuleCount];
      for (int i = 0, j = 0; i < count; i++) {
        if (lastValid[i]) {
          validStates[j++] = states[i];
        }
      }
      return kinematicsFor(lastValid).toChassisSpeeds(validStates);
    }
    for (int i = 0; i < count; i++) {
      if (lastValid[i]) {
        return new ChassisSpeeds(
            states[i].speedMetersPerSecond * states[i].angle.getCos(),
            states[i].speedMetersPerSecond * states[i].angle.getSin(),
            0.0);
      }
    }
    return new ChassisSpeeds();
  }

  /**
   * Returns the module positions to seed the estimator with on a pose reset, and continues from
   * them. A valid module uses its current measured position plus its offset, since queued samples
   * can lag the current position; a module that isn't valid keeps its estimated position.
   *
   * @param current each module's current measured position
   */
  SwerveModulePosition[] resetPositions(SwerveModulePosition[] current) {
    for (int i = 0; i < lastPositions.length; i++) {
      if (lastValid[i] && Double.isFinite(current[i].distanceMeters)) {
        lastPositions[i] =
            new SwerveModulePosition(
                current[i].distanceMeters + offsetsMeters[i], current[i].angle);
        lastValidRawMeters[i] = current[i].distanceMeters;
      }
    }
    return lastPositions();
  }

  /** Returns copies of the module positions the estimator last consumed. */
  SwerveModulePosition[] lastPositions() {
    var copies = new SwerveModulePosition[lastPositions.length];
    for (int i = 0; i < copies.length; i++) {
      copies[i] = lastPositions[i].copy();
    }
    return copies;
  }

  /** Returns the robot rotation from the gyro, or integrated from the wheels without one. */
  Rotation2d gyroRotation() {
    return gyroRotation;
  }

  /** Returns how many modules were valid in the last sample. */
  int validModuleCount() {
    return validModuleCount;
  }

  /** Returns how many times a module's offset has moved to absorb a jump. */
  long rebaseCount() {
    return rebaseCount;
  }

  /** Returns how many sample timestamps have been replaced with the loop time. */
  long substitutedTimestampCount() {
    return substitutedTimestampCount;
  }

  /** Returns the furthest a wheel can travel in the given time, capped at one second. */
  private static double maxWheelStep(double seconds) {
    double guardSeconds =
        Double.isFinite(seconds)
            ? MathUtil.clamp(seconds, 0.0, ODOMETRY_MAX_SAMPLE_AGE_SECONDS)
            : 0.0;
    return ODOMETRY_WHEEL_STEP_MARGIN_METERS + ODOMETRY_MAX_WHEEL_SPEED_MPS * guardSeconds;
  }

  private SwerveModulePosition[] validDeltas(
      boolean[] usable, double[] steps, SwerveModulePosition[] positions) {
    int validCount = 0;
    for (boolean u : usable) {
      validCount += u ? 1 : 0;
    }
    var deltas = new SwerveModulePosition[validCount];
    for (int i = 0, j = 0; i < usable.length; i++) {
      if (usable[i]) {
        deltas[j++] = new SwerveModulePosition(steps[i], positions[i].angle);
      }
    }
    return deltas;
  }

  /** Returns kinematics for the modules marked true, reusing the full set when all are. */
  private SwerveDriveKinematics kinematicsFor(boolean[] include) {
    int mask = 0;
    int included = 0;
    for (int i = 0; i < include.length; i++) {
      if (include[i]) {
        mask |= 1 << i;
        included++;
      }
    }
    if (included == include.length) {
      return kinematics;
    }
    return subsetKinematics.computeIfAbsent(
        mask,
        m -> {
          var translations = new Translation2d[Integer.bitCount(m)];
          for (int i = 0, j = 0; i < moduleTranslations.length; i++) {
            if ((m & (1 << i)) != 0) {
              translations[j++] = moduleTranslations[i];
            }
          }
          return new SwerveDriveKinematics(translations);
        });
  }
}
