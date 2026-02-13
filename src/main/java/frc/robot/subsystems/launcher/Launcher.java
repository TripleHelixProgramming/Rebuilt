package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.launcher.LauncherConstants.*;
import static frc.robot.subsystems.launcher.LauncherConstants.FlywheelConstants.*;
import static frc.robot.subsystems.launcher.LauncherConstants.TurretConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  private final TurretIO turretIO;
  private final FlywheelIO flywheelIO;
  private final HoodIO hoodIO;

  private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private final Supplier<Pose2d> chassisPoseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  private final Alert turretDisconnectedAlert;
  private final Alert flywheelDisconnectedAlert;
  private final Alert hoodDisconnectedAlert;

  private Translation3d vectorTurretBaseToTarget = new Translation3d();
  private Pose3d turretBasePose = new Pose3d();
  private Translation3d v0nominalLast = new Translation3d();
  private Translation3d v0replannedLast = new Translation3d();

  // Cached values for deferred logging (populated in aim(), logged in periodic())
  private Translation3d cachedBaseSpeeds = new Translation3d();
  private Translation3d cachedV0Nominal = new Translation3d();
  private Translation3d cachedV0Replanned = new Translation3d();
  private Translation3d cachedV0Actual = new Translation3d();
  private boolean cachedNominalReachable = false;
  private boolean cachedReplannedReachable = false;
  private double cachedPredictedRange = 0.0;

  // Fuel ballistics simulation
  private final ArrayList<BallisticObject> fuelNominal = new ArrayList<>();
  private final ArrayList<BallisticObject> fuelReplanned = new ArrayList<>();
  private final ArrayList<BallisticObject> fuelActual = new ArrayList<>();
  private double fuelSpawnTimer = 0.0;
  private double ballisticSimTimer = 0.0;
  private double ballisticLogTimer = 0.0;

  public Launcher(
      Supplier<Pose2d> chassisPoseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      TurretIO turretIO,
      FlywheelIO flywheelIO,
      HoodIO hoodIO) {
    this.turretIO = turretIO;
    this.flywheelIO = flywheelIO;
    this.hoodIO = hoodIO;

    this.chassisPoseSupplier = chassisPoseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;

    turretDisconnectedAlert = new Alert("Disconnected turret motor", AlertType.kError);
    flywheelDisconnectedAlert = new Alert("Disconnected flywheel motor", AlertType.kError);
    hoodDisconnectedAlert = new Alert("Disconnected hood motor", AlertType.kError);
  }

  @Override
  public void periodic() {
    long t0 = System.nanoTime();
    turretIO.updateInputs(turretInputs);
    long t1 = System.nanoTime();
    flywheelIO.updateInputs(flywheelInputs);
    long t2 = System.nanoTime();
    hoodIO.updateInputs(hoodInputs);
    long t3 = System.nanoTime();

    Logger.processInputs("Turret", turretInputs);
    long t4 = System.nanoTime();
    Logger.processInputs("Flywheel", flywheelInputs);
    long t5 = System.nanoTime();
    Logger.processInputs("Hood", hoodInputs);
    long t6 = System.nanoTime();

    turretDisconnectedAlert.set(!turretInputs.motorControllerConnected);
    flywheelDisconnectedAlert.set(!flywheelInputs.connected);
    hoodDisconnectedAlert.set(!hoodInputs.connected);

    // Log aim data (deferred from aim() to keep hot path fast)
    logAimData();

    // Profiling output
    long turretMs = (t1 - t0) / 1_000_000;
    long flywheelMs = (t2 - t1) / 1_000_000;
    long hoodMs = (t3 - t2) / 1_000_000;
    long turretLogMs = (t4 - t3) / 1_000_000;
    long flywheelLogMs = (t5 - t4) / 1_000_000;
    long hoodLogMs = (t6 - t5) / 1_000_000;
    long totalMs = (t6 - t0) / 1_000_000;
    if (totalMs > 3) {
      System.out.println(
          "[Launcher] turret="
              + turretMs
              + "ms flywheel="
              + flywheelMs
              + "ms hood="
              + hoodMs
              + "ms turretLog="
              + turretLogMs
              + "ms flywheelLog="
              + flywheelLogMs
              + "ms hoodLog="
              + hoodLogMs
              + "ms");
    }

    // Update and plot ball trajectories
    if (logFuelTrajectories) {
      double dt = Robot.defaultPeriodSecs;

      ballisticSimTimer += dt;
      ballisticLogTimer += dt;

      boolean doSim = ballisticSimTimer >= ballisticSimPeriod;
      boolean doLog = ballisticLogTimer >= ballisticLogPeriod;

      if (doSim) {
        ballisticSimTimer = 0.0;

        updateBallisticsSim(fuelNominal, nominalKey, ballisticSimPeriod, doLog);
        updateBallisticsSim(fuelReplanned, replannedKey, ballisticSimPeriod, doLog);
        updateBallisticsSim(fuelActual, actualKey, ballisticSimPeriod, doLog);
      }

      if (doLog) {
        ballisticLogTimer = 0.0;
      }
    }
  }

  public void stop() {
    turretIO.setOpenLoop(0.0);
    flywheelIO.setOpenLoop(0.0);
    hoodIO.setOpenLoop(0.0);
  }

  public void aim(Translation3d target) {
    long aimStart = System.nanoTime();

    // Get vector from static target to turret
    turretBasePose = new Pose3d(chassisPoseSupplier.get()).plus(chassisToTurretBase);
    vectorTurretBaseToTarget = target.minus(turretBasePose.getTranslation());

    // Set flywheel speed assuming a motionless robot
    long t1 = System.nanoTime();
    var v0_nominal = getV0Nominal(vectorTurretBaseToTarget, impactAngle);
    long t2 = System.nanoTime();
    AngularVelocity flywheelSetpoint =
        RadiansPerSecond.of(v0_nominal.getNorm() / wheelRadius.in(Meters));
    flywheelIO.setVelocity(flywheelSetpoint);
    long t3 = System.nanoTime();

    // Get translation velocities (m/s) of the turret caused by motion of the chassis
    var robotRelative = chassisSpeedsSupplier.get();
    var fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelative, turretBasePose.toPose2d().getRotation());
    var v_base = getTurretBaseSpeeds(turretBasePose.toPose2d().getRotation(), fieldRelative);
    long t4 = System.nanoTime();

    // Get actual flywheel speed
    double flywheelSpeedMetersPerSec = flywheelInputs.velocityRadPerSec * wheelRadius.in(Meters);

    // Replan shot using actual flywheel speed
    var v0_total = getV0Replanned(vectorTurretBaseToTarget, flywheelSpeedMetersPerSec);
    long t5 = System.nanoTime();

    // Point turret to align velocity vectors
    var v0_flywheel = v0_total.minus(v_base);

    // Check if v0_flywheel has non-zero horizontal component
    double v0_horizontal = Math.hypot(v0_flywheel.getX(), v0_flywheel.getY());
    if (!Double.isFinite(v0_horizontal) || v0_horizontal < 1e-6) {
      // Flywheel velocity is too low or target unreachable, stop mechanisms
      return;
    }

    Rotation2d turretSetpoint = new Rotation2d(v0_flywheel.getX(), v0_flywheel.getY());
    turretIO.setPosition(
        turretSetpoint.minus(turretBasePose.toPose2d().getRotation()),
        RadiansPerSecond.of(robotRelative.omegaRadiansPerSecond).unaryMinus().times(2.0));
    Rotation2d hoodSetpoint = new Rotation2d(v0_horizontal, v0_flywheel.getZ());
    hoodIO.setPosition(hoodSetpoint, RadiansPerSecond.of(0));
    long t6 = System.nanoTime();

    // Get actual hood & turret position
    Rotation2d hoodPosition = hoodInputs.position;
    Rotation2d turretPosition =
        turretInputs.relativePosition.plus(turretBasePose.toPose2d().getRotation());

    // Get mechanism angles
    double hoodCos = hoodPosition.getCos();
    double hoodSin = hoodPosition.getSin();
    double turretCos = turretPosition.getCos();
    double turretSin = turretPosition.getSin();

    // Build actual velocities and cache for deferred logging
    double vx = hoodCos * turretCos * flywheelSpeedMetersPerSec + v_base.getX();
    double vy = hoodCos * turretSin * flywheelSpeedMetersPerSec + v_base.getY();
    double vz = hoodSin * flywheelSpeedMetersPerSec;
    cachedV0Actual = new Translation3d(vx, vy, vz);

    // Profiling output
    long aimEnd = System.nanoTime();
    long totalUs = (aimEnd - aimStart) / 1_000;
    if (totalUs > 500) { // Log if aim() takes > 0.5ms
      System.out.println(
          "[Launcher.aim] setup="
              + (t1 - aimStart) / 1_000
              + "us v0nom="
              + (t2 - t1) / 1_000
              + "us flywheelSet="
              + (t3 - t2) / 1_000
              + "us baseSpeeds="
              + (t4 - t3) / 1_000
              + "us v0replan="
              + (t5 - t4) / 1_000
              + "us setPos="
              + (t6 - t5) / 1_000
              + "us rest="
              + (aimEnd - t6) / 1_000
              + "us total="
              + totalUs
              + "us");
    }

    // Spawn simulated fuel
    fuelSpawnTimer += Robot.defaultPeriodSecs;
    if (fuelSpawnTimer >= fuelSpawnPeriod && logFuelTrajectories) {
      fuelSpawnTimer = 0.0;

      fuelNominal.add(
          new BallisticObject(turretBasePose.getTranslation(), v0_nominal, target.getMeasureZ()));
      fuelReplanned.add(
          new BallisticObject(turretBasePose.getTranslation(), v0_total, target.getMeasureZ()));
      fuelActual.add(
          new BallisticObject(
              turretBasePose.getTranslation(), cachedV0Actual, target.getMeasureZ()));
    }
  }

  @AutoLogOutput(key = "Launcher/TurretPose")
  public Pose2d getTurretPose() {
    return turretBasePose.toPose2d().plus(new Transform2d(0, 0, turretInputs.relativePosition));
  }

  private Translation3d getTurretBaseSpeeds(Rotation2d rotation, ChassisSpeeds chassisSpeeds) {
    double vx = chassisSpeeds.vxMetersPerSecond;
    double vy = chassisSpeeds.vyMetersPerSecond;
    double omega = chassisSpeeds.omegaRadiansPerSecond;

    double rx = chassisToTurretBase.getX();
    double ry = chassisToTurretBase.getY();

    double offX = -omega * ry;
    double offY = omega * rx;

    double cos = rotation.getCos();
    double sin = rotation.getSin();

    double baseVx = vx + offX * cos - offY * sin;
    double baseVy = vy + offX * sin + offY * cos;

    cachedBaseSpeeds = new Translation3d(baseVx, baseVy, 0);
    return cachedBaseSpeeds;
  }

  private Translation3d getV0Nominal(Translation3d d, Rotation2d impactAngle) {
    double dr = Math.hypot(d.getX(), d.getY());
    if (dr < 1e-6) {
      cachedNominalReachable = false;
      return v0nominalLast;
    }

    double dz = d.getZ();

    double denominator = 2 * (dz + dr * impactAngle.getTan());
    if (denominator <= 0) {
      cachedNominalReachable = false;
      return v0nominalLast;
    }
    double v_0r = dr * Math.sqrt(g / denominator);
    double v_0z = (g * dr) / v_0r - v_0r * impactAngle.getTan();

    double v_0x = v_0r * d.toTranslation2d().getAngle().getCos();
    double v_0y = v_0r * d.toTranslation2d().getAngle().getSin();

    v0nominalLast = new Translation3d(v_0x, v_0y, v_0z);
    cachedNominalReachable = true;
    cachedV0Nominal = v0nominalLast;
    return v0nominalLast;
  }

  private Translation3d getV0Replanned(Translation3d d, double v_flywheel) {
    // Geometry
    double dr = Math.hypot(d.getX(), d.getY());
    if (dr < 1e-6) {
      cachedReplannedReachable = false;
      return v0replannedLast;
    }

    double dz = d.getZ();

    // Unit vectors toward target in XY plane
    double rHatX = d.getX() / dr;
    double rHatY = d.getY() / dr;

    double v_sq = v_flywheel * v_flywheel;
    double discriminant = v_sq * v_sq - g * (g * dr * dr + 2 * dz * v_sq);

    if (discriminant < 0) {
      // Unreachable target at this speed
      cachedReplannedReachable = false;
      return v0replannedLast;
    }

    // High-arc solution
    double tanTheta = (v_sq + Math.sqrt(discriminant)) / (g * dr);

    cachedPredictedRange = (v_sq * Math.sin(2 * Math.atan(tanTheta))) / g;

    // Effective velocity available for ballistics
    double v_r = v_flywheel / Math.sqrt(1 + tanTheta * tanTheta);
    double v_z = v_r * tanTheta;

    double v_required = Math.hypot(v_r, v_z);

    if (v_required < 1e-6) {
      cachedReplannedReachable = false;
      return v0replannedLast;
    }

    // Scale to match actual flywheel speed
    double scale = v_flywheel / v_required;

    v_r *= scale;
    v_z *= scale;

    // Back to field frame
    v0replannedLast = new Translation3d(rHatX * v_r, rHatY * v_r, v_z);
    cachedReplannedReachable = true;
    cachedV0Replanned = v0replannedLast;
    return v0replannedLast;
  }

  /** Logs all cached aim data. Called from periodic() to keep logging out of the hot path. */
  private void logAimData() {
    Logger.recordOutput("Launcher/BaseSpeeds", cachedBaseSpeeds);

    // Log nominal trajectory data
    Logger.recordOutput("Launcher/" + nominalKey + "/Reachable", cachedNominalReachable);
    if (cachedNominalReachable) {
      logTrajectory(cachedV0Nominal, nominalKey);
    }

    // Log replanned trajectory data
    Logger.recordOutput("Launcher/" + replannedKey + "/Reachable", cachedReplannedReachable);
    if (cachedReplannedReachable) {
      Logger.recordOutput("Launcher/" + replannedKey + "/PredictedRange", cachedPredictedRange);
      logTrajectory(cachedV0Replanned, replannedKey);
    }

    // Log actual trajectory data
    logTrajectory(cachedV0Actual, actualKey);
  }

  private void logTrajectory(Translation3d v, String key) {
    Logger.recordOutput("Launcher/" + key + "/InitialVelocities", v);
    Logger.recordOutput("Launcher/" + key + "/InitialSpeedMetersPerSecond", v.getNorm());

    var v_r = v.toTranslation2d().getNorm();
    var v_z = v.getZ();

    // Skip angle calculations if velocity is effectively zero to avoid Rotation2d error
    if (v_r < 1e-6 && Math.abs(v_z) < 1e-6) {
      return;
    }

    if (v_r >= 1e-6) {
      Logger.recordOutput(
          "Launcher/" + key + "/HorizontalLaunchAngleDegrees",
          v.toTranslation2d().getAngle().getDegrees());
      double dr = Math.hypot(vectorTurretBaseToTarget.getX(), vectorTurretBaseToTarget.getY());
      Logger.recordOutput("Launcher/" + key + "/TravelTime", dr / v_r);
    }

    if (v_r >= 1e-6 || Math.abs(v_z) >= 1e-6) {
      Logger.recordOutput(
          "Launcher/" + key + "/VerticalLaunchAngleDegrees",
          new Translation2d(v_r, v_z).getAngle().getDegrees());
    }

    var max_height = turretBasePose.getZ() + v_z * v_z / (2 * g);
    Logger.recordOutput("Launcher/" + key + "/MaxHeight", max_height);

    boolean clearsCeiling = Meters.of(max_height).plus(fuelRadius).lt(ceilingHeight);
    Logger.recordOutput("Launcher/" + key + "/ClearsCeiling", clearsCeiling);
  }

  private static class BallisticObject {
    double px, py, pz;
    double vx, vy, vz;
    double targetHeight;

    BallisticObject(Translation3d position, Translation3d velocity, Distance targetHeight) {
      this.px = position.getX();
      this.py = position.getY();
      this.pz = position.getZ();
      this.vx = velocity.getX();
      this.vy = velocity.getY();
      this.vz = velocity.getZ();
      this.targetHeight = targetHeight.in(Meters);
    }

    private Translation3d getPosition() {
      return new Translation3d(px, py, pz);
    }
  }

  private void updateBallisticsSim(
      ArrayList<BallisticObject> traj, String key, double dt, boolean log) {

    for (int i = traj.size() - 1; i >= 0; i--) {
      BallisticObject o = traj.get(i);

      o.vz -= g * dt;
      o.px += o.vx * dt;
      o.py += o.vy * dt;
      o.pz += o.vz * dt;

      if (o.pz < o.targetHeight && o.vz < 0) {
        traj.remove(i);
      }
    }

    if (log) {
      Logger.recordOutput("Launcher/" + key + "/FuelTrajectory", getBallTrajectory(traj));
    }
  }

  public Translation3d[] getBallTrajectory(ArrayList<BallisticObject> traj) {
    Translation3d[] t = new Translation3d[traj.size()];
    for (int i = 0; i < traj.size(); i++) {
      t[i] = traj.get(i).getPosition();
    }
    return t;
  }
}
