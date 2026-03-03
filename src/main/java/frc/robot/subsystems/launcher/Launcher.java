package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.launcher.LauncherConstants.*;
import static frc.robot.subsystems.launcher.LauncherConstants.FlywheelConstants.*;
import static frc.robot.subsystems.launcher.LauncherConstants.HoodConstants.ballToHoodOffset;
import static frc.robot.subsystems.launcher.LauncherConstants.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  private Rotation2d horizontalAimAngle = Rotation2d.kZero;

  // Setpoint tracking for isOnTarget() tolerance check
  private Rotation2d turretSetpoint = Rotation2d.kZero;
  private Rotation2d hoodSetpoint = Rotation2d.kZero;

  // Cached values for deferred logging (populated in aim(), logged in periodic())
  private boolean hasCachedAimData = false;
  private Translation3d cachedBaseSpeeds = new Translation3d();
  private boolean cachedFlywheelVelocityTooLow = false;
  private Translation3d cachedActualD = new Translation3d();
  private Translation3d cachedActualV = new Translation3d();

  // Fuel ballistics simulation
  private final ArrayList<BallisticObject> fuelNominal = new ArrayList<>();
  private final ArrayList<BallisticObject> fuelReplanned = new ArrayList<>();
  private final ArrayList<BallisticObject> fuelActual = new ArrayList<>();
  private double fuelSpawnTimer = 0.0;
  private double ballisticSimTimer = 0.0;
  private double ballisticLogTimer = 0.0;

  // Turret desaturation
  private final PIDController headingController = new PIDController(1.5, 0.0, 0.1);

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

    headingController.setTolerance(marginRad);
  }

  @Override
  public void periodic() {
    long t0 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;

    turretIO.updateInputs(turretInputs);
    flywheelIO.updateInputs(flywheelInputs);
    hoodIO.updateInputs(hoodInputs);
    long t1 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;

    Logger.processInputs("Turret", turretInputs);
    Logger.processInputs("Flywheel", flywheelInputs);
    Logger.processInputs("Hood", hoodInputs);
    long t2 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;

    turretDisconnectedAlert.set(!turretInputs.motorControllerConnected);
    flywheelDisconnectedAlert.set(!flywheelInputs.connected);
    hoodDisconnectedAlert.set(!hoodInputs.connected);

    // Log cached aim data (deferred from aim() to avoid Logger overhead in hot path)
    if (hasCachedAimData) {
      logCachedAimData();
      hasCachedAimData = false;
    }
    long t3 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;

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
    long t4 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Profiling output
    if (Constants.PROFILING_ENABLED) {
      long totalMs = (t4 - t0) / 1_000_000;
      if (totalMs > 3) {
        System.out.println(
            "[Launcher] update="
                + (t1 - t0) / 1_000_000
                + "ms log="
                + (t2 - t1) / 1_000_000
                + "ms aimLog="
                + (t3 - t2) / 1_000_000
                + "ms ballistics="
                + (t4 - t3) / 1_000_000
                + "ms total="
                + totalMs
                + "ms");
      }
    }
  }

  public void stop() {
    turretIO.setOpenLoop(Volts.of(0.0));
    flywheelIO.setOpenLoop(Volts.of(0.0));
    hoodIO.setOpenLoop(Volts.of(0.0));
  }

  public void aim(Translation3d target) {
    long aimStart = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Get vector from static target to turret
    turretBasePose = new Pose3d(chassisPoseSupplier.get()).plus(chassisToTurretBase);
    vectorTurretBaseToTarget = target.minus(turretBasePose.getTranslation());

    // Compute distance-based impact angle
    double horizontalDistance =
        Math.hypot(vectorTurretBaseToTarget.getX(), vectorTurretBaseToTarget.getY());
    Rotation2d dynamicImpactAngle = getImpactAngle(horizontalDistance);

    // Set flywheel speed assuming a motionless robot
    var v0_nominal = getV0Nominal(vectorTurretBaseToTarget, dynamicImpactAngle, nominalKey);
    var flywheelSetpoint = MetersPerSecond.of(flywheelSetpointfromBallistics(v0_nominal.getNorm()));
    flywheelIO.setVelocity(flywheelSetpoint);
    long t1 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Get translation velocities (m/s) of the turret caused by motion of the chassis
    var robotRelative = chassisSpeedsSupplier.get();
    var fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelative, turretBasePose.toPose2d().getRotation());
    var v_base = getTurretBaseSpeeds(turretBasePose.toPose2d().getRotation(), fieldRelative);
    long t2 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Get actual initial shot speed
    double initialSpeedMetersPerSec =
        ballisticsFromFlywheelSetpoint(flywheelInputs.velocityMetersPerSec);

    // Replan shot using actual initial shot speed speed
    var v0_total = getV0Replanned(vectorTurretBaseToTarget, initialSpeedMetersPerSec, replannedKey);
    long t3 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Point turret to align velocity vectors
    var v0_flywheel = v0_total.minus(v_base);

    // Check if v0_flywheel has non-zero horizontal component
    double v0_horizontal = Math.hypot(v0_flywheel.getX(), v0_flywheel.getY());
    if (!Double.isFinite(v0_horizontal) || v0_horizontal < 1e-6) {
      // Flywheel velocity is too low or target unreachable, cache for deferred logging
      cachedFlywheelVelocityTooLow = true;
      hasCachedAimData = true;
      return;
    }
    cachedFlywheelVelocityTooLow = false;

    horizontalAimAngle = new Rotation2d(v0_flywheel.getX(), v0_flywheel.getY());
    turretSetpoint = horizontalAimAngle.minus(turretBasePose.toPose2d().getRotation());
    turretIO.setPosition(
        turretSetpoint,
        RadiansPerSecond.of(robotRelative.omegaRadiansPerSecond).unaryMinus().times(2.0));
    hoodSetpoint = new Rotation2d(v0_horizontal, v0_flywheel.getZ()).minus(ballToHoodOffset);
    hoodIO.setPosition(hoodSetpoint, RadiansPerSecond.of(0));
    long t4 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Get actual hood & turret position
    Rotation2d hoodPosition = hoodInputs.position.plus(ballToHoodOffset);
    Rotation2d turretPosition =
        turretInputs.relativePosition.plus(turretBasePose.toPose2d().getRotation());

    // Get mechanism angles
    double hoodCos = hoodPosition.getCos();
    double hoodSin = hoodPosition.getSin();
    double turretCos = turretPosition.getCos();
    double turretSin = turretPosition.getSin();

    // Build actual velocities
    double vx = hoodCos * turretCos * initialSpeedMetersPerSec + v_base.getX();
    double vy = hoodCos * turretSin * initialSpeedMetersPerSec + v_base.getY();
    double vz = hoodSin * initialSpeedMetersPerSec;
    Translation3d v0_actual = new Translation3d(vx, vy, vz);

    // Cache values for deferred logging (will be logged in periodic())
    cachedActualD = vectorTurretBaseToTarget;
    cachedActualV = v0_actual;
    hasCachedAimData = true;
    long t5 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Spawn simulated fuel
    fuelSpawnTimer += Robot.defaultPeriodSecs;
    if (fuelSpawnTimer >= fuelSpawnPeriod && logFuelTrajectories) {
      fuelSpawnTimer = 0.0;

      fuelNominal.add(
          new BallisticObject(turretBasePose.getTranslation(), v0_nominal, target.getMeasureZ()));
      fuelReplanned.add(
          new BallisticObject(turretBasePose.getTranslation(), v0_total, target.getMeasureZ()));
      fuelActual.add(
          new BallisticObject(turretBasePose.getTranslation(), v0_actual, target.getMeasureZ()));
    }

    // Profiling output for aim()
    if (Constants.PROFILING_ENABLED) {
      long totalUs = (t5 - aimStart) / 1_000;
      if (totalUs > 500) {
        System.out.println(
            "[Launcher.aim] v0nom="
                + (t1 - aimStart) / 1_000
                + "us baseSpeeds="
                + (t2 - t1) / 1_000
                + "us v0replan="
                + (t3 - t2) / 1_000
                + "us setPos="
                + (t4 - t3) / 1_000
                + "us rest="
                + (t5 - t4) / 1_000
                + "us total="
                + totalUs
                + "us");
      }
    }
  }

  @AutoLogOutput(key = "Launcher/TurretPose")
  public Pose2d getTurretPose() {
    return new Pose2d(
        turretBasePose.getX(),
        turretBasePose.getY(),
        turretBasePose.getRotation().toRotation2d().plus(turretInputs.relativePosition));
  }

  @AutoLogOutput(key = "Launcher/HorizontalAimAngle")
  public Rotation2d getHorizontalAimAngle() {
    return horizontalAimAngle;
  }

  public double desaturateTurret() {
    return headingController.calculate(-turretInputs.oversaturationLessMargin, 0);
  }

  public boolean turretDesaturated() {
    // return headingController.atSetpoint();
    return Math.abs(turretInputs.oversaturation) < Units.degreesToRadians(8);
  }

  @AutoLogOutput(key = "Launcher/IsOnTarget")
  public boolean isOnTarget() {
    double tolerance = isOnTargetTolerance.in(Radians);
    double hoodError = Math.abs(hoodInputs.position.minus(hoodSetpoint).getRadians());
    double turretError = Math.abs(turretInputs.relativePosition.minus(turretSetpoint).getRadians());
    return hoodError < tolerance && turretError < tolerance;
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

    var baseSpeeds = new Translation3d(baseVx, baseVy, 0);

    // Cache for deferred logging (will be logged in periodic())
    cachedBaseSpeeds = baseSpeeds;

    return baseSpeeds;
  }

  private Rotation2d getImpactAngle(double horizontalDistanceMeters) {
    double t =
        (horizontalDistanceMeters - impactAngleCloseDistance.in(Meters))
            / (impactAngleFarDistance.in(Meters) - impactAngleCloseDistance.in(Meters));
    t = MathUtil.clamp(t, 0.0, 1.0);

    double angleDeg =
        MathUtil.interpolate(impactAngleClose.getDegrees(), impactAngleFar.getDegrees(), t);
    return Rotation2d.fromDegrees(angleDeg);
  }

  private Translation3d getV0Nominal(Translation3d d, Rotation2d impactAngle, String key) {
    double dr = Math.hypot(d.getX(), d.getY());
    if (dr < 1e-6) {
      Logger.recordOutput("Launcher/" + key + "/Reachable", false);
      // log(d, v0, key);
      return v0nominalLast;
    }

    double dz = d.getZ();

    double denominator = 2 * (dz + dr * impactAngle.getTan());
    if (denominator <= 0) {
      Logger.recordOutput("Launcher/" + key + "/Reachable", false);
      // log(d, v0, key);
      return v0nominalLast;
    }
    double v_0r = dr * Math.sqrt(g / denominator);
    double v_0z = (g * dr) / v_0r - v_0r * impactAngle.getTan();

    double v_0x = v_0r * d.toTranslation2d().getAngle().getCos();
    double v_0y = v_0r * d.toTranslation2d().getAngle().getSin();

    v0nominalLast = new Translation3d(v_0x, v_0y, v_0z);
    Logger.recordOutput("Launcher/" + key + "/Reachable", true);
    log(d, v0nominalLast, key);
    return v0nominalLast;
  }

  private Translation3d getV0Replanned(Translation3d d, double v_flywheel, String key) {
    // Geometry
    double dr = Math.hypot(d.getX(), d.getY());
    if (dr < 1e-6) {
      Logger.recordOutput("Launcher/" + key + "/Reachable", false);
      // log(d, v0, key);
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
      Logger.recordOutput("Launcher/" + key + "/Reachable", false);
      // log(d, v0, key);
      return v0replannedLast;
    }

    // High-arc solution
    double tanTheta = (v_sq + Math.sqrt(discriminant)) / (g * dr);

    Logger.recordOutput(
        "Launcher/" + key + "/PredictedRange", (v_sq * Math.sin(2 * Math.atan(tanTheta))) / g);

    // Effective velocity available for ballistics
    double v_r = v_flywheel / Math.sqrt(1 + tanTheta * tanTheta);
    double v_z = v_r * tanTheta;

    double v_required = Math.hypot(v_r, v_z);

    if (v_required < 1e-6) {
      Logger.recordOutput("Launcher/" + key + "/Reachable", false);
      // log(d, v0, key);
      return v0replannedLast;
    }

    // Scale to match actual flywheel speed
    double scale = v_flywheel / v_required;

    v_r *= scale;
    v_z *= scale;

    // Back to field frame
    v0replannedLast = new Translation3d(rHatX * v_r, rHatY * v_r, v_z);
    Logger.recordOutput("Launcher/" + key + "/Reachable", true);
    log(d, v0replannedLast, key);
    return v0replannedLast;
  }

  /** Logs cached aim data that was deferred from aim() to avoid Logger overhead in hot path. */
  private void logCachedAimData() {
    Logger.recordOutput("Launcher/BaseSpeeds", cachedBaseSpeeds);
    Logger.recordOutput("Launcher/Flywheel velocity too low", cachedFlywheelVelocityTooLow);

    if (!cachedFlywheelVelocityTooLow) {
      log(cachedActualD, cachedActualV, actualKey);
    }
  }

  private void log(Translation3d d, Translation3d v, String key) {
    Logger.recordOutput("Launcher/" + key + "/InitialVelocities", v);
    Logger.recordOutput("Launcher/" + key + "/InitialSpeedMetersPerSecond", v.getNorm());

    var v_r = v.toTranslation2d().getNorm();
    var v_z = v.getZ();

    // Zero-velocity guard: avoid getAngle() on zero-length vectors
    if (v_r < 1e-6) {
      Logger.recordOutput("Launcher/" + key + "/VerticalLaunchAngleDegrees", 0.0);
      Logger.recordOutput("Launcher/" + key + "/HorizontalLaunchAngleDegrees", 0.0);
      Logger.recordOutput("Launcher/" + key + "/TravelTime", 0.0);
    } else {
      Logger.recordOutput(
          "Launcher/" + key + "/VerticalLaunchAngleDegrees",
          new Translation2d(v_r, v_z).getAngle().getDegrees());
      Logger.recordOutput(
          "Launcher/" + key + "/HorizontalLaunchAngleDegrees",
          v.toTranslation2d().getAngle().getDegrees());

      double dr = Math.hypot(d.getX(), d.getY());
      Logger.recordOutput("Launcher/" + key + "/TravelTime", dr / v_r);
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

  public Command initializeHoodCommand() {
    return new StartEndCommand(
            // initialize
            () -> {
              hoodIO.configureSoftLimits(false);
              hoodIO.setOpenLoop(Volts.of(1.0));
            },
            // end
            () -> {
              hoodIO.configureSoftLimits(true);
              hoodIO.resetEncoder();
            },
            // requirements
            this)
        .withTimeout(1.0)
        .withName("Initialize hood");
  }

  private double flywheelSetpointfromBallistics(double ballistics) {
    return FlywheelScaling.coefficient * Math.pow(ballistics, FlywheelScaling.exponent);
  }

  private double ballisticsFromFlywheelSetpoint(double setpoint) {
    return Math.pow(setpoint / FlywheelScaling.coefficient, 1.0 / FlywheelScaling.exponent);
  }
}
