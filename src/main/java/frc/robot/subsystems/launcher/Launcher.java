package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.launcher.LauncherConstants.*;
import static frc.robot.subsystems.launcher.LauncherConstants.FlywheelConstants.*;
import static frc.robot.subsystems.launcher.LauncherConstants.HoodConstants.ballToHoodOffset;
import static frc.robot.subsystems.launcher.LauncherConstants.TurretConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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
    turretIO.updateInputs(turretInputs);
    flywheelIO.updateInputs(flywheelInputs);
    hoodIO.updateInputs(hoodInputs);

    Logger.processInputs("Turret", turretInputs);
    Logger.processInputs("Flywheel", flywheelInputs);
    Logger.processInputs("Hood", hoodInputs);

    turretDisconnectedAlert.set(!turretInputs.motorControllerConnected);
    flywheelDisconnectedAlert.set(!flywheelInputs.connected);
    hoodDisconnectedAlert.set(!hoodInputs.connected);

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
    turretIO.setOpenLoop(Volts.of(0.0));
    flywheelIO.setOpenLoop(Volts.of(0.0));
    hoodIO.setOpenLoop(Volts.of(0.0));
  }

  public void aim(Translation3d target) {
    // Get vector from static target to turret
    turretBasePose = new Pose3d(chassisPoseSupplier.get()).plus(chassisToTurretBase);
    vectorTurretBaseToTarget = target.minus(turretBasePose.getTranslation());

    // Set flywheel speed assuming a motionless robot
    var v0_nominal = getV0(vectorTurretBaseToTarget, impactAngle, nominalKey);
    flywheelIO.setVelocity(MetersPerSecond.of(ballToFlywheelFactor * v0_nominal.getNorm()));

    // Get translation velocities (m/s) of the turret caused by motion of the chassis
    var robotRelative = chassisSpeedsSupplier.get();
    var fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelative, turretBasePose.toPose2d().getRotation());
    var v_base = getTurretBaseSpeeds(turretBasePose.toPose2d().getRotation(), fieldRelative);

    // Get actual flywheel speed
    double flywheelSpeedMetersPerSec = flywheelInputs.velocityMetersPerSec / ballToFlywheelFactor;

    // Replan shot using actual flywheel speed
    var v0_total = getV0(vectorTurretBaseToTarget, flywheelSpeedMetersPerSec, replannedKey);

    // Point turret to align velocity vectors
    // var v0_flywheel = v0_total.minus(v_base);
    var v0_flywheel = v0_nominal.minus(v_base);

    // Check if v0_flywheel has non-zero horizontal component
    double v0_horizontal = Math.hypot(v0_flywheel.getX(), v0_flywheel.getY());
    if (!Double.isFinite(v0_horizontal) || v0_horizontal < 1e-6) {
      // Flywheel velocity is too low or target unreachable, stop mechanisms
      Logger.recordOutput("Launcher/Flywheel velocity too low", true);
      return;
    }
    Logger.recordOutput("Launcher/Flywheel velocity too low", false);

    Rotation2d turretSetpoint = new Rotation2d(v0_flywheel.getX(), v0_flywheel.getY());
    turretIO.setPosition(
        turretSetpoint.minus(turretBasePose.toPose2d().getRotation()),
        RadiansPerSecond.of(robotRelative.omegaRadiansPerSecond).unaryMinus().times(2.0));
    Rotation2d hoodSetpoint = new Rotation2d(v0_horizontal, v0_flywheel.getZ());
    hoodIO.setPosition(hoodSetpoint.minus(ballToHoodOffset), RadiansPerSecond.of(0));

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
    double vx = hoodCos * turretCos * flywheelSpeedMetersPerSec + v_base.getX();
    double vy = hoodCos * turretSin * flywheelSpeedMetersPerSec + v_base.getY();
    double vz = hoodSin * flywheelSpeedMetersPerSec;
    Translation3d v0_actual = new Translation3d(vx, vy, vz);
    log(vectorTurretBaseToTarget, v0_actual, actualKey);

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
  }

  @AutoLogOutput(key = "Launcher/TurretPose")
  public Pose2d getTurretPose() {
    return turretBasePose.toPose2d().plus(new Transform2d(0, 0, turretInputs.relativePosition));
  }

  // @AutoLogOutput(key = "Turret/IsOnTarget")
  // public boolean isOnTarget() {
  //   return turretInputs.position.minus(turretOrientationSetpoint).getMeasure().abs(Radians)
  //           * dynamicTargetToTurretBase.getNorm()
  //       < (hubWidth.in(Meters) / 2.0);
  // }

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

    Logger.recordOutput("Launcher/BaseSpeeds", baseSpeeds);

    return baseSpeeds;
  }

  private Translation3d getV0(Translation3d d, Rotation2d impactAngle, String key) {
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

  private Translation3d getV0(Translation3d d, double v_flywheel, String key) {
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

  private void log(Translation3d d, Translation3d v, String key) {
    Logger.recordOutput("Launcher/" + key + "/InitialVelocities", v);
    Logger.recordOutput("Launcher/" + key + "/InitialSpeedMetersPerSecond", v.getNorm());

    var v_r = v.toTranslation2d().getNorm();
    var v_z = v.getZ();
    Logger.recordOutput(
        "Launcher/" + key + "/VerticalLaunchAngleDegrees",
        new Translation2d(v_r, v_z).getAngle().getDegrees());
    Logger.recordOutput(
        "Launcher/" + key + "/HorizontalLaunchAngleDegrees",
        v.toTranslation2d().getAngle().getDegrees());

    double dr = Math.hypot(d.getX(), d.getY());
    Logger.recordOutput("Launcher/" + key + "/TravelTime", dr / v_r);

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

  public Command initializeHoodCommand(Runnable action) {
    return new FunctionalCommand(
            // initialize
            () -> {
              hoodIO.configureSoftLimits(false);
              hoodIO.setOpenLoop(Volts.of(1.0));
            },
            // execute
            () -> {},
            // end
            interrupted -> {
              hoodIO.configureSoftLimits(true);
              hoodIO.resetEncoder();

              this.setDefaultCommand(Commands.run(action, this).withName("Aim at hub"));
            },
            // isFinished
            // () -> hoodInputs.currentAmps > 15.0 && Math.abs(hoodInputs.velocityRadPerSec) < 0.01,
            () -> false,
            // requirements
            this)
        .withTimeout(1.0)
        .withName("Initialize hood");
  }
}
