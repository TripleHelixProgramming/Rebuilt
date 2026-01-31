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
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.game.GameState;
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

  private Translation3d target = new Translation3d();
  private Translation3d vectorTurretBaseToTarget = new Translation3d();
  private Pose3d turretBasePose = new Pose3d();
  // private Translation3d v0_nominal = new Translation3d();

  // Fuel ballistics simulation
  private final ArrayList<BallisticObject> fuelNominal = new ArrayList<>();
  private final ArrayList<BallisticObject> fuelReplanned = new ArrayList<>();
  private final ArrayList<BallisticObject> fuelActual = new ArrayList<>();
  private double fuelSpawnTimer = 0.0;

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

    turretDisconnectedAlert.set(!turretInputs.connected);
    flywheelDisconnectedAlert.set(!flywheelInputs.connected);
    hoodDisconnectedAlert.set(!hoodInputs.connected);

    // Update and plot ball trajectories
    updateBallisticsSim(fuelNominal, nominalKey);
    updateBallisticsSim(fuelReplanned, replannedKey);
    updateBallisticsSim(fuelActual, actualKey);
  }

  public void stop() {
    turretIO.setOpenLoop(0.0);
    flywheelIO.setOpenLoop(0.0);
    hoodIO.setOpenLoop(0.0);
  }

  public void aim(Translation3d target) {
    this.target = target;

    // Get vector from static target to turret
    turretBasePose = new Pose3d(chassisPoseSupplier.get()).plus(chassisToTurretBase);
    vectorTurretBaseToTarget = target.minus(turretBasePose.getTranslation());

    // Set flywheel speed assuming a motionless robot
    var v0_nominal = getV0(vectorTurretBaseToTarget, impactAngle, nominalKey);
    double fuelToFlyweel = 2.0;
    AngularVelocity flywheelSetpoint =
        RadiansPerSecond.of(fuelToFlyweel * v0_nominal.getNorm() / wheelRadius.in(Meters));
    flywheelIO.setVelocity(flywheelSetpoint);

    // Get translation velocities (m/s) of the turret caused by motion of the chassis
    var robotRelative = chassisSpeedsSupplier.get();
    var fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelative, turretBasePose.toPose2d().getRotation());
    var v_base = getTurretBaseSpeeds(turretBasePose.toPose2d().getRotation(), fieldRelative);

    // Get actual fuel speed
    LinearVelocity fuelSpeed =
        MetersPerSecond.of(
            flywheelInputs.velocityRadPerSec * wheelRadius.in(Meters) / fuelToFlyweel);

    // Replan shot using actual flywheel speed
    var v0_total = getV0(vectorTurretBaseToTarget, fuelSpeed, replannedKey);

    // Point turret to align velocity vectors
    var v0_flywheel = v0_total.minus(v_base);

    // Check if v0_flywheel has non-zero horizontal component
    double v0_horizontal = Math.hypot(v0_flywheel.getX(), v0_flywheel.getY());
    if (v0_horizontal < 1e-6) {
      // Flywheel velocity is too low or target unreachable, stop mechanisms
      return;
    }

    Rotation2d turretSetpoint = new Rotation2d(v0_flywheel.getX(), v0_flywheel.getY());
    turretIO.setPosition(
        turretSetpoint.minus(turretBasePose.toPose2d().getRotation()),
        RadiansPerSecond.of(robotRelative.omegaRadiansPerSecond).unaryMinus().times(2.0));
    Rotation2d hoodSetpoint = new Rotation2d(v0_horizontal, v0_flywheel.getZ());
    hoodIO.setPosition(hoodSetpoint, RadiansPerSecond.of(0));

    // Get actual hood & turret position
    Rotation2d hoodPosition = hoodInputs.position;
    Rotation2d turretPosition = turretInputs.position.plus(turretBasePose.toPose2d().getRotation());

    // Build actual velocities
    Translation3d v0_actual =
        new Translation3d(
                hoodPosition.getCos() * turretPosition.getCos(),
                hoodPosition.getCos() * turretPosition.getSin(),
                hoodPosition.getSin())
            .times(fuelSpeed.in(MetersPerSecond))
            .plus(v_base);
    log(vectorTurretBaseToTarget, v0_actual, actualKey);

    // Spawn simulated fuel
    fuelSpawnTimer += Robot.defaultPeriodSecs;
    if (fuelSpawnTimer >= fuelSpawnPeriod) {
      fuelSpawnTimer = 0.0;

      fuelNominal.add(
          new BallisticObject(
              new Translation3d(
                  turretBasePose.getX(), turretBasePose.getY(), turretBasePose.getZ()),
              new Translation3d(v0_nominal.getX(), v0_nominal.getY(), v0_nominal.getZ())));

      fuelReplanned.add(
          new BallisticObject(
              new Translation3d(
                  turretBasePose.getX(), turretBasePose.getY(), turretBasePose.getZ()),
              new Translation3d(v0_total.getX(), v0_total.getY(), v0_total.getZ())));

      fuelActual.add(
          new BallisticObject(
              new Translation3d(
                  turretBasePose.getX(), turretBasePose.getY(), turretBasePose.getZ()),
              new Translation3d(v0_actual.getX(), v0_actual.getY(), v0_actual.getZ())));
    }
  }

  @AutoLogOutput(key = "Launcher/TurretPose")
  public Pose2d getTurretPose() {
    return turretBasePose.toPose2d().plus(new Transform2d(0, 0, turretInputs.position));
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

    Translation2d r = new Translation2d(chassisToTurretBase.getX(), chassisToTurretBase.getY());
    Translation2d v_offset = new Translation2d(-omega * r.getY(), omega * r.getX());
    Translation2d v_base_field = new Translation2d(vx, vy).plus(v_offset.rotateBy(rotation));

    Translation3d baseSpeeds = new Translation3d(v_base_field.getX(), v_base_field.getY(), 0);

    Logger.recordOutput("Launcher/BaseSpeeds", baseSpeeds);
    return baseSpeeds;
  }

  private Translation3d getV0(Translation3d d, Rotation2d impactAngle, String key) {
    double dr = Math.hypot(d.getX(), d.getY());
    double dz = d.getZ();

    double v_0r = dr * Math.sqrt(g / (2 * (dz + dr * impactAngle.getTan())));
    double v_0z = (g * dr) / v_0r - v_0r * impactAngle.getTan();

    double v_0x = v_0r * d.toTranslation2d().getAngle().getCos();
    double v_0y = v_0r * d.toTranslation2d().getAngle().getSin();

    var v0 = new Translation3d(v_0x, v_0y, v_0z);
    log(d, v0, key);
    return v0;
  }

  private Translation3d getV0(Translation3d d, LinearVelocity shotSpeed, String key) {
    // Geometry
    Translation2d dxy = d.toTranslation2d();
    double dr = dxy.getNorm();
    double dz = d.getZ();

    // Unit vector toward target in XY plane
    Translation2d r_hat = dxy.div(dr);

    double v_flywheel = shotSpeed.in(MetersPerSecond);
    double v_sq = v_flywheel * v_flywheel;

    double discriminant = v_sq * v_sq - g * (g * dr * dr + 2 * dz * v_sq);

    if (discriminant < 0) {
      // Unreachable target at this speed
      Logger.recordOutput("Launcher/" + key + "/Reachable", false);
      return new Translation3d();
    }

    Logger.recordOutput("Launcher/" + key + "/Reachable", true);

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
      return new Translation3d();
    }

    // Scale to match actual flywheel speed
    double scale = shotSpeed.in(MetersPerSecond) / v_required;

    v_r *= scale;
    v_z *= scale;

    // Back to field frame
    Translation2d v_field_xy = r_hat.times(v_r);

    var v0 = new Translation3d(v_field_xy.getX(), v_field_xy.getY(), v_z);
    log(d, v0, key);
    return v0;
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
    Translation3d position;
    Translation3d velocity;

    BallisticObject(Translation3d position, Translation3d velocity) {
      this.position = position;
      this.velocity = velocity;
    }
  }

  private void updateBallisticsSim(ArrayList<BallisticObject> traj, String key) {
    double dt = Robot.defaultPeriodSecs;
    double hubZ = GameState.getMyHubPose().getZ();

    traj.removeIf(
        o -> {
          // Integrate velocity
          o.velocity = o.velocity.plus(new Translation3d(0, 0, -g * dt));

          // Integrate position
          o.position = o.position.plus(o.velocity.times(dt));

          // Remove when below target height and falling
          return o.position.getZ() < hubZ && o.velocity.getZ() < 0;
        });

    Logger.recordOutput("Launcher/" + key + "/FuelTrajectory", getBallTrajectory(traj));
  }

  public Translation3d[] getBallTrajectory(ArrayList<BallisticObject> traj) {
    Translation3d[] t = new Translation3d[traj.size()];
    for (int i = 0; i < traj.size(); i++) {
      t[i] = traj.get(i).position;
    }
    return t;
  }
}
