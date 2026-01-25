package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.launcher.LauncherConstants.TurretConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.game.GameState;
import frc.robot.Constants.RobotConstants;
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

  private Rotation2d turretOrientationSetpoint = Rotation2d.kZero;
  private Translation2d dynamicTargetToTurretBase = new Translation2d();
  private Pose2d target = new Pose2d();

  public Launcher(
      Supplier<Pose2d> chassisPoseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      TurretIO turretIO, FlywheelIO flywheelIO, HoodIO hoodIO) {
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
    Logger.processInputs("Flyweel", flywheelInputs);
    Logger.processInputs("Hood", hoodInputs);

    turretDisconnectedAlert.set(!turretInputs.connected);
    flywheelDisconnectedAlert.set(!flywheelInputs.connected);
    hoodDisconnectedAlert.set(!hoodInputs.connected);
  }

  public void stop() {
    turretIO.setOpenLoop(0.0);
    flywheelIO.setOpenLoop(0.0);
    hoodIO.setOpenLoop(0.0);
  }

  public void aimAtHub() {
    // Get vector from static target to turret
    var staticTarget = GameState.getMyHubPose();
    var turretBase = chassisPoseSupplier.get().plus(chassisToTurretBase);
    var staticTargetToTurretBase = turretBase.getTranslation().minus(staticTarget);

    // Calculate travel time of free projectile at constant horizontal velocity
    var radialDistance = Meters.of(staticTargetToTurretBase.getNorm());
    var fuelTravelTime = radialDistance.div(fuelVelocityRadial);

    // Get turret velocity (m/s) relative to hub
    var robotRelative = chassisSpeedsSupplier.get();
    var fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, turretBase.getRotation());
    Translation2d turretBaseSpeeds = getTurretBaseSpeeds(fieldRelative);

    // Create unit vectors
    var radialUnit = staticTargetToTurretBase.div(staticTargetToTurretBase.getNorm());
    var tangentialUnit = radialUnit.rotateBy(Rotation2d.kCCW_90deg);

    // Compute dot products
    var radialMagnitude = turretBaseSpeeds.dot(radialUnit);
    var tangentialMagnitude = turretBaseSpeeds.dot(tangentialUnit);

    // Create radial and tangential components of turret velocity (m/s) relative to hub
    var radialVelocity = radialUnit.times(radialMagnitude);
    var tangentialVelocity = tangentialUnit.times(tangentialMagnitude);

    // Calculate distance to lag the target
    var lag = tangentialVelocity.times(fuelTravelTime.in(Seconds));
    var dynamicTarget = staticTarget.minus(lag);
    target = new Pose2d(dynamicTarget, Rotation2d.kZero);

    // Get vector from dynamic target to turret
    dynamicTargetToTurretBase = turretBase.getTranslation().minus(dynamicTarget);

    // Get angular velocity omega, where V = omega x R
    var apparentAngularVelocityRadPerSec =
        turretBaseSpeeds.cross(dynamicTargetToTurretBase)
            / dynamicTargetToTurretBase.getSquaredNorm();

    turretOrientationSetpoint =
        dynamicTargetToTurretBase.unaryMinus().getAngle().minus(turretBase.getRotation());

    double feedforwardVolts =
        RobotConstants.kNominalVoltage
            * -(2 * robotRelative.omegaRadiansPerSecond // TODO: Why is this factor 2 needed?
                + apparentAngularVelocityRadPerSec)
            / turnMaxAngularVelocity.in(RadiansPerSecond);

    turretIO.setPosition(turretOrientationSetpoint, feedforwardVolts);
  }

  @AutoLogOutput(key = "Turret/Pose")
  public Pose2d getTurretPose() {
    return chassisPoseSupplier
        .get()
        .plus(chassisToTurretBase)
        .plus(new Transform2d(0, 0, turretInputs.position));
  }

  @AutoLogOutput(key = "Turret/Target")
  public Pose2d getTargetPose() {
    return target;
  }

  @AutoLogOutput(key = "Turret/IsOnTarget")
  public boolean isOnTarget() {
    return turretInputs.position.minus(turretOrientationSetpoint).getMeasure().abs(Radians)
            * dynamicTargetToTurretBase.getNorm()
        < (hubWidth.in(Meters) / 2.0);
  }

  private Translation2d getTurretBaseSpeeds(ChassisSpeeds chassisSpeeds) {
    // Extract translation from the transform
    Translation2d r = chassisToTurretBase.getTranslation();

    double vx = chassisSpeeds.vxMetersPerSecond;
    double vy = chassisSpeeds.vyMetersPerSecond;
    double omega = chassisSpeeds.omegaRadiansPerSecond;

    // Rigid-body velocity equation
    return new Translation2d(vx - omega * r.getY(), vy + omega * r.getX());
  }
}
