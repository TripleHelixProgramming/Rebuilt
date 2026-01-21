package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final Supplier<Pose2d> chassisPoseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  private final Alert turnDisconnectedAlert;

  private Rotation2d turretOrientationSetpoint = Rotation2d.kZero;
  private Translation2d dynamicTargetToTurretBase = new Translation2d();
  private Pose2d target = new Pose2d();

  public Turret(
      Supplier<Pose2d> chassisPoseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      TurretIO io) {
    this.io = io;
    this.chassisPoseSupplier = chassisPoseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    turnDisconnectedAlert = new Alert("Disconnected turret turn motor", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    turnDisconnectedAlert.set(!inputs.turnConnected);
  }

  public void stop() {
    io.setTurnOpenLoop(0.0);
  }

  public void aimAtHub() {
    // Get vector from static target to turret
    var staticTarget =
        FieldConstants.kBlueHubCenter; // TODO: Point at the hub of the correct alliance color
    var turretBase = chassisPoseSupplier.get().plus(chassisToTurretBase);
    var staticTargetToTurretBase = turretBase.getTranslation().minus(staticTarget);

    // Calculate travel time of free projectile at constant horizontal velocity
    var radialDistance = Meters.of(staticTargetToTurretBase.getNorm());
    var fuelTravelTime = radialDistance.div(fuelVelocityRadial);

    // Get turret velocity (m/s) relative to hub
    var chassisSpeeds = chassisSpeedsSupplier.get();
    Translation2d turretBaseSpeeds = getTurretBaseSpeeds(chassisSpeeds);

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
            * -(2 * chassisSpeeds.omegaRadiansPerSecond // TODO: Why is this factor 2 needed?
                + apparentAngularVelocityRadPerSec)
            / turnMaxAngularVelocity.in(RadiansPerSecond);

    io.setTurnPosition(turretOrientationSetpoint, feedforwardVolts);
  }

  @AutoLogOutput(key = "Turret/Pose")
  public Pose2d getTurretPose() {
    return chassisPoseSupplier
        .get()
        .plus(chassisToTurretBase)
        .plus(new Transform2d(0, 0, inputs.turnPosition));
  }

  @AutoLogOutput(key = "Turret/Target")
  public Pose2d getTargetPose() {
    return target;
  }

  @AutoLogOutput(key = "Turret/IsOnTarget")
  public boolean isOnTarget() {
    return inputs.turnPosition.minus(turretOrientationSetpoint).getMeasure().abs(Radians)
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
