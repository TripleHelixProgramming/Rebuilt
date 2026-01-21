package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
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
    var hub = FieldConstants.kBlueHubCenter; // TODO: Point at the hub of the correct alliance color
    var turretBase = chassisPoseSupplier.get().plus(chassisToTurretBase);
    var hubToTurretBase = turretBase.getTranslation().minus(hub);

    var turretOrientation = hubToTurretBase.unaryMinus().getAngle().minus(turretBase.getRotation());

    var chassisSpeeds = chassisSpeedsSupplier.get();
    Translation2d speeds =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    var tangentialSpeed =
        speeds.cross(hubToTurretBase) / hubToTurretBase.getNorm() / hubToTurretBase.getNorm();

    double feedforwardVolts =
        RobotConstants.kNominalVoltage
            * (-2 * chassisSpeeds.omegaRadiansPerSecond - tangentialSpeed) // TODO: Why is this factor 2 needed?
            / turnMaxAngularVelocity.in(RadiansPerSecond);

    io.setTurnPosition(turretOrientation, feedforwardVolts);
  }

  @AutoLogOutput(key = "Turret/Pose")
  public Pose2d getTurretPose() {
    return chassisPoseSupplier
        .get()
        .plus(chassisToTurretBase)
        .plus(new Transform2d(0, 0, inputs.turnPosition));
  }

  @AutoLogOutput(key = "Turret/IsOnTarget")
  public boolean isOnTarget() {
    return true; // TODO: Return whether the turret position is close to the setpoint
  }
}
