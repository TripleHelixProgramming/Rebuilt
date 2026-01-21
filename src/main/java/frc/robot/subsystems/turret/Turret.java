package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final Supplier<Pose2d> chassisPoseSupplier;

  private final Alert turnDisconnectedAlert;

  public Turret(Supplier<Pose2d> chassisPoseSupplier, TurretIO io) {
    this.io = io;
    this.chassisPoseSupplier = chassisPoseSupplier;
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
    var turretBase = chassisPoseSupplier.get().plus(chassisToTurretBase);
    var turretOrientation =
        FieldConstants.kBlueHubCenter // TODO: Point at the hub of the correct alliance color
            .minus(turretBase.getTranslation())
            .getAngle()
            .minus(turretBase.getRotation());

    io.setTurnPosition(turretOrientation);
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
