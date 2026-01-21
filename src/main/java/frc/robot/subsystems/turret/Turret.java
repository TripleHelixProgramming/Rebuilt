package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final Alert turnDisconnectedAlert;

  public Turret(TurretIO io) {
    this.io = io;
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

  public void setOrientation(Supplier<Pose2d> chassisPoseSupplier) {
    var turretPose = chassisPoseSupplier.get().plus(chassisToTurret);
    var turretOrientation =
        FieldConstants.kBlueHubCenter // TODO: Point at the hub of the correct alliance color
            .minus(turretPose.getTranslation())
            .getAngle()
            .minus(turretPose.getRotation());

    io.setTurnPosition(turretOrientation);
  }
}
