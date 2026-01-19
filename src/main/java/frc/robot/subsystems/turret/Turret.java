package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class Turret {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final Alert turnDisconnectedAlert;

  public Turret(TurretIO io) {
    this.io = io;
    turnDisconnectedAlert = new Alert("Disconnected turret turn motor", AlertType.kError);
  }
}
