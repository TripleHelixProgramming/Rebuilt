package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public void setOrientation(Supplier<Rotation2d> orientationSupplier) {
    io.setTurnPosition(orientationSupplier.get());
  }
}
