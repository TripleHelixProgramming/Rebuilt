package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeRollerIO io;

  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  private final Alert disconnectedAlert;

  public Intake(IntakeRollerIO io) {
    this.io = io;

    disconnectedAlert = new Alert("Disconnected intake motor", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("IntakeRoller", inputs);

    disconnectedAlert.set(!inputs.connected);
  }

  public void stop() {
    io.setOpenLoop(0.0);
  }

  public void intakeFuel() {
    io.setVelocity(MetersPerSecond.of(1.0));
  }
}
