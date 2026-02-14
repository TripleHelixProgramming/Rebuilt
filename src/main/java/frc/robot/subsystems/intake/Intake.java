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
    long t0 = System.nanoTime();
    io.updateInputs(inputs);
    long t1 = System.nanoTime();

    Logger.processInputs("IntakeRoller", inputs);
    long t2 = System.nanoTime();

    disconnectedAlert.set(!inputs.connected);

    // Profiling output
    long totalMs = (t2 - t0) / 1_000_000;
    if (totalMs > 2) {
      System.out.println(
          "[Intake] update="
              + (t1 - t0) / 1_000_000
              + "ms log="
              + (t2 - t1) / 1_000_000
              + "ms total="
              + totalMs
              + "ms");
    }
  }

  public void stop() {
    io.setOpenLoop(0.0);
  }

  public void intakeFuel() {
    io.setVelocity(MetersPerSecond.of(1.0));
  }
}
