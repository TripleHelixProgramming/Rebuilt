package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final SpindexerIO spindexerIO;
  private final KickerIO kickerIO;

  private final SpindexerIOInputsAutoLogged spindexerInputs = new SpindexerIOInputsAutoLogged();
  private final KickerIOInputsAutoLogged kickerInputs = new KickerIOInputsAutoLogged();

  private final Alert spindexerDisconnectedAlert;
  private final Alert kickerDisconnectedAlert;

  public Feeder(SpindexerIO spindexerIO, KickerIO kickerIO) {
    this.spindexerIO = spindexerIO;
    this.kickerIO = kickerIO;

    spindexerDisconnectedAlert = new Alert("Disconnected spindexer motor", AlertType.kError);
    kickerDisconnectedAlert = new Alert("Disconnected kicker motor", AlertType.kError);
  }

  @Override
  public void periodic() {
    long t0 = System.nanoTime();
    spindexerIO.updateInputs(spindexerInputs);
    long t1 = System.nanoTime();
    kickerIO.updateInputs(kickerInputs);
    long t2 = System.nanoTime();

    Logger.processInputs("Spindexer", spindexerInputs);
    long t3 = System.nanoTime();
    Logger.processInputs("Kicker", kickerInputs);
    long t4 = System.nanoTime();

    spindexerDisconnectedAlert.set(!spindexerInputs.connected);
    kickerDisconnectedAlert.set(!kickerInputs.connected);

    // Profiling output
    long totalMs = (t4 - t0) / 1_000_000;
    if (totalMs > 2) {
      System.out.println(
          "[Feeder] spindexer="
              + (t1 - t0) / 1_000_000
              + "ms kicker="
              + (t2 - t1) / 1_000_000
              + "ms spindexerLog="
              + (t3 - t2) / 1_000_000
              + "ms kickerLog="
              + (t4 - t3) / 1_000_000
              + "ms total="
              + totalMs
              + "ms");
    }
  }

  public void stop() {
    spindexerIO.setOpenLoop(0.0);
    kickerIO.setOpenLoop(0.0);
  }

  public void spinForward() {
    spindexerIO.setVelocity(MetersPerSecond.of(1.0));
    kickerIO.setVelocity(MetersPerSecond.of(1.0));
  }
}
