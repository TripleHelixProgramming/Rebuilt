package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
    spindexerIO.updateInputs(spindexerInputs);
    kickerIO.updateInputs(kickerInputs);

    Logger.processInputs("Spindexer", spindexerInputs);
    Logger.processInputs("Kicker", kickerInputs);

    spindexerDisconnectedAlert.set(!spindexerInputs.connected);
    kickerDisconnectedAlert.set(!kickerInputs.connected);
  }

  public void stop() {
    spindexerIO.setOpenLoop(Volts.of(0.0));
    kickerIO.setOpenLoop(Volts.of(0.0));
  }

  public void spinForward() {
    spindexerIO.setVelocity(MetersPerSecond.of(1.0));
    kickerIO.setVelocity(MetersPerSecond.of(1.0));
  }
}
