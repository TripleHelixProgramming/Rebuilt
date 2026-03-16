package frc.lib;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import org.littletonrobotics.junction.Logger;

public class LoggedCompressor extends Compressor {
  private final String key;

  public LoggedCompressor(PneumaticsModuleType moduleType, String logKey) {
    super(moduleType);
    this.key = logKey;
  }

  public void log() {
    Logger.recordOutput(key + "/Enabled", isEnabled());
    Logger.recordOutput(key + "/PressureSwitch", getPressureSwitchValue());
    Logger.recordOutput(key + "/CurrentAmps", getCurrent());
    Logger.recordOutput(key + "/PressurePSI", getPressure());
  }
}
