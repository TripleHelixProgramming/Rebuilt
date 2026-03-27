package frc.lib;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LoggedCompressor extends SubsystemBase {
  private final Compressor compressor;
  private final String key;

  public LoggedCompressor(PneumaticsModuleType moduleType, String logKey) {
    compressor = new Compressor(moduleType);
    this.key = logKey;
  }

  public void disable() {
    compressor.disable();
  }

  public void enableDigital() {
    compressor.enableDigital();
  }

  public boolean isEnabled() {
    return compressor.isEnabled();
  }

  public void log() {
    Logger.recordOutput(key + "/Enabled", compressor.isEnabled());
    Logger.recordOutput(key + "/PressureSwitch", compressor.getPressureSwitchValue());
    Logger.recordOutput(key + "/CurrentAmps", compressor.getCurrent());
    Logger.recordOutput(key + "/PressurePSI", compressor.getPressure());
  }
}
