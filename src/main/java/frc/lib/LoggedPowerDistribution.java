package frc.lib;

import edu.wpi.first.wpilibj.PowerDistribution;
import org.littletonrobotics.junction.Logger;

public class LoggedPowerDistribution extends PowerDistribution {
  private final String key;

  public LoggedPowerDistribution(int module, ModuleType moduleType, String logKey) {
    super(module, moduleType);
    this.key = logKey;
  }

  public void log() {
    Logger.recordOutput(key + "/Voltage", getVoltage());
    Logger.recordOutput(key + "/TotalCurrentAmps", getTotalCurrent());
    Logger.recordOutput(key + "/TotalPowerWatts", getTotalPower());
    Logger.recordOutput(key + "/TotalEnergyJoules", getTotalEnergy());
    Logger.recordOutput(key + "/TemperatureCelsius", getTemperature());
    Logger.recordOutput(key + "/SwitchableChannelActive", getSwitchableChannel());
    int n = getNumChannels();
    double[] currents = new double[n];
    for (int i = 0; i < n; i++) currents[i] = getCurrent(i);
    Logger.recordOutput(key + "/ChannelCurrentsAmps", currents);
  }
}
