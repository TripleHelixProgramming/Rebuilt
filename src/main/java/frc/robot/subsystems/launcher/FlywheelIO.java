package frc.robot.subsystems.launcher;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean connected = false;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setOpenLoop(Voltage volts) {}

  public default void setVelocity(LinearVelocity tangentialVelocity) {}
}
