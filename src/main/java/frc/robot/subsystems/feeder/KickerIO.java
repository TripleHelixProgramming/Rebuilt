package frc.robot.subsystems.feeder;

import edu.wpi.first.units.measure.LinearVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public boolean connected = false;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(KickerIOInputs inputs) {}

  public default void setOpenLoop(double output) {}

  public default void setVelocity(LinearVelocity tangentialVelocity) {}
}
