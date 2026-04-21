package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {
  @AutoLog
  public static class IntakeArmIOInputs {
    public boolean connected = false;
    public double position = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(IntakeArmIOInputs inputs) {}

  public default void setOpenLoop(Voltage volts) {}

  public default void setPosition(Angle rotation, AngularVelocity velocity) {}

  public default void setVelocity(AngularVelocity velocity) {}

  public default void configureSoftLimits(boolean enable) {}

  public default void resetEncoder() {}
}
