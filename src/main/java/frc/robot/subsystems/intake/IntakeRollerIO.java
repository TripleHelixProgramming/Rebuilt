package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
  @AutoLog
  public static class IntakeRollerIOInputs {
    public boolean connected = false;
    public double lowerVelocityMetersPerSec = 0.0;
    public double lowerAppliedVolts = 0.0;
    public double lowerCurrentAmps = 0.0;
    public double upperVelocityMetersPerSec = 0.0;
    public double upperAppliedVolts = 0.0;
    public double upperCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeRollerIOInputs inputs) {}

  public default void setOpenLoop(Voltage volts) {}

  public default void setVelocity(LinearVelocity tangentialVelocity) {}
}
