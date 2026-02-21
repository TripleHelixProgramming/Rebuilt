package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
  @AutoLog
  public static class IntakeRollerIOInputs {
    public boolean connected = false;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(IntakeRollerIOInputs inputs) {}

  public default void setOpenLoop(Voltage volts) {}

  public default void setVelocity(LinearVelocity tangentialVelocity) {}
}
