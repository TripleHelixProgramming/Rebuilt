package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface IntakeIO {
    @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = false;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setOpenLoop(double output) {}

  public default void setVelocity(AngularVelocity angularVelocity) {}
}
