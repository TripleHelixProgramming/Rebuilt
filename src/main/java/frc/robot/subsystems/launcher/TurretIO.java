package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean motorControllerConnected = false;
    public Rotation2d relativePosition = Rotation2d.kZero;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    public boolean absoluteEncoderConnected = false;
    public Rotation2d absolutePosition = Rotation2d.kZero;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setOpenLoop(Voltage volts) {}

  public default void setPosition(Rotation2d rotation, AngularVelocity angularVelocity) {}
}
