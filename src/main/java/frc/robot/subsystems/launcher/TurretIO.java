package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean connected = false;
    public Rotation2d position = Rotation2d.kZero;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    public boolean absoluteEncoderConnected = false;
    public Rotation2d absolutePosition = Rotation2d.kZero;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setOpenLoop(double output) {}

  public default void setPosition(Rotation2d rotation, AngularVelocity angularVelocity) {}

  public default void resetEncoder() {}
}
