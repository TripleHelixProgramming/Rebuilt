package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean connected = false;
    public Rotation2d position = Rotation2d.kZero;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setOpenLoop(double output) {}

  public default void setPosition(Rotation2d rotation, double feedforwardVolts) {}
}
