package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean turnConnected = false;
    public Rotation2d turnPosition = Rotation2d.kZero;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setTurnOpenLoop(double output) {}

  public default void setTurnPosition(Rotation2d rotation, double feedforwardVolts) {}
}
