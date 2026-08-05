package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {
  @AutoLog
  public static class IntakeArmIOInputs {
    public boolean connected = false;
    public double position = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    public Rotation2d absolutePosition = Rotation2d.kZero;
  }

  public default void updateInputs(IntakeArmIOInputs inputs) {}

  public default void setPosition(Angle rotation, AngularVelocity velocity) {}

  public default void resetEncoder(Angle position) {}
}
