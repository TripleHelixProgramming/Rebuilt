package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public Value isDeployed = Value.kReverse;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void deploy() {}

  public default void retract() {}
}
