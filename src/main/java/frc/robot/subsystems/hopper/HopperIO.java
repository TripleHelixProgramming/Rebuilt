package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public DoubleSolenoid.Value isDeployed = DoubleSolenoid.Value.kReverse;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void deploy() {}

  public default void retract() {}
}
