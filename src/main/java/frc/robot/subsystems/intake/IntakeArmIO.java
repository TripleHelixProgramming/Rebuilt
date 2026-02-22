package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {
  @AutoLog
  public static class IntakeArmIOInputs {
    public DoubleSolenoid.Value isDeployed = DoubleSolenoid.Value.kReverse;
  }

  public default void updateInputs(IntakeArmIOInputs inputs) {}

  public default void deploy() {}

  public default void retract() {}
}
