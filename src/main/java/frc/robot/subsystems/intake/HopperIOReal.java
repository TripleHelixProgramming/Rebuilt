package frc.robot.subsystems.intake;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.subsystems.intake.IntakeConstants.PneumaticConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class HopperIOReal implements HopperIO {
  public final DoubleSolenoid hopperPneumatic;

  public HopperIOReal() {
    hopperPneumatic =
        new DoubleSolenoid(PneumaticsModuleType.REVPH, hopperForwardChannel, hopperReverseChannel);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.isDeployed = hopperPneumatic.get();
  }

  @Override
  public void deploy() {
    hopperPneumatic.set(kForward);
  }

  @Override
  public void retract() {
    hopperPneumatic.set(kReverse);
  }
}
