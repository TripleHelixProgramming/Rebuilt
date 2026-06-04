package frc.robot.subsystems.hopper;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.Constants.PneumaticChannels.*;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;

public class HopperIOSim implements HopperIO {
  public final DoubleSolenoidSim hopperPneumatic;

  public HopperIOSim() {
    hopperPneumatic = new DoubleSolenoidSim(PneumaticsModuleType.REVPH, HOPPER_FWD, HOPPER_REV);
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
