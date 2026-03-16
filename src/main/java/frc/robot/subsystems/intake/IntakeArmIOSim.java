package frc.robot.subsystems.intake;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.Constants.PneumaticChannels.*;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;

public class IntakeArmIOSim implements IntakeArmIO {
  public final DoubleSolenoidSim intakeArmPneumatic;

  public IntakeArmIOSim() {
    intakeArmPneumatic =
        new DoubleSolenoidSim(PneumaticsModuleType.REVPH, intakeArmForward, intakeArmReverse);
    intakeArmPneumatic.set(kReverse);
  }

  @Override
  public void updateInputs(IntakeArmIOInputs inputs) {
    inputs.isDeployed = intakeArmPneumatic.get();
  }

  @Override
  public void deploy() {
    intakeArmPneumatic.set(kForward);
  }

  @Override
  public void retract() {
    intakeArmPneumatic.set(kReverse);
  }
}
