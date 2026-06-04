package frc.robot.subsystems.intake;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.Constants.PneumaticChannels.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class IntakeArmIOReal implements IntakeArmIO {
  public final DoubleSolenoid intakeArmPneumatic;

  public IntakeArmIOReal() {
    intakeArmPneumatic =
        new DoubleSolenoid(PneumaticsModuleType.REVPH, INTAKE_ARM_FWD, INTAKE_ARM_REV);
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
