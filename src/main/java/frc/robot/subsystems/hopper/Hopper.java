package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO hopperIO;
  private final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();

  public Hopper(HopperIO hopperIO) {
    this.hopperIO = hopperIO;
  }

  @Override
  public void periodic() {
    hopperIO.updateInputs(hopperInputs);
    Logger.processInputs("Hopper", hopperInputs);
  }

  public void retract() {
    hopperIO.retract();
  }

  public void deploy() {
    hopperIO.deploy();
  }

  public Boolean isDeployed() {
    return hopperInputs.isDeployed.equals(DoubleSolenoid.Value.kForward);
  }

  @Override
  public Command getDefaultCommand() {
    return Commands.startEnd(this::retract, () -> {}, this).withName("Retract");
  }
}
