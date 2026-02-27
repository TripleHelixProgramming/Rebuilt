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
    long t0 = System.nanoTime();
    hopperIO.updateInputs(hopperInputs);
    long t1 = System.nanoTime();
    Logger.processInputs("Hopper", hopperInputs);
    long t2 = System.nanoTime();

    // Profiling output
    long totalMs = (t2 - t0) / 1_000_000;
    if (totalMs > 2) {
      System.out.println(
          "[Hopper] update="
              + (t1 - t0) / 1_000_000
              + "ms log="
              + (t2 - t1) / 1_000_000
              + "ms total="
              + totalMs
              + "ms");
    }
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
