package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO hopperIO;
  private final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();

  public Hopper(HopperIO hopperIO) {
    this.hopperIO = hopperIO;
  }

  @Override
  public void periodic() {
    long t0 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;
    hopperIO.updateInputs(hopperInputs);
    long t1 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;
    Logger.processInputs("Hopper", hopperInputs);
    long t2 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Profiling output
    if (Constants.PROFILING_ENABLED) {
      Logger.recordOutput("Profiling/Hopper/UpdateMs", (t1 - t0) / 1_000_000);
      Logger.recordOutput("Profiling/Hopper/LogMs", (t2 - t1) / 1_000_000);
      Logger.recordOutput("Profiling/Hopper/TotalMs", (t2 - t0) / 1_000_000);
    }
  }

  public void retract() {
    hopperIO.retract();
  }

  public void deploy() {
    hopperIO.deploy();
  }

  public boolean isDeployed() {
    return hopperInputs.isDeployed == DoubleSolenoid.Value.kForward;
  }

  @Override
  public Command getDefaultCommand() {
    return Commands.startEnd(this::retract, () -> {}, this).withName("Retract");
  }
}
