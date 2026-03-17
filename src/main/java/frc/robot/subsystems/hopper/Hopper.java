package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO hopperIO;
  private final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();

  // Injected after both subsystems are created to avoid a circular dependency.
  // When set, getRetractCommand() will stop the intake first if needed.
  private BooleanSupplier intakeIsStowed;
  private Supplier<Command> intakeStopCommand;

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

  public boolean isStowed() {
    return hopperInputs.isDeployed == DoubleSolenoid.Value.kReverse;
  }

  public Command getDeployCommand() {
    return Commands.startEnd(this::deploy, () -> {}, this).withName("Deploy Hopper");
  }

  /**
   * Configures the intake stow interlock for getRetractCommand(). Must be called after both the
   * Hopper and Intake subsystems are created.
   *
   * @param intakeIsStowed supplier returning true when the intake arm is stowed
   * @param intakeStopCommand factory that returns a fresh command to stop and stow the intake
   */
  public void setRetractInterlock(
      BooleanSupplier intakeIsStowed, Supplier<Command> intakeStopCommand) {
    this.intakeIsStowed = intakeIsStowed;
    this.intakeStopCommand = intakeStopCommand;
  }

  public Command getRetractCommand() {
    if (intakeIsStowed == null) {
      return Commands.startEnd(this::retract, () -> {}, this).withName("Retract Hopper");
    }
    return Commands.sequence(
            Commands.either(Commands.none(), intakeStopCommand.get(), intakeIsStowed),
            Commands.startEnd(this::retract, () -> {}, this))
        .withName("Retract Hopper");
  }
}
