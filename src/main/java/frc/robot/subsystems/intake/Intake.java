package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final RollerIO upperRollerIO;
  private final RollerIO lowerRollerIO;
  private final IntakeArmIO intakeArmIO;

  private final RollerIOInputsAutoLogged upperRollerInputs = new RollerIOInputsAutoLogged();
  private final RollerIOInputsAutoLogged lowerRollerInputs = new RollerIOInputsAutoLogged();
  private final IntakeArmIOInputsAutoLogged intakeArmInputs = new IntakeArmIOInputsAutoLogged();

  private final Alert upperRollerDisconnectedAlert;
  private final Alert lowerRollerDisconnectedAlert;

  // Injected after both subsystems are created to avoid a circular dependency.
  // When set, getDeployCommand() and getReverseCommand() will deploy the hopper first if needed.
  private BooleanSupplier hopperIsDeployed;
  private Supplier<Command> hopperDeployCommand;

  public Intake(RollerIO upperRollerIO, RollerIO lowerRollerIO, IntakeArmIO intakeArmIO) {
    this.upperRollerIO = upperRollerIO;
    this.lowerRollerIO = lowerRollerIO;
    this.intakeArmIO = intakeArmIO;

    upperRollerDisconnectedAlert = new Alert("Disconnected upper intake roller", AlertType.kError);
    lowerRollerDisconnectedAlert = new Alert("Disconnected lower intake roller", AlertType.kError);
  }

  @Override
  public void periodic() {
    long t0 = Constants.FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;
    upperRollerIO.updateInputs(upperRollerInputs);
    lowerRollerIO.updateInputs(lowerRollerInputs);
    intakeArmIO.updateInputs(intakeArmInputs);
    long t1 = Constants.FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    Logger.processInputs("UpperRoller", upperRollerInputs);
    Logger.processInputs("LowerRoller", lowerRollerInputs);
    Logger.processInputs("IntakeArm", intakeArmInputs);
    long t2 = Constants.FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    upperRollerDisconnectedAlert.set(!upperRollerInputs.connected);
    lowerRollerDisconnectedAlert.set(!lowerRollerInputs.connected);
    Logger.recordOutput("Faults/Intake/UpperRollerDisconnected", !upperRollerInputs.connected);
    Logger.recordOutput("Faults/Intake/LowerRollerDisconnected", !lowerRollerInputs.connected);

    // Profiling output
    if (Constants.FeatureFlags.PROFILING_ENABLED) {
      long totalMs = (t2 - t0) / 1_000_000;
      if (totalMs > 2) {
        System.out.println(
            "[Intake] update="
                + (t1 - t0) / 1_000_000
                + "ms log="
                + (t2 - t1) / 1_000_000
                + "ms total="
                + totalMs
                + "ms");
      }
    }
  }

  public void stop() {
    upperRollerIO.setOpenLoop(Volts.of(0.0));
    lowerRollerIO.setOpenLoop(Volts.of(0.0));
    intakeArmIO.retract();
  }

  public void deployArm() {
    intakeArmIO.deploy();
  }

  public void retractArm() {
    intakeArmIO.retract();
  }

  public Boolean isDeployed() {
    return intakeArmInputs.isDeployed == DoubleSolenoid.Value.kForward;
  }

  public boolean isStowed() {
    return intakeArmInputs.isDeployed == DoubleSolenoid.Value.kReverse;
  }

  /**
   * Configures the hopper deploy interlock for getDeployCommand() and getReverseCommand(). Must be
   * called after both the Intake and Hopper subsystems are created.
   *
   * @param hopperIsDeployed supplier returning true when the hopper is deployed
   * @param hopperDeployCommand factory that returns a fresh command to deploy the hopper
   */
  public void setDeployInterlock(
      BooleanSupplier hopperIsDeployed, Supplier<Command> hopperDeployCommand) {
    this.hopperIsDeployed = hopperIsDeployed;
    this.hopperDeployCommand = hopperDeployCommand;
  }

  public Command getStopCommand() {
    return Commands.startEnd(this::stop, () -> {}, this).withName("Retract and stop");
  }

  @Override
  public Command getDefaultCommand() {
    return getStopCommand();
  }

  public Command getDeployCommand() {
    return Commands.sequence(
            hopperInterlock(),
            Commands.runOnce(this::deployArm, this),
            this.idle().withTimeout(0.5),
            Commands.startEnd(
                () -> {
                  upperRollerIO.setVelocity(MetersPerSecond.of(6.0));
                  lowerRollerIO.setVelocity(MetersPerSecond.of(6.0));
                },
                () -> {},
                this))
        .withName("Intake");
  }

  /** Returns the total motor current draw for battery simulation. */
  public double getSimCurrentDrawAmps() {
    return upperRollerInputs.currentAmps + lowerRollerInputs.currentAmps;
  }

  public Command getReverseCommand() {
    return Commands.sequence(
            hopperInterlock(),
            Commands.runOnce(this::deployArm, this),
            this.idle().withTimeout(0.5),
            Commands.startEnd(
                () -> {
                  upperRollerIO.setVelocity(MetersPerSecond.of(-4.0));
                  lowerRollerIO.setVelocity(MetersPerSecond.of(-4.0));
                },
                () -> {},
                this))
        .withName("Reverse");
  }

  /** Returns the hopper deploy interlock step, or a no-op if no interlock has been configured. */
  private Command hopperInterlock() {
    if (hopperIsDeployed == null) {
      return Commands.none();
    }
    return Commands.either(Commands.none(), hopperDeployCommand.get(), hopperIsDeployed);
  }

  public Command getShakeIntakeCommand() {
    return Commands.sequence(
            this.idle().withTimeout(1.0),
            Commands.runOnce(this::stop, this),
            this.idle().withTimeout(1.0),
            Commands.runOnce(this::deployArm, this))
        .repeatedly();
  }
}
