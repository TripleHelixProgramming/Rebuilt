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

  public Intake(RollerIO upperRollerIO, RollerIO lowerRollerIO, IntakeArmIO intakeArmIO) {
    this.upperRollerIO = upperRollerIO;
    this.lowerRollerIO = lowerRollerIO;
    this.intakeArmIO = intakeArmIO;

    upperRollerDisconnectedAlert = new Alert("Disconnected upper intake roller", AlertType.kError);
    lowerRollerDisconnectedAlert = new Alert("Disconnected lower intake roller", AlertType.kError);
  }

  @Override
  public void periodic() {
    long t0 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;
    upperRollerIO.updateInputs(upperRollerInputs);
    lowerRollerIO.updateInputs(lowerRollerInputs);
    intakeArmIO.updateInputs(intakeArmInputs);
    long t1 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;

    Logger.processInputs("UpperRoller", upperRollerInputs);
    Logger.processInputs("LowerRoller", lowerRollerInputs);
    Logger.processInputs("IntakeArm", intakeArmInputs);
    long t2 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;

    upperRollerDisconnectedAlert.set(!upperRollerInputs.connected);
    lowerRollerDisconnectedAlert.set(!lowerRollerInputs.connected);

    // Profiling output
    if (Constants.PROFILING_ENABLED) {
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

  public boolean isDeployed() {
    return intakeArmInputs.isDeployed == DoubleSolenoid.Value.kForward;
  }

  public boolean isStowed() {
    return intakeArmInputs.isDeployed == DoubleSolenoid.Value.kReverse;
  }

  @Override
  public Command getDefaultCommand() {
    return Commands.startEnd(this::stop, () -> {}, this).withName("Retract and stop");
  }

  public Command getDeployCommand() {
    return Commands.sequence(
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
}
