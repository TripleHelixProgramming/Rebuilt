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
  private final IntakeRollerIO intakeRollerIO;
  private final IntakeArmIO intakeArmIO;

  private final IntakeRollerIOInputsAutoLogged intakeRollerInputs =
      new IntakeRollerIOInputsAutoLogged();
  private final IntakeArmIOInputsAutoLogged intakeArmInputs = new IntakeArmIOInputsAutoLogged();

  private final Alert disconnectedAlert;

  public Intake(IntakeRollerIO intakeRollerIO, IntakeArmIO intakeArmIO) {
    this.intakeRollerIO = intakeRollerIO;
    this.intakeArmIO = intakeArmIO;

    disconnectedAlert = new Alert("Disconnected intake motor", AlertType.kError);
  }

  @Override
  public void periodic() {
    long t0 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;
    intakeRollerIO.updateInputs(intakeRollerInputs);
    intakeArmIO.updateInputs(intakeArmInputs);
    long t1 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;

    Logger.processInputs("IntakeRoller", intakeRollerInputs);
    Logger.processInputs("IntakeArm", intakeArmInputs);
    long t2 = Constants.PROFILING_ENABLED ? System.nanoTime() : 0;

    disconnectedAlert.set(!intakeRollerInputs.connected);

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
    intakeRollerIO.setOpenLoop(Volts.of(0.0));
    intakeArmIO.retract();
  }

  public void runRoller() {
    intakeRollerIO.setVelocity(MetersPerSecond.of(4));
  }

  public void deployArm() {
    intakeArmIO.deploy();
  }

  public Boolean isDeployed() {
    return intakeArmInputs.isDeployed.equals(DoubleSolenoid.Value.kForward);
  }

  public Boolean isStowed() {
    return intakeArmInputs.isDeployed.equals(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public Command getDefaultCommand() {
    return Commands.startEnd(this::stop, () -> {}, this).withName("Retract and stop");
  }

  public Command getDeployCommand() {
    return Commands.sequence(
            Commands.runOnce(this::deployArm, this),
            this.idle().withTimeout(1.0),
            Commands.startEnd(this::runRoller, () -> {}, this))
        .withName("Intake");
  }
}
