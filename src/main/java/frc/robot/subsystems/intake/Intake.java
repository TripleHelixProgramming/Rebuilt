package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    long t0 = System.nanoTime();
    intakeRollerIO.updateInputs(intakeRollerInputs);
    intakeArmIO.updateInputs(intakeArmInputs);
    long t1 = System.nanoTime();

    Logger.processInputs("IntakeRoller", intakeRollerInputs);
    Logger.processInputs("IntakeArm", intakeArmInputs);
    long t2 = System.nanoTime();

    disconnectedAlert.set(!intakeRollerInputs.connected);

    // Profiling output
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

  public void stop() {
    intakeRollerIO.setOpenLoop(Volts.of(0.0));
    intakeArmIO.retract();
  }

  public void intakeFuel() {
    intakeRollerIO.setVelocity(MetersPerSecond.of(6));
    intakeArmIO.deploy();
  }

  public void reverse() {
    intakeRollerIO.setVelocity(MetersPerSecond.of(-6));
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
}
