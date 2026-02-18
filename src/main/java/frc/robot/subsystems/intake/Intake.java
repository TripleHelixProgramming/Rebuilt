package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeRollerIO intakeRollerIO;
  private final IntakeArmIO intakeArmIO;
  private final HopperIO hopperIO;

  private final IntakeRollerIOInputsAutoLogged intakeRollerInputs =
      new IntakeRollerIOInputsAutoLogged();
  private final IntakeArmIOInputsAutoLogged intakeArmInputs = new IntakeArmIOInputsAutoLogged();
  private final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();

  private final Alert disconnectedAlert;

  public Intake(IntakeRollerIO intakeRollerIO, IntakeArmIO intakeArmIO, HopperIO hopperIO) {
    this.intakeRollerIO = intakeRollerIO;
    this.intakeArmIO = intakeArmIO;
    this.hopperIO = hopperIO;

    disconnectedAlert = new Alert("Disconnected intake motor", AlertType.kError);
  }

  @Override
  public void periodic() {
    intakeRollerIO.updateInputs(intakeRollerInputs);
    intakeArmIO.updateInputs(intakeArmInputs);
    hopperIO.updateInputs(hopperInputs);

    Logger.processInputs("IntakeRoller", intakeRollerInputs);
    Logger.processInputs("IntakeArm", intakeArmInputs);
    Logger.processInputs("Hopper", hopperInputs);

    disconnectedAlert.set(!intakeRollerInputs.connected);
  }

  public void stop() {
    intakeRollerIO.setOpenLoop(0.0);
    intakeArmIO.retract();
    hopperIO.retract();
  }

  public void intakeFuel() {
    intakeRollerIO.setVelocity(MetersPerSecond.of(6));
    hopperIO.deploy();
    intakeArmIO.deploy();
  }
}
