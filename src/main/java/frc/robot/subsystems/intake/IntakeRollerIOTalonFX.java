// "Changed" = It has some code from FlywheelIOTalonFX but is not exactly the same.
// "New" = It has been added and is not referenced in FlywheelIOTalonFX.
// Not including flywheel -> intake changes. Also not including chnaged/removed comments.
// Everything in the original code. Only thing changed is the added comments and spaces.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.IntakeRoller.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANBusPorts.CAN2;
import org.littletonrobotics.junction.Logger;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX intakeMotorLeader;
  private final TalonFX intakeMotorFollower;
  private final TalonFXConfiguration config;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest =
      new VelocityVoltage(0.0).withSlot(0); // Unused in here, but not in FlywheelIOTalonFX
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0).withSlot(1); // New
  private final NeutralOut brake = new NeutralOut(); // New

  // Inputs from intake motor
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> intakeAppliedVolts, followerAppliedVolts; // Changed
  private final StatusSignal<Current> intakeCurrent, followerCurrent; // Changed

  public IntakeRollerIOTalonFX() {
    intakeMotorLeader = new TalonFX(CAN2.intakeRollerLeader, CAN2.bus);
    intakeMotorFollower = new TalonFX(CAN2.intakeRollerFollower, CAN2.bus);
    config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake); // Changed
    config.Slot0 = velocityVoltageGains; // New
    config.Slot1 = velocityTorqueCurrentGains; // Changed to 1. Used to be Slot0
    tryUntilOk(5, () -> intakeMotorLeader.getConfigurator().apply(config, 0.25)); // -1 tryUntilOkay

    intakeVelocity = intakeMotorLeader.getVelocity();
    intakeAppliedVolts = intakeMotorLeader.getMotorVoltage();
    intakeCurrent = intakeMotorLeader.getSupplyCurrent(); // New
    followerAppliedVolts = intakeMotorFollower.getMotorVoltage(); // New
    followerCurrent = intakeMotorFollower.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakeVelocity,
        intakeAppliedVolts,
        intakeCurrent,
        // -1 intakeVelocity. Flywheel has 2. One goes here
        followerAppliedVolts,
        followerCurrent);

    intakeMotorFollower.setControl(
        new Follower(CAN2.intakeRollerLeader, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.connected =
        connectedDebounce.calculate(
            BaseStatusSignal.refreshAll(intakeVelocity, intakeAppliedVolts, intakeCurrent)
                .isOK()); // Chnaged

    inputs.appliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.currentAmps = intakeCurrent.getValueAsDouble();
    inputs.velocityMetersPerSec =
        intakeVelocity.getValue().in(RadiansPerSecond) * rollerRadius.in(Meters) / motorReduction;

    BaseStatusSignal.refreshAll(followerCurrent, followerAppliedVolts); // New
    Logger.recordOutput("Intake/Follower/Current", followerCurrent.getValue()); // New
    Logger.recordOutput("Intake/Follower/Volts", followerAppliedVolts.getValue()); // New
  }

  @Override
  public void setOpenLoop(double output) {
    if (output < 1e-6) {
      intakeMotorLeader.setControl(brake);
    } else {
      intakeMotorLeader.setControl(voltageRequest.withOutput(output)); // New and Changed
    }
  }

  @Override
  public void setVelocity(LinearVelocity tangentialVelocity) {
    var angularVelocity =
        RadiansPerSecond.of(
            tangentialVelocity.in(MetersPerSecond)
                * motorReduction
                / rollerRadius.in(Meters)); // New
    intakeMotorLeader.setControl(
        velocityTorqueCurrentRequest.withVelocity(angularVelocity)); // Changed
  }
}
