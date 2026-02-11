package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.intake.IntakeConstants.IntakeRoller.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANBusPorts.CAN2;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX intakeMotor;
  private final TalonFXConfiguration config;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Inputs from intake motor
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Current> intakeCurrent;

  public IntakeRollerIOTalonFX() {
    intakeMotor = new TalonFX(CAN2.intakeRoller, CAN2.bus);
    config = new TalonFXConfiguration();
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    config.Slot0 = intakeGains;
    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(config, 0.25));

    intakeVelocity = intakeMotor.getVelocity();
    intakeAppliedVolts = intakeMotor.getMotorVoltage();
    intakeCurrent = intakeMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, intakeVelocity, intakeAppliedVolts, intakeCurrent);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(intakeVelocity, intakeAppliedVolts, intakeCurrent);
    if (!status.isOK()) {
      return;
    }
    inputs.appliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.connected = intakeMotor.isConnected();
    inputs.currentAmps = intakeCurrent.getValueAsDouble();
    inputs.velocityMetersPerSec =
        intakeVelocity.getValueAsDouble() * rollerRadius.in(Meters) * (2 * Math.PI);
  }

  @Override
  public void setOpenLoop(double output) {
    intakeMotor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setVelocity(LinearVelocity tangentialVelocity) {
    var angularVelocity =
        RadiansPerSecond.of(tangentialVelocity.in(MetersPerSecond) / rollerRadius.in(Meters));
    intakeMotor.setControl(velocityTorqueCurrentRequest.withVelocity(angularVelocity));
  }
}
