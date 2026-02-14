package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.intake.IntakeConstants.IntakeRoller.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANBusPorts.CAN2;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX intakeMotorLeader;
  private final TalonFX intakeMotorFollower;
  private final TalonFXConfiguration config;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  // private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
  //     new VelocityTorqueCurrentFOC(0.0);

  // Inputs from intake motor
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Current> intakeCurrent;

  public IntakeRollerIOTalonFX() {
    intakeMotorLeader = new TalonFX(CAN2.intakeRollerLeader, CAN2.bus);
    intakeMotorFollower = new TalonFX(CAN2.intakeRollerFollower, CAN2.bus);
    config = new TalonFXConfiguration();
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    config.Slot0 = intakeGains;
    tryUntilOk(5, () -> intakeMotorLeader.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> intakeMotorFollower.getConfigurator().apply(config, 0.25));

    intakeVelocity = intakeMotorLeader.getVelocity();
    intakeAppliedVolts = intakeMotorLeader.getMotorVoltage();
    intakeCurrent = intakeMotorLeader.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, intakeVelocity, intakeAppliedVolts, intakeCurrent);

    intakeMotorFollower.setControl(
        new Follower(CAN2.intakeRollerLeader, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.connected =
        connectedDebounce.calculate(
            BaseStatusSignal.refreshAll(intakeVelocity, intakeAppliedVolts, intakeCurrent).isOK());

    inputs.appliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.currentAmps = intakeCurrent.getValueAsDouble();
    inputs.velocityMetersPerSec =
        intakeVelocity.getValue().in(RadiansPerSecond) * rollerRadius.in(Meters) / motorReduction;
  }

  @Override
  public void setOpenLoop(double output) {
    intakeMotorLeader.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setVelocity(LinearVelocity tangentialVelocity) {
    intakeMotorLeader.setControl(
        velocityVoltageRequest.withVelocity(
            RadiansPerSecond.of(
                tangentialVelocity.in(MetersPerSecond)
                    * motorReduction
                    / rollerRadius.in(Meters))));
  }
}
