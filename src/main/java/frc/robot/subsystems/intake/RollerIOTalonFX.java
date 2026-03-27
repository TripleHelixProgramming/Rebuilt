package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.RollerConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.MotorConstants.KrakenX60Constants;
import frc.robot.subsystems.intake.IntakeConstants.RollerConfig;

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX motor;
  private final TalonFXConfiguration config;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0).withSlot(1);
  private final NeutralOut brake = new NeutralOut();

  // private final TrapezoidProfile profile =
  //     new TrapezoidProfile(new TrapezoidProfile.Constraints(maxAcceleration, maxJerk));
  // Inputs from intake motor
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;

  public RollerIOTalonFX(RollerConfig rollerConfig) {
    motor = new TalonFX(rollerConfig.port, rollerConfig.bus);
    config = new TalonFXConfiguration();
    config.TorqueCurrent.PeakForwardTorqueCurrent = kStatorCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = KrakenX60Constants.kDefaultSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        rollerConfig.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    config.Slot0 = velocityVoltageGains;
    config.Slot1 = velocityTorqueCurrentGains;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25)); // -1 tryUntilOkay

    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, supplyCurrent);

    // Disable all signals not explicitly configured above to reduce CAN bus load
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.connected =
        connectedDebounce.calculate(
            BaseStatusSignal.refreshAll(velocity, appliedVolts, supplyCurrent).isOK());

    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = supplyCurrent.getValueAsDouble();
    inputs.velocityMetersPerSec =
        velocity.getValue().in(RadiansPerSecond) * rollerRadius.in(Meters) / motorReduction;
  }

  @Override
  public void setOpenLoop(Voltage volts) {
    if (volts.in(Volts) < 1e-6) {
      motor.setControl(brake);
    } else {
      motor.setControl(voltageRequest.withOutput(volts));
    }
  }

  @Override
  public void setVelocity(LinearVelocity tangentialVelocity) {
    AngularVelocity angularVelocity =
        RadiansPerSecond.of(
            tangentialVelocity.in(MetersPerSecond) * motorReduction / rollerRadius.in(Meters));

    // TrapezoidProfile.State goal =
    //     new TrapezoidProfile.State(angularVelocity.in(RotationsPerSecond), 0);
    // TrapezoidProfile.State setpoint =
    //     new TrapezoidProfile.State(
    //         intakeVelocity.getValueAsDouble(), intakeAcceleration.getValueAsDouble());

    // setpoint = profile.calculate(Robot.defaultPeriodSecs, setpoint, goal);

    // velocityTorqueCurrentRequest.Velocity = setpoint.position;
    // velocityTorqueCurrentRequest.Acceleration = setpoint.velocity;
    motor.setControl(velocityTorqueCurrentRequest.withVelocity(angularVelocity));
  }
}
