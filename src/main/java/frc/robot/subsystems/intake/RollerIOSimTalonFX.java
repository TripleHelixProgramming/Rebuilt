package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.RollerConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.RollerConstants.TalonConfig.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.MotorConstants.KrakenX60Constants;
import frc.robot.Robot;

public class RollerIOSimTalonFX implements RollerIO {
  private final DCMotorSim rollerSim;

  private final TalonFX motor;
  private final TalonFXConfiguration config;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0).withSlot(1);
  private final NeutralOut brake = new NeutralOut();

  // Inputs from intake motor
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent, torqueCurrent;
  private final StatusSignal<Double> dutyCycle;

  public RollerIOSimTalonFX(RollerConfig rollerConfig) {
    motor = new TalonFX(rollerConfig.port(), rollerConfig.bus());
    config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        rollerConfig.inverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    config.Slot0 = VELOCITY_VOLTAGE_GAINS;
    config.Slot1 = VELOCITY_TORQUE_CURRENT_GAINS;
    config.TorqueCurrent.PeakForwardTorqueCurrent = KrakenX60Constants.DEFAULT_STATOR_CURRENT_LIMIT;
    config.TorqueCurrent.PeakReverseTorqueCurrent =
        -KrakenX60Constants.DEFAULT_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimit = KrakenX60Constants.DEFAULT_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = KrakenX60Constants.DEFAULT_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(GEARBOX, 0.0005, MOTOR_REDUCTION), GEARBOX);

    velocity = motor.getVelocity();
    acceleration = motor.getAcceleration();
    appliedVolts = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    dutyCycle = motor.getDutyCycle();
    torqueCurrent = motor.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, acceleration, appliedVolts, supplyCurrent, dutyCycle, torqueCurrent);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.connected =
        connectedDebounce.calculate(
            BaseStatusSignal.refreshAll(
                    velocity, acceleration, appliedVolts, supplyCurrent, dutyCycle, torqueCurrent)
                .isOK());

    // Update simulation state
    var motorSim = motor.getSimState();
    motorSim.setSupplyVoltage(RoboRioSim.getVInVoltage());
    rollerSim.setInput(motorSim.getMotorVoltage());
    rollerSim.update(Robot.defaultPeriodSecs);
    motorSim.setRawRotorPosition(rollerSim.getAngularPositionRotations() * MOTOR_REDUCTION);
    motorSim.setRotorVelocity(rollerSim.getAngularVelocity().times(MOTOR_REDUCTION));

    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = supplyCurrent.getValueAsDouble();
    inputs.velocityMetersPerSec =
        velocity.getValue().in(RadiansPerSecond) * RADIUS.in(Meters) / MOTOR_REDUCTION;
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
            tangentialVelocity.in(MetersPerSecond) * MOTOR_REDUCTION / RADIUS.in(Meters));
    motor.setControl(velocityTorqueCurrentRequest.withVelocity(angularVelocity));
  }
}
