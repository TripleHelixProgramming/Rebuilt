package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.RollerConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeConstants.RollerConfig;

public class RollerIOSimTalonFX implements RollerIO {
  private static final double kP = 0.11;
  private static final double kD = 0.0;
  private static final Slot0Configs velocityVoltageGains =
      new Slot0Configs().withKP(kP).withKI(0.0).withKD(kD).withKS(0.1).withKV(0.12);
  private static final Slot1Configs velocityTorqueCurrentGains =
      new Slot1Configs().withKP(kP).withKI(0.0).withKD(kD).withKS(2.5);
  private static final DCMotor gearbox = DCMotor.getKrakenX60(numMotors);

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
    motor = new TalonFX(rollerConfig.port, rollerConfig.bus);
    config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        rollerConfig.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    config.Slot0 = velocityVoltageGains;
    config.Slot1 = velocityTorqueCurrentGains;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.0005, motorReduction), gearbox);

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
    motorSim.setRawRotorPosition(rollerSim.getAngularPositionRotations() * motorReduction);
    motorSim.setRotorVelocity(rollerSim.getAngularVelocity().times(motorReduction));

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
    motor.setControl(velocityTorqueCurrentRequest.withVelocity(angularVelocity));
  }
}
