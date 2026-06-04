package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.launcher.LauncherConstants.FlywheelConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Constants.MotorConstants.KrakenX60Constants;
import frc.robot.Robot;

public class FlywheelIOSimTalonFX implements FlywheelIO {
  private static final double FLYWHEEL_MOI_KG_M2 = 0.00026;

  private final DCMotorSim flywheelSim;

  private final TalonFX flywheelLeaderTalon;
  private final TalonFX flywheelFollowerTalon;
  private final TalonFXConfiguration config;
  private final Debouncer flywheelConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  // private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0).withSlot(1);

  // Inputs from flywheel motor
  private final StatusSignal<AngularVelocity> flywheelVelocity;
  private final StatusSignal<Voltage> flywheelAppliedVolts;
  private final StatusSignal<Current> flywheelCurrent;

  public FlywheelIOSimTalonFX() {
    flywheelLeaderTalon = new TalonFX(CAN2.FLYWHEEL_LEADER, CAN2.BUS);
    flywheelFollowerTalon = new TalonFX(CAN2.FLYWHEEL_FOLLOWER, CAN2.BUS);
    // Configuration
    config = new TalonFXConfiguration();
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    config.Slot0 = VELOCITY_VOLTAGE_GAINS;
    config.Slot1 = VELOCITY_TORQUE_CURRENT_GAINS;
    config.TorqueCurrent.PeakForwardTorqueCurrent = KrakenX60Constants.DEFAULT_STATOR_CURRENT_LIMIT;
    config.TorqueCurrent.PeakReverseTorqueCurrent =
        -KrakenX60Constants.DEFAULT_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimit = KrakenX60Constants.DEFAULT_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = KrakenX60Constants.DEFAULT_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> flywheelLeaderTalon.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> flywheelFollowerTalon.getConfigurator().apply(config, 0.25));

    var flywheelMotorSim = flywheelLeaderTalon.getSimState();
    flywheelMotorSim.Orientation = ChassisReference.Clockwise_Positive;
    flywheelMotorSim.setMotorType(MotorType.KrakenX60);

    flywheelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(GEARBOX, FLYWHEEL_MOI_KG_M2, MOTOR_REDUCTION),
            GEARBOX);

    flywheelVelocity = flywheelLeaderTalon.getVelocity();
    flywheelAppliedVolts = flywheelLeaderTalon.getMotorVoltage();
    flywheelCurrent = flywheelLeaderTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, flywheelVelocity, flywheelAppliedVolts, flywheelCurrent);

    flywheelFollowerTalon.setControl(
        new Follower(CAN2.FLYWHEEL_LEADER, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Synchronous refresh for leader/follower motors to ensure consistent state
    inputs.connected =
        flywheelConnectedDebounce.calculate(
            BaseStatusSignal.refreshAll(flywheelVelocity, flywheelAppliedVolts, flywheelCurrent)
                .isOK());

    // Update simulation state
    var flywheelMotorSim = flywheelLeaderTalon.getSimState();
    flywheelMotorSim.setSupplyVoltage(RoboRioSim.getVInVoltage());
    flywheelSim.setInput(flywheelMotorSim.getMotorVoltage());
    flywheelSim.update(Robot.defaultPeriodSecs);
    flywheelMotorSim.setRawRotorPosition(
        flywheelSim.getAngularPositionRotations() * MOTOR_REDUCTION);
    flywheelMotorSim.setRotorVelocity(flywheelSim.getAngularVelocity().times(MOTOR_REDUCTION));

    inputs.appliedVolts = flywheelAppliedVolts.getValueAsDouble();
    inputs.currentAmps = flywheelCurrent.getValueAsDouble();
    inputs.velocityMetersPerSec =
        (flywheelVelocity.getValue().in(RadiansPerSecond) * WHEEL_RADIUS.in(Meters))
            / MOTOR_REDUCTION;
  }

  @Override
  public void setOpenLoop(Voltage volts) {
    flywheelLeaderTalon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setVelocity(LinearVelocity tangentialVelocity) {
    var angularVelocity =
        RadiansPerSecond.of(
            tangentialVelocity.in(MetersPerSecond) * MOTOR_REDUCTION / WHEEL_RADIUS.in(Meters));
    flywheelLeaderTalon.setControl(velocityTorqueCurrentRequest.withVelocity(angularVelocity));
  }
}
