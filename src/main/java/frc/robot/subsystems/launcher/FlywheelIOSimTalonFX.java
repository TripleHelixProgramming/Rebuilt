package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Robot;

public class FlywheelIOSimTalonFX implements FlywheelIO {
  private final DCMotorSim flywheelSim;

  private final TalonFX flywheelLeaderTalon;
  private final TalonFX flywheelFollowerTalon;
  private final TalonFXConfiguration config;
  private final Debouncer flywheelConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  // private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  //   private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Inputs from flywheel motor
  private final StatusSignal<AngularVelocity> flywheelVelocity;
  private final StatusSignal<Voltage> flywheelAppliedVolts;
  private final StatusSignal<Current> flywheelCurrent;

  public FlywheelIOSimTalonFX() {
    flywheelLeaderTalon = new TalonFX(CAN2.flywheelLeader, CAN2.bus);
    flywheelFollowerTalon = new TalonFX(CAN2.flywheelFollower, CAN2.bus);
    // Configuration
    config = new TalonFXConfiguration();
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    config.Slot0 = flywheelGains;
    tryUntilOk(5, () -> flywheelLeaderTalon.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> flywheelFollowerTalon.getConfigurator().apply(config, 0.25));

    var flywheelMotorSim = flywheelLeaderTalon.getSimState();
    flywheelMotorSim.Orientation = ChassisReference.Clockwise_Positive;
    flywheelMotorSim.setMotorType(MotorType.KrakenX60);

    flywheelSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.004, motorReduction), gearbox);

    flywheelVelocity = flywheelLeaderTalon.getVelocity();
    flywheelAppliedVolts = flywheelLeaderTalon.getMotorVoltage();
    flywheelCurrent = flywheelLeaderTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelVelocity,
        flywheelAppliedVolts,
        flywheelCurrent,
        flywheelVelocity,
        flywheelAppliedVolts,
        flywheelCurrent);

    flywheelFollowerTalon.setControl(
        new Follower(CAN2.flywheelLeader, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Update simulation state
    var flywheelMotorSim = flywheelLeaderTalon.getSimState();
    flywheelMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    flywheelSim.setInput(flywheelMotorSim.getMotorVoltageMeasure().in(Volts));
    flywheelSim.update(Robot.defaultPeriodSecs);
    flywheelMotorSim.setRawRotorPosition(
        flywheelSim.getAngularPositionRotations() * motorReduction);
    flywheelMotorSim.setRotorVelocity(flywheelSim.getAngularVelocityRPM() * motorReduction);

    BaseStatusSignal.refreshAll(
            flywheelVelocity,
            flywheelAppliedVolts,
            flywheelCurrent,
            flywheelVelocity,
            flywheelAppliedVolts,
            flywheelCurrent)
        .isOK();

    inputs.appliedVolts = flywheelAppliedVolts.getValueAsDouble();
    inputs.connected =
        flywheelConnectedDebounce.calculate(
            flywheelAppliedVolts.getStatus().isOK()
                && flywheelCurrent.getStatus().isOK()
                && flywheelVelocity.getStatus().isOK());
    inputs.currentAmps = flywheelCurrent.getValueAsDouble();
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(flywheelVelocity.getValueAsDouble()) / gearboxRatio;
  }

  @Override
  public void setOpenLoop(double output) {
    // flywheelLeaderTalon.setControl(voltageRequest.withOutput(output));
    flywheelSim.setInputVoltage(output);
  }

  @Override
  public void setVelocity(AngularVelocity angularVelocity) {
    // flywheelLeaderTalon.setControl(velocityTorqueCurrentRequest.withVelocity(angularVelocity));
    flywheelSim.setState(0.0, angularVelocity.in(RadiansPerSecond));
  }
}
