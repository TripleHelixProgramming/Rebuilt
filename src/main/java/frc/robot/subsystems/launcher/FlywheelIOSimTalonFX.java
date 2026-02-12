package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.launcher.LauncherConstants.FlywheelConstants.*;
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
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Constants.RobotConstants;
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
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  // private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
  //     new VelocityTorqueCurrentFOC(0.0);

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

    flywheelSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.004, motorReduction), gearbox);

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
    inputs.connected =
        flywheelConnectedDebounce.calculate(
            BaseStatusSignal.refreshAll(flywheelVelocity, flywheelAppliedVolts, flywheelCurrent)
                .isOK());

    // Update simulation state
    var flywheelMotorSim = flywheelLeaderTalon.getSimState();
    flywheelMotorSim.setSupplyVoltage(RobotConstants.kNominalVoltage);
    flywheelSim.setInput(flywheelMotorSim.getMotorVoltage());
    flywheelSim.update(Robot.defaultPeriodSecs);
    flywheelMotorSim.setRawRotorPosition(
        flywheelSim.getAngularPositionRotations() * motorReduction);
    flywheelMotorSim.setRotorVelocity(flywheelSim.getAngularVelocity().times(motorReduction));

    inputs.appliedVolts = flywheelAppliedVolts.getValueAsDouble();
    inputs.currentAmps = flywheelCurrent.getValueAsDouble();
    inputs.velocityRadPerSec = flywheelVelocity.getValue().in(RadiansPerSecond) / motorReduction;
  }

  @Override
  public void setOpenLoop(double output) {
    flywheelLeaderTalon.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setVelocity(AngularVelocity angularVelocity) {
    flywheelLeaderTalon.setControl(velocityVoltageRequest.withVelocity(angularVelocity));
  }
}
