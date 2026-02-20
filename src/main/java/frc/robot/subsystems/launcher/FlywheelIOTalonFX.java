package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.launcher.LauncherConstants.FlywheelConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX flywheelLeaderTalon;
  private final TalonFX flywheelFollowerTalon;
  private final TalonFXConfiguration config;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  // private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0).withSlot(1);
  private final NeutralOut brake = new NeutralOut();

  // Inputs from flywheel motor
  private final StatusSignal<AngularVelocity> flywheelVelocity;
  private final StatusSignal<Voltage> flywheelAppliedVolts, followerAppliedVolts;
  private final StatusSignal<Current> flywheelCurrent, followerCurrent;

  public FlywheelIOTalonFX() {
    flywheelLeaderTalon = new TalonFX(CAN2.flywheelLeader, CAN2.bus);
    flywheelFollowerTalon = new TalonFX(CAN2.flywheelFollower, CAN2.bus);
    // Configuration
    config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
        .withNeutralMode(NeutralModeValue.Brake);
    config.Slot0 = velocityVoltageGains;
    config.Slot1 = velocityTorqueCurrentGains;
    tryUntilOk(5, () -> flywheelLeaderTalon.getConfigurator().apply(config, 0.25));

    flywheelVelocity = flywheelLeaderTalon.getVelocity();
    flywheelAppliedVolts = flywheelLeaderTalon.getMotorVoltage();
    followerCurrent = flywheelLeaderTalon.getSupplyCurrent();
    flywheelCurrent = flywheelLeaderTalon.getSupplyCurrent();
    followerAppliedVolts = flywheelFollowerTalon.getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelVelocity,
        flywheelAppliedVolts,
        flywheelCurrent,
        flywheelAppliedVolts,
        flywheelCurrent);

    flywheelFollowerTalon.setControl(
        new Follower(CAN2.flywheelLeader, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.connected =
        connectedDebounce.calculate(
            BaseStatusSignal.refreshAll(flywheelVelocity, flywheelAppliedVolts, flywheelCurrent)
                .isOK());

    inputs.appliedVolts = flywheelAppliedVolts.getValueAsDouble();
    inputs.currentAmps = flywheelCurrent.getValueAsDouble();
    inputs.velocityRadPerSec = flywheelVelocity.getValue().in(RadiansPerSecond) / motorReduction;

    BaseStatusSignal.refreshAll(followerCurrent, followerAppliedVolts);
    Logger.recordOutput("Flywheel/Follower/Current", followerCurrent.getValue());
    Logger.recordOutput("Flywheel/Follower/Volts", followerAppliedVolts.getValue());
  }

  @Override
  public void setOpenLoop(double output) {
    if (output < 1e-6) {
      flywheelLeaderTalon.setControl(brake);
    } else {
      flywheelLeaderTalon.setControl(voltageRequest.withOutput(output));
    }
  }

  @Override
  public void setVelocity(LinearVelocity tangentialVelocity) {
    var angularVelocity =
        RadiansPerSecond.of(
            tangentialVelocity.in(MetersPerSecond) * motorReduction / wheelRadius.in(Meters));
    flywheelLeaderTalon.setControl(velocityTorqueCurrentRequest.withVelocity(angularVelocity));
  }
}
