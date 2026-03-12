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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Robot;

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

  private final TrapezoidProfile profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(maxAcceleration, maxJerk));

  // Inputs from flywheel motor
  private final StatusSignal<AngularVelocity> flywheelVelocity;
  private final StatusSignal<AngularAcceleration> flywheelAcceleration;
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
    config.TorqueCurrent.PeakForwardTorqueCurrent = maxCurrentAmps;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -maxCurrentAmps;
    config.CurrentLimits.StatorCurrentLimit = maxCurrentAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> flywheelLeaderTalon.getConfigurator().apply(config, 0.25));

    flywheelVelocity = flywheelLeaderTalon.getVelocity();
    flywheelAcceleration = flywheelLeaderTalon.getAcceleration();
    flywheelAppliedVolts = flywheelLeaderTalon.getMotorVoltage();
    flywheelCurrent = flywheelLeaderTalon.getSupplyCurrent();

    followerAppliedVolts = flywheelFollowerTalon.getMotorVoltage();
    followerCurrent = flywheelFollowerTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelVelocity,
        flywheelAcceleration,
        flywheelAppliedVolts,
        flywheelCurrent,
        followerAppliedVolts,
        followerCurrent);

    // Note: Do not use optimizeBusUtilizationForAll() here - leader/follower
    // configurations require certain status signals for proper follower behavior

    flywheelFollowerTalon.setControl(
        new Follower(CAN2.flywheelLeader, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Synchronous refresh for leader/follower motors to ensure consistent state
    inputs.connected =
        connectedDebounce.calculate(
            BaseStatusSignal.refreshAll(
                    flywheelVelocity,
                    flywheelAcceleration,
                    flywheelAppliedVolts,
                    flywheelCurrent,
                    followerAppliedVolts,
                    followerCurrent)
                .isOK());

    inputs.appliedVolts = flywheelAppliedVolts.getValueAsDouble();
    inputs.currentAmps = flywheelCurrent.getValueAsDouble();
    inputs.velocityMetersPerSec =
        (flywheelVelocity.getValue().in(RadiansPerSecond) * wheelRadius.in(Meters))
            / motorReduction;

    // Populate follower telemetry via inputs struct (AdvantageKit best practice:
    // IO layers should be pure - only populate inputs, logging happens via @AutoLog)
    inputs.followerAppliedVolts = followerAppliedVolts.getValueAsDouble();
    inputs.followerCurrentAmps = followerCurrent.getValueAsDouble();
  }

  @Override
  public void setOpenLoop(Voltage volts) {
    if (volts.in(Volts) < 1e-6) {
      flywheelLeaderTalon.setControl(brake);
    } else {
      flywheelLeaderTalon.setControl(voltageRequest.withOutput(volts));
    }
  }

  @Override
  public void setVelocity(LinearVelocity tangentialVelocity) {
    AngularVelocity angularVelocity =
        RadiansPerSecond.of(
            tangentialVelocity.in(MetersPerSecond) * motorReduction / wheelRadius.in(Meters));

    TrapezoidProfile.State goal =
        new TrapezoidProfile.State(angularVelocity.in(RotationsPerSecond), 0);
    TrapezoidProfile.State setpoint =
        new TrapezoidProfile.State(
            flywheelVelocity.getValueAsDouble(), flywheelAcceleration.getValueAsDouble());

    setpoint = profile.calculate(Robot.defaultPeriodSecs, setpoint, goal);

    velocityTorqueCurrentRequest.Velocity = setpoint.position;
    velocityTorqueCurrentRequest.Acceleration = setpoint.velocity;
    flywheelLeaderTalon.setControl(velocityTorqueCurrentRequest);
  }
}
