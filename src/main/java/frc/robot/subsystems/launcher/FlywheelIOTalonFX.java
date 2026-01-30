package frc.robot.subsystems.launcher;

import static frc.robot.subsystems.drive.DriveConstants.kCANBus;
import static frc.robot.subsystems.launcher.LauncherConstants.FlywheelConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.PhoenixOdometryThread;
import java.util.Queue;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX flywheelTalon;
  private final TalonFXConfiguration config;
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Timestamp inputs from Phoenix thread
  private final Queue<Double> timestampQueue;

  // Inputs from drive motor
  private final StatusSignal<Angle> flywheelPosition;
  private final Queue<Double> flywheelPositionQueue;
  private final StatusSignal<AngularVelocity> flywheelVelocity;
  private final StatusSignal<Voltage> flywheelAppliedVolts;
  private final StatusSignal<Current> flywheelCurrent;

  public FlywheelIOTalonFX() {
    flywheelTalon = new TalonFX(port, kCANBus);
    // Configuration
    config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = flywheelGains;
    tryUntilOk(5, () -> flywheelTalon.getConfigurator().apply(config, 0.25));

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    flywheelPosition = flywheelTalon.getPosition();
    flywheelPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(flywheelPosition.clone());
    flywheelVelocity = flywheelTalon.getVelocity();
    flywheelAppliedVolts = flywheelTalon.getMotorVoltage();
    flywheelCurrent = flywheelTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelVelocity,
        flywheelAppliedVolts,
        flywheelCurrent,
        flywheelVelocity,
        flywheelAppliedVolts,
        flywheelCurrent);
  }
}
