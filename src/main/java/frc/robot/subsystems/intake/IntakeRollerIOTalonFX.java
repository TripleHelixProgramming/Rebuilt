package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.IntakeRoller.*;
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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANBusPorts.CAN2;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX intakeMotorLower;
  private final TalonFX intakeMotorUpper;
  private final TalonFXConfiguration config;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final VoltageOut voltageRequest = new VoltageOut(0);
  // private final VelocityVoltage velocityVoltageRequest =
  //     new VelocityVoltage(0.0).withSlot(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0).withSlot(1);
  private final NeutralOut brake = new NeutralOut();

  private final TrapezoidProfile profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(maxAcceleration, maxJerk));

  // Inputs from intake motor
  private final StatusSignal<AngularVelocity> lowerVelocity, upperVelocity;
  private final StatusSignal<AngularAcceleration> lowerAcceleration, upperAcceleration;
  private final StatusSignal<Voltage> lowerAppliedVolts, upperAppliedVolts;
  private final StatusSignal<Current> lowerCurrent, upperCurrent;
  private final StatusSignal<Double> lowerDutyCycle, upperDutyCycle;
  private final StatusSignal<Current> lowerTorqueCurrent, upperTorqueCurrent;

  public IntakeRollerIOTalonFX() {
    intakeMotorLower = new TalonFX(CAN2.intakeRollerLower, CAN2.bus);
    intakeMotorUpper = new TalonFX(CAN2.intakeRollerUpper, CAN2.bus);
    config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    config.Slot0 = velocityVoltageGains;
    config.Slot1 = velocityTorqueCurrentGains;
    tryUntilOk(5, () -> intakeMotorLower.getConfigurator().apply(config, 0.25)); // -1 tryUntilOkay
    tryUntilOk(5, () -> intakeMotorUpper.getConfigurator().apply(config, 0.25)); // -1 tryUntilOkay

    lowerVelocity = intakeMotorLower.getVelocity();
    lowerAcceleration = intakeMotorLower.getAcceleration();
    lowerAppliedVolts = intakeMotorLower.getMotorVoltage();
    lowerCurrent = intakeMotorLower.getSupplyCurrent();
    lowerDutyCycle = intakeMotorLower.getDutyCycle();
    lowerTorqueCurrent = intakeMotorLower.getTorqueCurrent();

    upperVelocity = intakeMotorUpper.getVelocity();
    upperAcceleration = intakeMotorUpper.getAcceleration();
    upperAppliedVolts = intakeMotorUpper.getMotorVoltage();
    upperCurrent = intakeMotorUpper.getSupplyCurrent();
    upperDutyCycle = intakeMotorUpper.getDutyCycle();
    upperTorqueCurrent = intakeMotorUpper.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        lowerVelocity,
        lowerAcceleration,
        lowerAppliedVolts,
        lowerCurrent,
        lowerDutyCycle,
        lowerTorqueCurrent,
        upperAcceleration,
        upperVelocity,
        upperAppliedVolts,
        upperCurrent,
        upperDutyCycle,
        upperTorqueCurrent);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.connected =
        connectedDebounce.calculate(
            BaseStatusSignal.refreshAll(
                    lowerVelocity,
                    lowerAcceleration,
                    lowerAppliedVolts,
                    lowerCurrent,
                    lowerDutyCycle,
                    lowerTorqueCurrent,
                    upperAcceleration,
                    upperVelocity,
                    upperAppliedVolts,
                    upperCurrent,
                    upperDutyCycle,
                    upperTorqueCurrent)
                .isOK());

    inputs.lowerAppliedVolts = lowerAppliedVolts.getValueAsDouble();
    inputs.lowerCurrentAmps = lowerCurrent.getValueAsDouble();
    inputs.lowerVelocityMetersPerSec =
        lowerVelocity.getValue().in(RadiansPerSecond) * rollerRadius.in(Meters) / motorReduction;
    inputs.upperAppliedVolts = upperAppliedVolts.getValueAsDouble();
    inputs.upperCurrentAmps = upperCurrent.getValueAsDouble();
    inputs.upperVelocityMetersPerSec =
        upperVelocity.getValue().in(RadiansPerSecond) * rollerRadius.in(Meters) / motorReduction;
  }

  @Override
  public void setOpenLoop(Voltage volts) {
    if (volts.in(Volts) < 1e-6) {
      intakeMotorLower.setControl(brake);
      intakeMotorUpper.setControl(brake);
    } else {
      intakeMotorLower.setControl(voltageRequest.withOutput(volts));
      intakeMotorUpper.setControl(voltageRequest.withOutput(volts));
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
    intakeMotorLower.setControl(velocityTorqueCurrentRequest.withVelocity(angularVelocity));
    intakeMotorUpper.setControl(velocityTorqueCurrentRequest.withVelocity(angularVelocity));
  }
}
