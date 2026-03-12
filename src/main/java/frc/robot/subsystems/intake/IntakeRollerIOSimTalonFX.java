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
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;

public class IntakeRollerIOSimTalonFX implements IntakeRollerIO {

  // Single sim for both rollers (they receive the same commands)
  private final DCMotorSim intakeRollerSim;

  private final TalonFX intakeMotorLower;
  private final TalonFX intakeMotorUpper;
  private final TalonFXConfiguration config;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0).withSlot(1);
  private final NeutralOut brake = new NeutralOut();

  // Inputs from intake motors
  private final StatusSignal<AngularVelocity> lowerVelocity, upperVelocity;
  private final StatusSignal<Voltage> lowerAppliedVolts, upperAppliedVolts;
  private final StatusSignal<Current> lowerCurrent, upperCurrent;

  public IntakeRollerIOSimTalonFX() {
    intakeMotorLower = new TalonFX(CAN2.intakeRollerLower, CAN2.bus);
    intakeMotorUpper = new TalonFX(CAN2.intakeRollerUpper, CAN2.bus);
    config = new TalonFXConfiguration();
    config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    config.Slot0 = velocityVoltageGains;
    config.Slot1 = velocityTorqueCurrentGains;
    tryUntilOk(5, () -> intakeMotorLower.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> intakeMotorUpper.getConfigurator().apply(config, 0.25));

    // Configure simulation for both motors
    var lowerMotorSim = intakeMotorLower.getSimState();
    lowerMotorSim.Orientation = ChassisReference.Clockwise_Positive;
    lowerMotorSim.setMotorType(MotorType.KrakenX60);

    var upperMotorSim = intakeMotorUpper.getSimState();
    upperMotorSim.Orientation = ChassisReference.Clockwise_Positive;
    upperMotorSim.setMotorType(MotorType.KrakenX60);

    intakeRollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.0005, motorReduction), gearbox);

    lowerVelocity = intakeMotorLower.getVelocity();
    lowerAppliedVolts = intakeMotorLower.getMotorVoltage();
    lowerCurrent = intakeMotorLower.getStatorCurrent();

    upperVelocity = intakeMotorUpper.getVelocity();
    upperAppliedVolts = intakeMotorUpper.getMotorVoltage();
    upperCurrent = intakeMotorUpper.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        lowerVelocity,
        lowerAppliedVolts,
        lowerCurrent,
        upperVelocity,
        upperAppliedVolts,
        upperCurrent);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    // No explicit refresh - Phoenix 6 auto-updates signals at configured frequency (50Hz).
    // This avoids blocking CAN calls in the main loop.
    inputs.connected =
        connectedDebounce.calculate(
            lowerVelocity.getStatus().isOK()
                && lowerAppliedVolts.getStatus().isOK()
                && lowerCurrent.getStatus().isOK()
                && upperVelocity.getStatus().isOK()
                && upperAppliedVolts.getStatus().isOK()
                && upperCurrent.getStatus().isOK());

    // Update simulation state - use lower motor voltage as input, apply to both
    var lowerMotorSim = intakeMotorLower.getSimState();
    var upperMotorSim = intakeMotorUpper.getSimState();

    lowerMotorSim.setSupplyVoltage(RobotConstants.kNominalVoltage);
    upperMotorSim.setSupplyVoltage(RobotConstants.kNominalVoltage);

    intakeRollerSim.setInput(lowerMotorSim.getMotorVoltage());
    intakeRollerSim.update(Robot.defaultPeriodSecs);

    // Apply same simulated state to both motors
    double position = intakeRollerSim.getAngularPositionRotations() * motorReduction;
    var velocity = intakeRollerSim.getAngularVelocity().times(motorReduction);

    lowerMotorSim.setRawRotorPosition(position);
    lowerMotorSim.setRotorVelocity(velocity);
    upperMotorSim.setRawRotorPosition(position);
    upperMotorSim.setRotorVelocity(velocity);

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
    intakeMotorLower.setControl(velocityTorqueCurrentRequest.withVelocity(angularVelocity));
    intakeMotorUpper.setControl(velocityTorqueCurrentRequest.withVelocity(angularVelocity));
  }
}
