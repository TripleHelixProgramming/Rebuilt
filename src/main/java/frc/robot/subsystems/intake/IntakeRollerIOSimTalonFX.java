package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.IntakeRoller.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

  private final DCMotorSim intakeRollerSim;

  private final TalonFX intakeMotor;
  private final TalonFXConfiguration config;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  // private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
  //     new VelocityTorqueCurrentFOC(0.0);

  // Inputs from intake motor
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Current> intakeCurrent;

  public IntakeRollerIOSimTalonFX() {
    intakeMotor = new TalonFX(CAN2.intakeRoller, CAN2.bus);
    config = new TalonFXConfiguration();
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    config.Slot0 = intakeGains;
    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(config, 0.25));

    var intakeMotorSim = intakeMotor.getSimState();
    intakeMotorSim.Orientation = ChassisReference.Clockwise_Positive;
    intakeMotorSim.setMotorType(MotorType.KrakenX60);

    intakeRollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.0005, motorReduction), gearbox);

    intakeVelocity = intakeMotor.getVelocity();
    intakeAppliedVolts = intakeMotor.getMotorVoltage();
    intakeCurrent = intakeMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, intakeVelocity, intakeAppliedVolts, intakeCurrent);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.connected =
        connectedDebounce.calculate(
            BaseStatusSignal.refreshAll(intakeVelocity, intakeAppliedVolts, intakeCurrent).isOK());

    // Update simulation state
    var intakeMotorSim = intakeMotor.getSimState();
    intakeMotorSim.setSupplyVoltage(RobotConstants.kNominalVoltage);
    intakeRollerSim.setInput(intakeMotorSim.getMotorVoltage());
    intakeRollerSim.update(Robot.defaultPeriodSecs);
    intakeMotorSim.setRawRotorPosition(
        intakeRollerSim.getAngularPositionRotations() * motorReduction);
    intakeMotorSim.setRotorVelocity(intakeRollerSim.getAngularVelocity().times(motorReduction));

    inputs.appliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.currentAmps = intakeCurrent.getValueAsDouble();
    inputs.velocityMetersPerSec =
        intakeVelocity.getValue().in(RadiansPerSecond) * rollerRadius.in(Meters) / motorReduction;
  }

  @Override
  public void setOpenLoop(double output) {
    intakeMotor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setVelocity(LinearVelocity tangentialVelocity) {
    intakeMotor.setControl(
        velocityVoltageRequest.withVelocity(
            RadiansPerSecond.of(
                tangentialVelocity.in(MetersPerSecond)
                    * motorReduction
                    / rollerRadius.in(Meters))));
  }
}
