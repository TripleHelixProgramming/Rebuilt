package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.DriveConstants.kCANBus;
import static frc.robot.subsystems.intake.IntakeConstants.IntakeRoller.*;
import static frc.robot.subsystems.launcher.LauncherConstants.TurretConstants.motorReduction;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class IntakeRollerIOSim implements IntakeRollerIO {

  private final DCMotorSim intakeRollerSim;

  private final TalonFX intakeMotor;
  private final TalonFXConfiguration config;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Inputs from intake motor
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Current> intakeCurrent;

  public IntakeRollerIOSim() {
    intakeMotor = new TalonFX(port, kCANBus);
    config = new TalonFXConfiguration();
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    config.Slot0 = intakeGains;
    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(config, 0.25));

    var intakeMotorSim = intakeMotor.getSimState();
    intakeMotorSim.Orientation = ChassisReference.Clockwise_Positive;
    intakeMotorSim.setMotorType(MotorType.KrakenX60);

    intakeRollerSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.004, motorReduction), gearbox);

    intakeVelocity = intakeMotor.getVelocity();
    intakeAppliedVolts = intakeMotor.getMotorVoltage();
    intakeCurrent = intakeMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, intakeVelocity, intakeAppliedVolts, intakeCurrent);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
        // Update simulation state
        var intakeMotorSim = intakeMotor.getSimState();
    intakeRollerSim.setInput(intakeMotorSim.getMotorVoltageMeasure().in(Volts));
    intakeRollerSim.update(Robot.defaultPeriodSecs);
    intakeMotorSim.setRawRotorPosition(intakeRollerSim.getAngularPosition().times(motorReudction));
   intakeMotorSim.setRotorVelocity(intakeRollerSim.getAngularVelocity().times(motorReudction));

    BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, intakeVelocity, intakeAppliedVolts, intakeCurrent)
        .isOK();

    inputs.appliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.connected = intakeMotor.isConnected();
    inputs.currentAmps = intakeCurrent.getValueAsDouble();
    inputs.velocityMetersPerSec =
        intakeVelocity.getValueAsDouble() * rollerRadius.in(Meters) * (2 * Math.PI);
  }

  @Override
  public void setOpenLoop(double output) {
    intakeMotor.setControl(voltageRequest.withOutput(output));
  }

  public void setVelocity(AngularVelocity angularVelocity) {
    intakeMotor.setControl(velocityTorqueCurrentRequest.withVelocity(angularVelocity));
  }
}
