package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.ArmConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.ArmConstants.motorReduction;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Constants.MotorConstants.NEOConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;

public class IntakeArmIOSim implements IntakeArmIO {

  private final DCMotorSim armSim;

  private final SparkMax maxRight;
  private final SparkMax maxLeft;
  private final SparkClosedLoopController controller;
  private final SparkMaxSim maxSim;

  private final SparkMaxConfig armConfig;
  private final SparkMaxConfig followerConfig;

  public IntakeArmIOSim() {
    maxRight = new SparkMax(CAN2.intakeArmRight, MotorType.kBrushless);
    maxLeft = new SparkMax(CAN2.intakeArmLeft, MotorType.kBrushless);

    controller = maxRight.getClosedLoopController();

    armConfig = new SparkMaxConfig();

    armConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(NEOConstants.kDefaultSupplyCurrentLimit)
        .voltageCompensation(RobotConstants.kNominalVoltage);

    armConfig
        .encoder
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor);

    armConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kPSim, 0.0, kDSim);

    armConfig
        .softLimit
        .forwardSoftLimit(maxPosRad)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(minPosRad)
        .reverseSoftLimitEnabled(true);

    followerConfig = new SparkMaxConfig();

    followerConfig.apply(armConfig).follow(CAN2.intakeArmRight);

    maxRight.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    maxSim = new SparkMaxSim(maxRight, gearbox);

    armSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.004, motorReduction), gearbox);

    armSim.setState(0.0, 0.0);
    maxSim.setPosition(0.0);

    maxLeft.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeArmIOInputs inputs) {
    // Update simulation state
    double busVoltage = RoboRioSim.getVInVoltage();
    armSim.setInput(maxSim.getAppliedOutput() * busVoltage);
    armSim.update(Robot.defaultPeriodSecs);

    if (maxSim.getPosition() > maxPosRad) {
      armSim.setState(maxPosRad, 0.0);
      maxSim.setPosition(maxPosRad);
    }

    maxSim.iterate(armSim.getAngularVelocityRadPerSec(), busVoltage, Robot.defaultPeriodSecs);

    // Update inputs
    inputs.connected = true;
    inputs.position = maxSim.getPosition();
    inputs.velocityMetersPerSec = maxSim.getVelocity();
    inputs.appliedVolts = maxSim.getAppliedOutput() * maxSim.getBusVoltage();
    inputs.currentAmps = Math.abs(maxSim.getMotorCurrent());
  }

  @Override
  public void setOpenLoop(Voltage volts) {
    maxSim.setAppliedOutput(volts.in(Volts) / RobotConstants.kNominalVoltage);
  }

  @Override
  public void setPosition(Angle rotation) {
    controller.setSetpoint(rotation.magnitude(), ControlType.kPosition);
  }

  @Override
  public void configureSoftLimits(boolean enable) {
    armConfig.softLimit.forwardSoftLimitEnabled(enable);
    armConfig.softLimit.reverseSoftLimitEnabled(enable);
  }

  @Override
  public void resetEncoder() {
    maxSim.setPosition(maxPosRad);
  }
}
