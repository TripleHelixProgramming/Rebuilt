package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.launcher.LauncherConstants.HoodConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Constants.MotorConstants.NEO550Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;

public class HoodIOSimSpark implements HoodIO {

  private final DCMotorSim hoodSim;

  private final SparkMax max;
  private final SparkClosedLoopController controller;
  private final SparkMaxSim maxSim;

  private final SparkMaxConfig hoodConfig;

  public HoodIOSimSpark() {
    max = new SparkMax(CAN2.hood, MotorType.kBrushless);
    controller = max.getClosedLoopController();

    hoodConfig = new SparkMaxConfig();

    hoodConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(NEO550Constants.kDefaultSupplyCurrentLimit)
        .voltageCompensation(RobotConstants.kNominalVoltage);

    hoodConfig
        .encoder
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor);

    hoodConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kPSimPos, 0.0, kDSimPos);

    hoodConfig
        .softLimit
        .forwardSoftLimit(maxPosRad)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(minPosRad)
        .reverseSoftLimitEnabled(true);

    hoodConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20)
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    max.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    maxSim = new SparkMaxSim(max, gearbox);

    hoodSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.004, motorReduction), gearbox);

    hoodSim.setState(minPosRad, 0);
    maxSim.setPosition(minPosRad);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Update simulation state
    hoodSim.setInput(maxSim.getAppliedOutput() * RobotConstants.kNominalVoltage);
    hoodSim.update(Robot.defaultPeriodSecs);

    if (maxSim.getPosition() > maxPosRad) {
      hoodSim.setState(maxPosRad, 0);
      maxSim.setPosition(maxPosRad);
    }

    maxSim.iterate(
        hoodSim.getAngularVelocityRadPerSec(),
        RobotConstants.kNominalVoltage,
        Robot.defaultPeriodSecs);

    // Update inputs
    inputs.connected = true;
    inputs.position = new Rotation2d(maxSim.getPosition());
    inputs.velocityRadPerSec = maxSim.getVelocity();
    inputs.appliedVolts = maxSim.getAppliedOutput() * maxSim.getBusVoltage();
    inputs.currentAmps = Math.abs(maxSim.getMotorCurrent());
  }

  @Override
  public void setOpenLoop(Voltage volts) {
    maxSim.setAppliedOutput(volts.in(Volts) / 12.0);
  }

  @Override
  public void setPosition(Rotation2d rotation, AngularVelocity angularVelocity) {
    double setpoint = MathUtil.clamp(rotation.getRadians(), minPosRad, maxPosRad);
    double feedforwardVolts =
        RobotConstants.kNominalVoltage
            * angularVelocity.in(RadiansPerSecond)
            / maxAngularVelocity.in(RadiansPerSecond);
    controller.setSetpoint(
        setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforwardVolts);
  }

  @Override
  public void setVelocity(AngularVelocity angularVelocity) {
    double setpoint = angularVelocity.in(RadiansPerSecond);
    double feedforwardVolts =
        RobotConstants.kNominalVoltage
            * angularVelocity.in(RadiansPerSecond)
            / maxAngularVelocity.in(RadiansPerSecond);
    controller.setSetpoint(
        setpoint, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforwardVolts);
  }

  @Override
  public void configureSoftLimits(boolean enable) {
    hoodConfig.softLimit.forwardSoftLimitEnabled(enable);
    hoodConfig.softLimit.reverseSoftLimitEnabled(enable);
  }

  @Override
  public void resetEncoder() {
    maxSim.setPosition(maxPosRad);
  }
}
