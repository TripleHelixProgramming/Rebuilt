package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.launcher.LauncherConstants.TurretConstants.*;

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

public class TurretIOSimSpark implements TurretIO {

  private final DCMotorSim turnSim;

  private final SparkMax turnSpark;
  private final SparkClosedLoopController controller;
  private final SparkMaxSim turnSparkSim;

  public TurretIOSimSpark() {
    turnSpark = new SparkMax(CAN2.turret, MotorType.kBrushless);
    controller = turnSpark.getClosedLoopController();

    var turnConfig = new SparkMaxConfig();

    turnConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(NEO550Constants.kDefaultSupplyCurrentLimit)
        .voltageCompensation(RobotConstants.kNominalVoltage);

    turnConfig
        .encoder
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor);

    turnConfig
        .softLimit
        .forwardSoftLimit(Math.PI + rangeOfMotion.div(2).in(Radians))
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(Math.PI - rangeOfMotion.div(2).in(Radians))
        .reverseSoftLimitEnabled(true);

    turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kPSim, 0.0, kDSim);

    turnConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnSparkSim = new SparkMaxSim(turnSpark, gearbox);

    turnSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.004, motorReduction), gearbox);

    turnSim.setState(2.0 * Math.PI - mechanismOffset.getRadians(), 0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Update simulation state
    turnSim.setInput(turnSparkSim.getAppliedOutput() * RobotConstants.kNominalVoltage);
    turnSim.update(Robot.defaultPeriodSecs);
    turnSparkSim.iterate(
        turnSim.getAngularVelocityRadPerSec(),
        RobotConstants.kNominalVoltage,
        Robot.defaultPeriodSecs);

    // Update inputs
    inputs.motorControllerConnected = true;
    inputs.relativePosition = new Rotation2d(turnSparkSim.getPosition()).plus(mechanismOffset);
    inputs.velocityRadPerSec = turnSparkSim.getVelocity();
    inputs.appliedVolts = turnSparkSim.getAppliedOutput() * turnSparkSim.getBusVoltage();
    inputs.currentAmps = Math.abs(turnSparkSim.getMotorCurrent());

    inputs.absoluteEncoderConnected = true;
    inputs.absolutePosition = new Rotation2d(turnSparkSim.getPosition()).plus(mechanismOffset);
  }

  @Override
  public void setOpenLoop(Voltage volts) {
    controller.setSetpoint(volts.in(Volts), ControlType.kVoltage);
  }

  @Override
  public void setPosition(Rotation2d rotation, AngularVelocity angularVelocity) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.getRadians() - mechanismOffset.getRadians(), 0.0, 2.0 * Math.PI);
    setpoint =
        MathUtil.clamp(
            setpoint,
            Math.PI - rangeOfMotion.div(2).in(Radians),
            Math.PI + rangeOfMotion.div(2).in(Radians));
    var feedforwardVolts =
        RobotConstants.kNominalVoltage
            * angularVelocity.in(RadiansPerSecond)
            / maxAngularVelocity.in(RadiansPerSecond);
    controller.setSetpoint(
        setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforwardVolts);
  }
}
