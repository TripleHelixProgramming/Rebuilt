package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.launcher.LauncherConstants.TurretConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Constants.MotorConstants.NEO550Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;

public class TurretIOSim implements TurretIO {

  private final DCMotorSim turnSim;

  private final SparkMax turnSpark;
  private final SparkMaxSim turnSparkSim;
  private final PIDController controller;

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;
  private double feedforwardVolts = 0.0;

  public TurretIOSim() {
    turnSpark = new SparkMax(CAN2.turret, MotorType.kBrushless);
    controller = new PIDController(kPSim, 0.0, kDSim);

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

    turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kPReal, 0.0, 0.0);

    turnConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnSparkSim = new SparkMaxSim(turnSpark, gearbox);

    turnSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.004, motorReduction), gearbox);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Run closed-loop control
    if (closedLoop) {
      appliedVolts = controller.calculate(turnSim.getAngularPositionRad()) + feedforwardVolts;
    } else {
      controller.reset();
    }

    // Update state
    turnSparkSim.setAppliedOutput(appliedVolts / RobotConstants.kNominalVoltage);

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
  }

  @Override
  public void setOpenLoop(double output) {
    closedLoop = false;
    appliedVolts = output;
  }

  @Override
  public void setPosition(Rotation2d rotation, AngularVelocity angularVelocity) {
    closedLoop = true;
    double setpoint =
        MathUtil.inputModulus(
            rotation.getRadians() - mechanismOffset.getRadians(), 0.0, 2.0 * Math.PI);
    this.feedforwardVolts =
        RobotConstants.kNominalVoltage
            * angularVelocity.in(RadiansPerSecond)
            / maxAngularVelocity.in(RadiansPerSecond);
    controller.setSetpoint(setpoint);
  }
}
