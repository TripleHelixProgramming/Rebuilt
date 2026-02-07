package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.launcher.LauncherConstants.TurretConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.MotorConstants.NEO550Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;

public class TurretIOSim implements TurretIO {

  private final DCMotorSim turnSim;

  private final SparkMax turnSpark;
  private final RelativeEncoder turnSparkEncoder;
  private final PIDController controller;
  private final DutyCycleEncoder absoluteEncoder;
  private boolean closedLoop = false;
  private PIDController positionController = new PIDController(kPSim, 0.0, kDSim);
  private double appliedVolts = 0.0;
  private double feedforwardVolts = 0.0;
  private boolean seeded = false;
  private final SparkMaxSim turnSparkSim;

  public TurretIOSim() {

    // Enable wrapping for turn PID
    // positionController.enableContinuousInput(-Math.PI, Math.PI);

    turnSpark = new SparkMax(port, MotorType.kBrushless);
    turnSparkEncoder = turnSpark.getEncoder();
    controller = new PIDController(kPReal, 0.0, 0.0);
    absoluteEncoder =
        new DutyCycleEncoder(
            new DigitalInput(absEncoderPort),
            2 * Math.PI,
            absEncoderOffset.getRadians() + mechanismOffset.getRadians());

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

    turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kPReal, 0.0, 0.0);

    turnConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnSparkSim = new SparkMaxSim(turnSpark, gearbox);

    turnSparkSim.setPosition(-mechanismOffset.getRadians());

    turnSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.004, motorReduction), gearbox);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Run closed-loop control
    if (closedLoop) {
      appliedVolts =
          positionController.calculate(turnSim.getAngularPositionRad()) + feedforwardVolts;
    } else {
      positionController.reset();
    }

    // Update simulation state
    turnSim.setInputVoltage(
        MathUtil.clamp(
            appliedVolts, -RobotConstants.kNominalVoltage, RobotConstants.kNominalVoltage));
    turnSim.update(Robot.defaultPeriodSecs);

    // Update turn inputs
    inputs.motorControllerConnected = true;
    inputs.relativePosition = new Rotation2d(turnSim.getAngularPositionRad()).plus(mechanismOffset);
    inputs.velocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(turnSim.getCurrentDrawAmps());
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
    positionController.setSetpoint(setpoint);
  }
}
