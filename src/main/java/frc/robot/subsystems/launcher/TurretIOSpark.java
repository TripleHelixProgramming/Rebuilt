package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.launcher.LauncherConstants.TurretConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.MotorConstants.NEO550Constants;
import frc.robot.Constants.RobotConstants;
import java.util.function.DoubleSupplier;

public class TurretIOSpark implements TurretIO {

  private final SparkBase turnSpark;
  private final RelativeEncoder turnSparkEncoder;
  private final PIDController controller;
  private final DutyCycleEncoder absoluteEncoder;
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;
  private double feedforwardVolts = 0.0;
  private boolean seeded = false;

  public TurretIOSpark() {
    turnSpark = new SparkMax(port, MotorType.kBrushless);
    turnSparkEncoder = turnSpark.getEncoder();
    controller = new PIDController(kPReal, 0.0, 0.0);
    absoluteEncoder =
        new DutyCycleEncoder(
            new DigitalInput(DIOPort),
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

    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (!seeded && absoluteEncoder.isConnected()) {
      turnSparkEncoder.setPosition(absoluteEncoder.get());
      seeded = true;
    }

    // Run closed-loop control
    if (closedLoop) {
      appliedVolts = controller.calculate(turnSparkEncoder.getPosition()) + feedforwardVolts;
    } else {
      controller.reset();
    }

    // Update state
    turnSpark.setVoltage(
        MathUtil.clamp(
            appliedVolts, -RobotConstants.kNominalVoltage, RobotConstants.kNominalVoltage));

    sparkStickyFault = false;
    ifOk(
        turnSpark,
        turnSparkEncoder::getPosition,
        (value) -> inputs.relativePosition = new Rotation2d(value).plus(mechanismOffset));
    ifOk(turnSpark, turnSparkEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.motorControllerConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    inputs.absoluteEncoderConnected = absoluteEncoder.isConnected();
    inputs.absolutePosition = new Rotation2d(absoluteEncoder.get());
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
            rotation.getRadians() - mechanismOffset.getRadians(), minInput, maxInput);
    this.feedforwardVolts =
        RobotConstants.kNominalVoltage
            * angularVelocity.in(RadiansPerSecond)
            / maxAngularVelocity.in(RadiansPerSecond);
    controller.setSetpoint(setpoint);
  }
}
