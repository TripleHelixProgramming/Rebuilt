package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.launcher.LauncherConstants.TurretConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Constants.DIOPorts;
import frc.robot.Constants.MotorConstants.NEO550Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.util.SparkOdometryThread;
import frc.robot.util.SparkOdometryThread.SparkInputs;

public class TurretIOSpark implements TurretIO {

  private final SparkBase turnSpark;
  private final RelativeEncoder turnSparkEncoder;
  private final SparkClosedLoopController controller;
  private final DutyCycleEncoder absoluteEncoder;
  private final SparkInputs sparkInputs;

  private final Debouncer motorControllerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer absEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private boolean relativeEncoderSeeded = false;

  public TurretIOSpark() {
    turnSpark = new SparkMax(CAN2.turret, MotorType.kBrushless);
    controller = turnSpark.getClosedLoopController();
    turnSparkEncoder = turnSpark.getEncoder();
    absoluteEncoder =
        new DutyCycleEncoder(
            new DigitalInput(DIOPorts.turretAbsEncoder),
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

    turnConfig
        .softLimit
        .forwardSoftLimit(Math.PI + rangeOfMotion.div(2).in(Radians))
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(Math.PI - rangeOfMotion.div(2).in(Radians))
        .reverseSoftLimitEnabled(true);

    turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kPReal, 0.0, 0.0);

    turnConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Register with background thread for non-blocking CAN reads
    sparkInputs = SparkOdometryThread.getInstance().registerSpark(turnSpark, turnSparkEncoder);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (!relativeEncoderSeeded && inputs.absoluteEncoderConnected) {
      turnSparkEncoder.setPosition(absoluteEncoder.get());
      relativeEncoderSeeded = true;
    }

    // Read from cached values (non-blocking) - updated by SparkOdometryThread
    inputs.relativePosition = new Rotation2d(sparkInputs.getPosition()).plus(mechanismOffset);
    inputs.velocityRadPerSec = sparkInputs.getVelocity();
    inputs.appliedVolts = sparkInputs.getAppliedVolts();
    inputs.currentAmps = sparkInputs.getOutputCurrent();
    inputs.motorControllerConnected =
        motorControllerConnectedDebounce.calculate(sparkInputs.isConnected());

    // Absolute encoder is read directly (DIO, not CAN - already fast)
    inputs.absoluteEncoderConnected =
        absEncoderConnectedDebounce.calculate(absoluteEncoder.isConnected());
    inputs.absolutePosition = new Rotation2d(absoluteEncoder.get());
  }

  @Override
  public void setOpenLoop(double output) {
    controller.setSetpoint(output, ControlType.kDutyCycle);
  }

  @Override
  public void setPosition(Rotation2d rotation, AngularVelocity angularVelocity) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.getRadians() - mechanismOffset.getRadians(), 0.0, 2 * Math.PI);
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
