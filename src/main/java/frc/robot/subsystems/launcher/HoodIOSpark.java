package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.launcher.LauncherConstants.HoodConstants.*;
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
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Constants.MotorConstants.NEO550Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.util.SparkOdometryThread;
import frc.robot.util.SparkOdometryThread.SparkInputs;

public class HoodIOSpark implements HoodIO {

  private final SparkBase hoodSpark;
  private final RelativeEncoder encoderSpark;
  private final SparkClosedLoopController hoodController;
  private final SparkInputs sparkInputs;

  private final SparkMaxConfig hoodConfig;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public HoodIOSpark() {
    hoodSpark = new SparkMax(CAN2.hood, MotorType.kBrushless);
    encoderSpark = hoodSpark.getEncoder();
    hoodController = hoodSpark.getClosedLoopController();

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

    hoodConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kPReal, 0.0, 0.0);

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

    tryUntilOk(
        hoodSpark,
        5,
        () ->
            hoodSpark.configure(
                hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Register with background thread for non-blocking CAN reads
    sparkInputs = SparkOdometryThread.getInstance().registerSpark(hoodSpark, encoderSpark);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Read from cached values (non-blocking) - updated by SparkOdometryThread
    inputs.position = new Rotation2d(sparkInputs.getPosition());
    inputs.velocityRadPerSec = sparkInputs.getVelocity();
    inputs.appliedVolts = sparkInputs.getAppliedVolts();
    inputs.currentAmps = sparkInputs.getOutputCurrent();
    inputs.connected = connectedDebounce.calculate(sparkInputs.isConnected());
  }

  @Override
  public void setOpenLoop(double output) {
    hoodSpark.setVoltage(output);
  }

  @Override
  public void setPosition(Rotation2d rotation, AngularVelocity angularVelocity) {
    double setpoint = MathUtil.clamp(rotation.getRadians(), minPosRad, maxPosRad);
    double feedforwardVolts =
        RobotConstants.kNominalVoltage
            * angularVelocity.in(RadiansPerSecond)
            / maxAngularVelocity.in(RadiansPerSecond);
    hoodController.setSetpoint(
        setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforwardVolts);
  }

  @Override
  public void setVelocity(AngularVelocity angularVelocity) {
    double setpoint = angularVelocity.in(RadiansPerSecond);
    double feedforwardVolts =
        RobotConstants.kNominalVoltage
            * angularVelocity.in(RadiansPerSecond)
            / maxAngularVelocity.in(RadiansPerSecond);
    hoodController.setSetpoint(
        setpoint, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforwardVolts);
  }

  @Override
  public void configureSoftLimits(boolean enable) {
    hoodConfig.softLimit.forwardSoftLimitEnabled(enable);
    hoodConfig.softLimit.reverseSoftLimitEnabled(enable);
    hoodSpark.configure(
        hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void resetEncoder() {
    encoderSpark.setPosition(maxPosRad);
  }
}
