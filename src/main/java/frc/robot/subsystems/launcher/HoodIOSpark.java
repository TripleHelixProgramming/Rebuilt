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
import java.util.function.DoubleSupplier;

public class HoodIOSpark implements HoodIO {

  private final SparkBase hoodSpark;
  private final RelativeEncoder encoderSpark;
  private final SparkClosedLoopController hoodController;
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private boolean relatvieEncoderSeeded = false;

  public HoodIOSpark() {
    hoodSpark = new SparkMax(CAN2.hood, MotorType.kBrushless);
    encoderSpark = hoodSpark.getEncoder();
    hoodController = hoodSpark.getClosedLoopController();

    var hoodConfig = new SparkMaxConfig();

    hoodConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(NEO550Constants.kDefaultSupplyCurrentLimit)
        .voltageCompensation(RobotConstants.kNominalVoltage);

    hoodConfig
        .encoder
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor);

    hoodConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(kPReal, 0.0, 0.0);

    hoodConfig
        .softLimit
        .forwardSoftLimit(maxValue)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(minValue)
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
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (!relatvieEncoderSeeded) {
      encoderSpark.setPosition(maxValue);
      relatvieEncoderSeeded = true;
    }

    sparkStickyFault = false;
    ifOk(hoodSpark, encoderSpark::getPosition, (value) -> inputs.position = new Rotation2d(value));
    ifOk(hoodSpark, encoderSpark::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        hoodSpark,
        new DoubleSupplier[] {hoodSpark::getAppliedOutput, hoodSpark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(hoodSpark, hoodSpark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.connected = turnConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setOpenLoop(double output) {
    hoodSpark.setVoltage(output);
  }

  @Override
  public void setPosition(Rotation2d rotation, AngularVelocity angularVelocity) {
    double setpoint = MathUtil.clamp(rotation.getRadians(), minValue, maxValue);
    double feedforwardVolts =
        RobotConstants.kNominalVoltage
            * angularVelocity.in(RadiansPerSecond)
            / maxAngularVelocity.in(RadiansPerSecond);
    hoodController.setSetpoint(
        setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforwardVolts);
  }
}
