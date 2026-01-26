package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.launcher.LauncherConstants.TurretConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
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
import frc.robot.Constants.MotorConstants.NEO550Constants;
import frc.robot.Constants.RobotConstants;
import java.util.function.DoubleSupplier;

public class TurretIOSpark implements TurretIO {

  private final SparkBase turnSpark;
  private final AbsoluteEncoder turnEncoder;
  private final SparkClosedLoopController turnController;
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public TurretIOSpark() {
    turnSpark = new SparkMax(port, MotorType.kBrushless);
    turnEncoder = turnSpark.getAbsoluteEncoder();
    turnController = turnSpark.getClosedLoopController();

    var turnConfig = new SparkMaxConfig();

    turnConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(NEO550Constants.kDefaultSupplyCurrentLimit)
        .voltageCompensation(RobotConstants.kNominalVoltage);

    turnConfig
        .absoluteEncoder
        .inverted(false)
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .averageDepth(2);

    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pid(turnKp, 0.0, 0.0);

    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20)
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.position = new Rotation2d(value).minus(rotationOffset));
    ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.connected = turnConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setPosition(Rotation2d rotation, AngularVelocity angularVelocity) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(rotationOffset).getRadians(), turnPIDMinInput, turnPIDMaxInput);
    double feedforwardVolts =
        RobotConstants.kNominalVoltage
            * angularVelocity.in(RadiansPerSecond)
            / turnMaxAngularVelocity.in(RadiansPerSecond);
    turnController.setSetpoint(
        setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforwardVolts);
  }
}
