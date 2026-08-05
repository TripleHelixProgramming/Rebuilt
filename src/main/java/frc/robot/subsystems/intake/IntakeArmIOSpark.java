package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.intake.IntakeConstants.ArmConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.MotorConstants.NEOConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.util.SparkOdometryThread;
import frc.robot.util.SparkOdometryThread.SparkInputs;

public class IntakeArmIOSpark implements IntakeArmIO {
  private static final double kPPos = 1.0;

  private final SparkMax motor;
  private final AbsoluteEncoder absEncoder;
  private final RelativeEncoder relEncoder;
  private final SparkClosedLoopController controller;
  private final SparkInputs sparkInputs;

  private final SparkMaxConfig motorConfig;
  private final AbsoluteEncoderConfig absEncoderConfig;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeArmIOSpark(ArmConfig armConfig) {
    motor = new SparkMax(armConfig.port(), MotorType.kBrushless);
    absEncoder = motor.getAbsoluteEncoder();
    relEncoder = motor.getEncoder();
    controller = motor.getClosedLoopController();

    absEncoderConfig = new AbsoluteEncoderConfig();

    absEncoderConfig
        .inverted(true)
        .zeroOffset(absEncoderOffset)
        .positionConversionFactor(absEncoderPositionFactor)
        .velocityConversionFactor(absEncoderVelocityFactor);

    motorConfig = new SparkMaxConfig();

    motorConfig
        .inverted(armConfig.inverted())
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(NEOConstants.DEFAULT_SUPPLY_CURRENT_LIMIT)
        .voltageCompensation(RobotConstants.NOMINAL_VOLTAGE);

    motorConfig
        .encoder
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor);

    motorConfig.absoluteEncoder.apply(absEncoderConfig);

    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kPPos, 0.0, 0.0, ClosedLoopSlot.kSlot0);

    motorConfig
        .softLimit
        .forwardSoftLimit(maxPosRad)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(minPosRad)
        .reverseSoftLimitEnabled(true);

    motorConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    sparkInputs = SparkOdometryThread.getInstance().registerSpark(motor, relEncoder);
  }

  @Override
  public void updateInputs(IntakeArmIOInputs inputs) {
    inputs.positionRad = sparkInputs.getPosition();
    inputs.velocityRadPerSec = sparkInputs.getVelocity();
    inputs.appliedVolts = sparkInputs.getAppliedVolts();
    inputs.currentAmps = sparkInputs.getOutputCurrent();
    inputs.connected = connectedDebounce.calculate(sparkInputs.isConnected());

    inputs.absolutePosition = new Rotation2d(absEncoder.getPosition());
  }

  @Override
  public void setPosition(Angle rotation, AngularVelocity velocity) {
    double feedforward =
        RobotConstants.NOMINAL_VOLTAGE
                * velocity.in(RadiansPerSecond)
                / maxAngularVelocity.in(RadiansPerSecond)
            + kG * Math.cos(relEncoder.getPosition());
    double setpoint = MathUtil.clamp(rotation.magnitude(), minPosRad, maxPosRad);
    controller.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);
  }

  @Override
  public void resetEncoder(Angle position) {
    relEncoder.setPosition(position.magnitude());
  }
}
