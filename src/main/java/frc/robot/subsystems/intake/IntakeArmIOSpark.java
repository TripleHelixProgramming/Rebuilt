package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.intake.IntakeConstants.ArmConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Constants.MotorConstants.NEOConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.util.SparkOdometryThread;
import frc.robot.util.SparkOdometryThread.SparkInputs;

public class IntakeArmIOSpark implements IntakeArmIO {
  private static final double kPPos = 1.0;
  private static final double kPVel = 1.0;

  private final SparkMax intakeArmRight;
  private final SparkMax intakeArmLeft;
  private final RelativeEncoder encoderSpark;
  private final SparkClosedLoopController intakeArmController;
  private final SparkInputs sparkInputs;

  private final SparkMaxConfig rightArmConfig;
  private final SparkMaxConfig leftArmConfig;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeArmIOSpark() {
    intakeArmRight = new SparkMax(CAN2.intakeArmRight, MotorType.kBrushless);
    intakeArmLeft = new SparkMax(CAN2.intakeArmLeft, MotorType.kBrushless);
    encoderSpark = intakeArmRight.getEncoder();
    intakeArmController = intakeArmRight.getClosedLoopController();

    rightArmConfig = new SparkMaxConfig();

    rightArmConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(NEOConstants.kDefaultSupplyCurrentLimit)
        .voltageCompensation(RobotConstants.kNominalVoltage);

    rightArmConfig
        .encoder
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor);

    rightArmConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kPPos, 0.0, 0.0, ClosedLoopSlot.kSlot0)
        .pid(kPVel, 0.0, 0.0, ClosedLoopSlot.kSlot1);

    rightArmConfig
        .softLimit
        .forwardSoftLimit(maxPosRad)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(minPosRad)
        .reverseSoftLimitEnabled(true);

    leftArmConfig = new SparkMaxConfig();

    leftArmConfig.apply(rightArmConfig).follow(CAN2.intakeArmRight, true);

    rightArmConfig
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        intakeArmRight,
        5,
        () ->
            intakeArmRight.configure(
                rightArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        intakeArmLeft,
        5,
        () ->
            intakeArmLeft.configure(
                leftArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    sparkInputs = SparkOdometryThread.getInstance().registerSpark(intakeArmRight, encoderSpark);
  }

  @Override
  public void updateInputs(IntakeArmIOInputs inputs) {
    inputs.position = sparkInputs.getPosition();
    inputs.velocityMetersPerSec = sparkInputs.getVelocity();
    inputs.appliedVolts = sparkInputs.getAppliedVolts();
    inputs.currentAmps = sparkInputs.getOutputCurrent();
    inputs.connected = connectedDebounce.calculate(sparkInputs.isConnected());
  }

  @Override
  public void setOpenLoop(Voltage volts) {
    intakeArmRight.setVoltage(volts);
  }

  @Override
  public void setPosition(Angle rotation, AngularVelocity velocity) {
    double feedforward =
        RobotConstants.kNominalVoltage
            * velocity.in(RadiansPerSecond)
            / maxAngularVelocity.in(RadiansPerSecond);
    double setpoint = MathUtil.clamp(rotation.magnitude(), minPosRad, maxPosRad);
    intakeArmController.setSetpoint(
        setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    intakeArmController.setSetpoint(
        velocity.in(RadiansPerSecond), ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  @Override
  public void configureSoftLimits(boolean enable) {
    rightArmConfig.softLimit.forwardSoftLimitEnabled(enable);
    rightArmConfig.softLimit.reverseSoftLimitEnabled(enable);
    tryUntilOk(
        intakeArmRight,
        5,
        () ->
            intakeArmRight.configure(
                rightArmConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters));
    tryUntilOk(
        intakeArmLeft,
        5,
        () ->
            intakeArmLeft.configure(
                leftArmConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void resetEncoder() {
    encoderSpark.setPosition(0.0);
  }
}
