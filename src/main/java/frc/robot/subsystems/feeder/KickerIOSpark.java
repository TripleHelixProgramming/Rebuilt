package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.feeder.FeederConstants.KickerConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Constants.MotorConstants.NEOVortexConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.util.SparkOdometryThread;
import frc.robot.util.SparkOdometryThread.SparkInputs;

public class KickerIOSpark implements KickerIO {

  private final SparkFlex flex;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final SparkInputs sparkInputs;

  public KickerIOSpark() {
    flex = new SparkFlex(CAN2.KICKER, MotorType.kBrushless);
    encoder = flex.getEncoder();
    controller = flex.getClosedLoopController();

    var config = new SparkFlexConfig();
    config
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(NEOVortexConstants.DEFAULT_STATOR_CURRENT_LIMIT)
        .voltageCompensation(RobotConstants.NOMINAL_VOLTAGE);

    config
        .encoder
        .positionConversionFactor(ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(ENCODER_VELOCITY_FACTOR)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(8);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, 0.0, kD);

    tryUntilOk(
        flex,
        5,
        () ->
            flex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    sparkInputs = SparkOdometryThread.getInstance().registerSpark(flex, encoder);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {

    inputs.connected = sparkInputs.isConnected();
    inputs.velocityMetersPerSec = sparkInputs.getVelocity() * RADIUS.in(Meters);
    inputs.appliedVolts = sparkInputs.getAppliedVolts();
    inputs.currentAmps = sparkInputs.getOutputCurrent();
  }

  @Override
  public void setOpenLoop(Voltage volts) {
    flex.setVoltage(volts);
    ;
  }

  @Override
  public void setVelocity(LinearVelocity tangentialVelocity) {
    double feedforwardVolts =
        RobotConstants.NOMINAL_VOLTAGE
            * tangentialVelocity.in(MetersPerSecond)
            / MAX_TANGENTIAL_VELOCITY.in(MetersPerSecond);
    controller.setSetpoint(
        tangentialVelocity.in(MetersPerSecond) / RADIUS.in(Meters),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforwardVolts);
  }
}
