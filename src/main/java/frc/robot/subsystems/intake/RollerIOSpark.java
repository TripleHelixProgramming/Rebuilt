package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.RollerConstants.*;
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
import frc.robot.Constants.MotorConstants.NEOVortexConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.intake.IntakeConstants.RollerConfig;
import frc.robot.util.SparkOdometryThread;
import frc.robot.util.SparkOdometryThread.SparkInputs;

public class RollerIOSpark implements RollerIO {
  private static final double KP = 0.001;
  private static final double KD = 0.0;
  private static final LinearVelocity maxTangentialVelocity =
      MetersPerSecond.of(
          NEOVortexConstants.kFreeSpeed.in(RadiansPerSecond)
              * rollerRadius.in(Meters)
              / motorReduction);

  private final SparkFlex flex;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final SparkInputs sparkInputs;

  public RollerIOSpark(RollerConfig rollerConfig) {
    flex = new SparkFlex(rollerConfig.port, MotorType.kBrushless);
    encoder = flex.getEncoder();
    controller = flex.getClosedLoopController();

    var config = new SparkFlexConfig();
    config
        .inverted(rollerConfig.inverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(NEOVortexConstants.kDefaultSupplyCurrentLimit)
        .voltageCompensation(RobotConstants.kNominalVoltage);

    config
        .encoder
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(8);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(KP, 0.0, KD);

    tryUntilOk(
        flex,
        5,
        () ->
            flex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    sparkInputs = SparkOdometryThread.getInstance().registerSpark(flex, encoder);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.connected = sparkInputs.isConnected();
    inputs.velocityMetersPerSec = sparkInputs.getVelocity() * rollerRadius.in(Meters);
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
        RobotConstants.kNominalVoltage
            * tangentialVelocity.in(MetersPerSecond)
            / maxTangentialVelocity.in(MetersPerSecond);
    controller.setSetpoint(
        tangentialVelocity.in(MetersPerSecond) / rollerRadius.in(Meters),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforwardVolts);
  }
}
