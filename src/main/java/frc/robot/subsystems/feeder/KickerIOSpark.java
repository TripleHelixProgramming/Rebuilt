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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Constants.MotorConstants.NEOVortexConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;
import frc.robot.util.SparkOdometryThread;
import frc.robot.util.SparkOdometryThread.SparkInputs;

public class KickerIOSpark implements KickerIO {

  private final SparkFlex flex;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final SparkInputs sparkInputs;

  // Trapezoidal profile to limit angular acceleration (rad/s and rad/s^2)
  private final TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              // max angular velocity (rad/s)
              maxTangentialVelocity.in(MetersPerSecond) / radius.in(Meters),
              // max angular acceleration (rad/s^2) derived from tangential accel
              maxTangentialAcceleration / radius.in(Meters)));

  public KickerIOSpark() {
    flex = new SparkFlex(CAN2.kicker, MotorType.kBrushless);
    encoder = flex.getEncoder();
    controller = flex.getClosedLoopController();

    var config = new SparkFlexConfig();
    config
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(NEOVortexConstants.kDefaultSupplyCurrentLimit)
        .voltageCompensation(RobotConstants.kNominalVoltage);

    config
        .encoder
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(8);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kPSim, 0.0, kDSim);

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
    inputs.velocityMetersPerSec = sparkInputs.getVelocity() * radius.in(Meters);
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
    double desiredAngular = tangentialVelocity.in(MetersPerSecond) / radius.in(Meters);

    TrapezoidProfile.State goal = new TrapezoidProfile.State(desiredAngular, 0.0);
    TrapezoidProfile.State setpoint = new TrapezoidProfile.State(sparkInputs.getVelocity(), 0.0);

    setpoint = profile.calculate(Robot.defaultPeriodSecs, setpoint, goal);

    double tangentialSetpoint = setpoint.position * radius.in(Meters);
    double feedforwardVolts =
        RobotConstants.kNominalVoltage
            * tangentialSetpoint
            / maxTangentialVelocity.in(MetersPerSecond);

    controller.setSetpoint(
        setpoint.position, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforwardVolts);
  }
}
