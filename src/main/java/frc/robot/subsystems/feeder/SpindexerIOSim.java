package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.feeder.FeederConstants.SpindexerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.MotorConstants.NEOVortexConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;

public class SpindexerIOSim implements SpindexerIO {

  private final DCMotor gearbox = DCMotor.getNeoVortex(1);

  private final SparkFlex flex;
  private final SparkClosedLoopController controller;
  private final SparkFlexSim flexSim;

  public SpindexerIOSim() {
    flex = new SparkFlex(port, MotorType.kBrushless);
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
        .velocityConversionFactor(encoderVelocityFactor);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kPSim, 0.0, 0.0);

    flex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flexSim = new SparkFlexSim(flex, gearbox);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    flexSim.iterate(flexSim.getVelocity(), RobotConstants.kNominalVoltage, Robot.defaultPeriodSecs);

    inputs.connected = true;
    inputs.velocityRadPerSec = flexSim.getVelocity();
    inputs.appliedVolts = flexSim.getAppliedOutput() * flexSim.getBusVoltage();
    inputs.currentAmps = Math.abs(flexSim.getMotorCurrent());
  }

  @Override
  public void setOpenLoop(double output) {
    flexSim.setAppliedOutput(output);
  }

  @Override
  public void setVelocity(AngularVelocity angularVelocity) {
    double feedforwardVolts =
        RobotConstants.kNominalVoltage
            * angularVelocity.in(RadiansPerSecond)
            / maxAngularVelocity.in(RadiansPerSecond);
    controller.setSetpoint(
        angularVelocity.in(RadiansPerSecond),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforwardVolts);
  }
}
