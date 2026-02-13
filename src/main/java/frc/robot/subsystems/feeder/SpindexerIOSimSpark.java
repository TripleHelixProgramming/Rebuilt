package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
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
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Constants.MotorConstants.NEOVortexConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;

public class SpindexerIOSimSpark implements SpindexerIO {

  private final DCMotorSim spindexerSim;

  private final SparkFlex flex;
  private final SparkClosedLoopController controller;
  private final SparkFlexSim flexSim;

  public SpindexerIOSimSpark() {
    flex = new SparkFlex(CAN2.spindexer, MotorType.kBrushless);
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

    spindexerSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.004, motorReduction), gearbox);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    // Update simulation state
    spindexerSim.setInput(flexSim.getAppliedOutput() * RobotConstants.kNominalVoltage);
    spindexerSim.update(Robot.defaultPeriodSecs);
    flexSim.iterate(
        spindexerSim.getAngularVelocityRadPerSec(),
        RobotConstants.kNominalVoltage,
        Robot.defaultPeriodSecs);

    // Update inputs
    inputs.connected = true;
    inputs.velocityMetersPerSec = flexSim.getVelocity() * radius.in(Meters);
    inputs.appliedVolts = flexSim.getAppliedOutput() * flexSim.getBusVoltage();
    inputs.currentAmps = Math.abs(flexSim.getMotorCurrent());
  }

  @Override
  public void setOpenLoop(double output) {
    flexSim.setAppliedOutput(output);
  }

  @Override
  public void setVelocity(LinearVelocity tangentialVelocity) {
    double feedforwardVolts =
        RobotConstants.kNominalVoltage
            * tangentialVelocity.in(MetersPerSecond)
            / maxTangentialVelocity.in(MetersPerSecond);
    controller.setSetpoint(
        tangentialVelocity.in(MetersPerSecond) / radius.in(Meters),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforwardVolts);
  }
}
