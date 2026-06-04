package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;
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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Constants.MotorConstants.NEOVortexConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;

public class SpindexerIOSimSpark implements SpindexerIO {
  private static final double SPINDEXER_MOI_KG_M2 = 0.00207;

  private final DCMotorSim spindexerSim;

  private final SparkFlex flex;
  private final SparkClosedLoopController controller;
  private final SparkFlexSim flexSim;

  public SpindexerIOSimSpark() {
    flex = new SparkFlex(CAN2.SPINDEXER, MotorType.kBrushless);
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
        .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, 0.0, kD);

    flex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flexSim = new SparkFlexSim(flex, GEARBOX);

    spindexerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(GEARBOX, SPINDEXER_MOI_KG_M2, MOTOR_REDUCTION),
            GEARBOX);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    // Update simulation state
    double busVoltage = RoboRioSim.getVInVoltage();
    spindexerSim.setInput(flexSim.getAppliedOutput() * busVoltage);
    spindexerSim.update(Robot.defaultPeriodSecs);
    flexSim.iterate(
        spindexerSim.getAngularVelocityRadPerSec(), busVoltage, Robot.defaultPeriodSecs);

    // Update inputs
    inputs.connected = true;
    inputs.velocityMetersPerSec = flexSim.getVelocity() * RADIUS.in(Meters);
    inputs.appliedVolts = flexSim.getAppliedOutput() * flexSim.getBusVoltage();
    inputs.currentAmps = Math.abs(flexSim.getMotorCurrent());
  }

  @Override
  public void setOpenLoop(Voltage volts) {
    flexSim.setAppliedOutput(volts.in(Volts) / RobotConstants.NOMINAL_VOLTAGE);
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
