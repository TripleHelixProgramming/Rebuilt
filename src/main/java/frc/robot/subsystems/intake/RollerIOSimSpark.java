package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.RollerConstants.*;

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
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.MotorConstants.NEOVortexConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeConstants.RollerConfig;

public class RollerIOSimSpark implements RollerIO {
  private static final double KICKER_MOI_KG_M2 = 0.00052;
  private static final double KP = 0.11;
  private static final double KD = 0.0;
  private static final LinearVelocity MAX_TANGENTIAL_VELOCITY =
      MetersPerSecond.of(
          NEOVortexConstants.FREE_SPEED.in(RadiansPerSecond)
              * rollerRadius.in(Meters)
              / MOTOR_REDUCTION);
  private static final DCMotor GEARBOX = DCMotor.getNeoVortex(numMotors);

  private final DCMotorSim rollerSim;

  private final SparkFlex flex;
  private final SparkClosedLoopController controller;
  private final SparkFlexSim flexSim;

  public RollerIOSimSpark(RollerConfig rollerConfig) {
    flex = new SparkFlex(rollerConfig.port, MotorType.kBrushless);
    controller = flex.getClosedLoopController();

    var config = new SparkFlexConfig();
    config
        .inverted(rollerConfig.inverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(NEOVortexConstants.DEFAULT_SUPPLY_CURRENT_LIMIT)
        .voltageCompensation(RobotConstants.NOMINAL_VOLTAGE);

    config
        .encoder
        .positionConversionFactor(ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(KP, 0.0, KD);

    flex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flexSim = new SparkFlexSim(flex, GEARBOX);

    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(GEARBOX, KICKER_MOI_KG_M2, MOTOR_REDUCTION),
            GEARBOX);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    // Update simulation state
    double busVoltage = RoboRioSim.getVInVoltage();
    rollerSim.setInput(flexSim.getAppliedOutput() * busVoltage);
    rollerSim.update(Robot.defaultPeriodSecs);
    flexSim.iterate(rollerSim.getAngularVelocityRadPerSec(), busVoltage, Robot.defaultPeriodSecs);

    // Update inputs
    inputs.connected = true;
    inputs.velocityMetersPerSec = flexSim.getVelocity() * rollerRadius.in(Meters);
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
        tangentialVelocity.in(MetersPerSecond) / rollerRadius.in(Meters),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforwardVolts);
  }
}
