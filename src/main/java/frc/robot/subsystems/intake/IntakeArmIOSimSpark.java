package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.ArmConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.MotorConstants.NEOConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;

public class IntakeArmIOSimSpark implements IntakeArmIO {
  private static final double kP = 1.0;
  private static final double kD = 1.0;

  private final DCMotorSim armSim;

  private final SparkMax motor;
  private final SparkClosedLoopController controller;
  private final SparkMaxSim motorSim;

  private final SparkMaxConfig motorConfig;

  public IntakeArmIOSimSpark(ArmConfig armConfig) {
    motor = new SparkMax(armConfig.port(), MotorType.kBrushless);

    controller = motor.getClosedLoopController();

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

    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, 0.0, kD);

    motorConfig
        .softLimit
        .forwardSoftLimit(maxPosRad)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(minPosRad)
        .reverseSoftLimitEnabled(true);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorSim = new SparkMaxSim(motor, gearbox);

    armSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.004, motorReduction), gearbox);

    armSim.setState(0.0, 0.0);
    motorSim.setPosition(0.0);
  }

  @Override
  public void updateInputs(IntakeArmIOInputs inputs) {
    // Update simulation state
    double busVoltage = RoboRioSim.getVInVoltage();
    armSim.setInput(motorSim.getAppliedOutput() * busVoltage);
    armSim.update(Robot.defaultPeriodSecs);

    motorSim.iterate(armSim.getAngularVelocityRadPerSec(), busVoltage, Robot.defaultPeriodSecs);

    // Update inputs
    inputs.connected = true;
    inputs.position = motorSim.getPosition();
    inputs.velocityMetersPerSec = motorSim.getVelocity();
    inputs.appliedVolts = motorSim.getAppliedOutput() * motorSim.getBusVoltage();
    inputs.currentAmps = Math.abs(motorSim.getMotorCurrent());
  }

  @Override
  public void setPosition(Angle rotation, AngularVelocity velocity) {
    double feedforward =
        RobotConstants.NOMINAL_VOLTAGE
            * velocity.in(RadiansPerSecond)
            / maxAngularVelocity.in(RadiansPerSecond);
    controller.setSetpoint(
        rotation.magnitude(), ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);
  }

  @Override
  public void resetEncoder(Angle position) {
    motorSim.setPosition(position.magnitude());
  }
}
