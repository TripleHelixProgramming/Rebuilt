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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.MotorConstants.NEOConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;

public class IntakeArmIOSimSpark implements IntakeArmIO {
  private static final double kP = 1.0;
  private static final double kD = 1.0;

  private final SingleJointedArmSim armSim;

  private final SparkMax motor;
  private final SparkClosedLoopController controller;
  private final SparkMaxSim motorSim;
  private final boolean hasAbsoluteEncoder;

  private final SparkMaxConfig motorConfig;

  public IntakeArmIOSimSpark(ArmConfig armConfig) {
    motor = new SparkMax(armConfig.port(), MotorType.kBrushless);
    hasAbsoluteEncoder = armConfig.hasAbsoluteEncoder();

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

    // Starts stowed (maxPosRad), matching where the real arm sits at boot.
    armSim =
        new SingleJointedArmSim(
            gearbox,
            motorReduction,
            MOMENT_OF_INERTIA_KG_M2,
            ARM_LENGTH_METERS,
            minPosRad,
            maxPosRad,
            true,
            maxPosRad);

    motorSim.setPosition(maxPosRad);
  }

  @Override
  public void updateInputs(IntakeArmIOInputs inputs) {
    // Update simulation state
    double busVoltage = RoboRioSim.getVInVoltage();
    armSim.setInputVoltage(motorSim.getAppliedOutput() * busVoltage);
    armSim.update(Robot.defaultPeriodSecs);

    motorSim.iterate(armSim.getVelocityRadPerSec(), busVoltage, Robot.defaultPeriodSecs);

    // Update inputs
    inputs.connected = true;
    inputs.positionRad = motorSim.getPosition();
    inputs.velocityRadPerSec = motorSim.getVelocity();
    inputs.appliedVolts = motorSim.getAppliedOutput() * motorSim.getBusVoltage();
    inputs.currentAmps = Math.abs(motorSim.getMotorCurrent());

    if (hasAbsoluteEncoder) {
      // No offset/calibration to simulate — the sim arm's position is already ground truth, so
      // it's reported directly, matching the real absolute encoder's convention.
      inputs.absolutePosition = new Rotation2d(motorSim.getPosition());
    }
  }

  @Override
  public void setPosition(Angle rotation, AngularVelocity velocity) {
    double feedforward =
        RobotConstants.NOMINAL_VOLTAGE
                * velocity.in(RadiansPerSecond)
                / maxAngularVelocity.in(RadiansPerSecond)
            + kG * Math.cos(motorSim.getPosition());
    controller.setSetpoint(
        rotation.magnitude(), ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);
  }

  @Override
  public void resetEncoder(Angle position) {
    motorSim.setPosition(position.magnitude());
  }
}
