package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.launcher.LauncherConstants.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;

public class TurretIOSim implements TurretIO {
  private final DCMotorSim turnSim;

  private boolean closedLoop = false;
  private PIDController positionController = new PIDController(kPSim, 0.0, kDSim);
  private double appliedVolts = 0.0;
  private double feedforwardVolts = 0.0;

  public TurretIOSim() {
    turnSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.004, motorReduction), gearbox);

    // Enable wrapping for turn PID
    positionController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Run closed-loop control
    if (closedLoop) {
      appliedVolts =
          positionController.calculate(turnSim.getAngularPositionRad()) + feedforwardVolts;
    } else {
      positionController.reset();
    }

    // Update simulation state
    turnSim.setInputVoltage(
        MathUtil.clamp(
            appliedVolts, -RobotConstants.kNominalVoltage, RobotConstants.kNominalVoltage));
    turnSim.update(Robot.defaultPeriodSecs);

    // Update turn inputs
    inputs.motorControllerConnected = true;
    inputs.relativePosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.velocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(turnSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double output) {
    closedLoop = false;
    appliedVolts = output;
  }

  @Override
  public void setPosition(Rotation2d rotation, AngularVelocity angularVelocity) {
    closedLoop = true;
    this.feedforwardVolts =
        RobotConstants.kNominalVoltage
            * angularVelocity.in(RadiansPerSecond)
            / maxAngularVelocity.in(RadiansPerSecond);
    positionController.setSetpoint(rotation.getRadians());
  }
}
