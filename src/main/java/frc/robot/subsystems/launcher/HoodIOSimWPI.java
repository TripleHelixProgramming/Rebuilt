package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.launcher.LauncherConstants.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;

public class HoodIOSimWPI implements HoodIO {

  private final DCMotorSim hoodSim;

  private boolean closedLoop = false;
  private PIDController positionController = new PIDController(kPSimPos, 0.0, kDSimPos);
  private double appliedVolts = 0.0;
  private double feedforwardVolts = 0.0;

  public HoodIOSimWPI() {
    hoodSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.004, motorReduction), gearbox);

    hoodSim.setState(0, 0.0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Run closed-loop control
    if (closedLoop) {
      appliedVolts =
          positionController.calculate(hoodSim.getAngularPositionRad()) + feedforwardVolts;
    } else {
      positionController.reset();
    }

    // Update simulation state
    hoodSim.setInputVoltage(
        MathUtil.clamp(
            appliedVolts, -RobotConstants.kNominalVoltage, RobotConstants.kNominalVoltage));
    hoodSim.update(Robot.defaultPeriodSecs);

    // Update turn inputs
    inputs.connected = true;
    inputs.position = new Rotation2d(hoodSim.getAngularPositionRad());
    inputs.velocityRadPerSec = hoodSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(hoodSim.getCurrentDrawAmps());

    if (hoodSim.getAngularPositionRad() > maxPosRad) {
      hoodSim.setState(maxPosRad, 0);
    }
  }

  @Override
  public void setOpenLoop(double output) {
    closedLoop = false;
    appliedVolts = output;
  }

  @Override
  public void setPosition(Rotation2d rotation, AngularVelocity angularVelocity) {
    closedLoop = true;
    double setpoint = MathUtil.clamp(rotation.getRadians(), minPosRad, maxPosRad);
    this.feedforwardVolts =
        RobotConstants.kNominalVoltage
            * angularVelocity.in(RadiansPerSecond)
            / maxAngularVelocity.in(RadiansPerSecond);
    positionController.setSetpoint(setpoint);
  }

  @Override
  public void setVelocity(AngularVelocity angularVelocity) {
    closedLoop = true;
    double setpoint = angularVelocity.in(RadiansPerSecond);
    hoodSim.setAngularVelocity(setpoint);
  }

  @Override
  public void configureSoftLimits(boolean enable) {}

  @Override
  public void resetEncoder() {
    hoodSim.setAngle(maxPosRad);
  }
}
