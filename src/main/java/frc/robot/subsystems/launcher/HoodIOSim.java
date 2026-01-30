package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.launcher.LauncherConstants.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;

public class HoodIOSim implements HoodIO {

  private final DCMotorSim hoodSim;

  private boolean closedLoop = false;
  private PIDController positionController = new PIDController(hoodKpSim, 0.0, hoodKdSim);
  private double appliedVolts = 0.0;
  private double feedforwardVolts = 0.0;

  public HoodIOSim() {
    hoodSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(hoodGearbox, 0.004, hoodMotorReduction),
            hoodGearbox);
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
    hoodSim.setInputVoltage(MathUtil.clamp(appliedVolts, -RobotConstants.kNominalVoltage, RobotConstants.kNominalVoltage));
    hoodSim.update(Robot.defaultPeriodSecs);

    // Update turn inputs
    inputs.connected = true;
    inputs.position = new Rotation2d(hoodSim.getAngularPositionRad());
    inputs.velocityRadPerSec = hoodSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(hoodSim.getCurrentDrawAmps());
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
            / hoodMaxAngularVelocity.in(RadiansPerSecond);
    positionController.setSetpoint(rotation.getRadians());
  }
}
