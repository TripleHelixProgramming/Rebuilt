package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.launcher.LauncherConstants.FlywheelConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;

public class FlywheelIOSimWPI implements FlywheelIO {
  private final DCMotorSim flywheelSim;

  private boolean closedLoop = false;
  private PIDController velocityController = new PIDController(kPSim, 0.0, 0.0);
  private double appliedVolts = 0.0;
  private double feedforwardVolts = 0.0;

  public FlywheelIOSimWPI() {
    flywheelSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.004, motorReduction), gearbox);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Run closed-loop control
    if (closedLoop) {
      appliedVolts =
          velocityController.calculate(flywheelSim.getAngularVelocityRadPerSec())
              + feedforwardVolts;
    } else {
      velocityController.reset();
    }

    // Update simulation state
    flywheelSim.setInputVoltage(
        MathUtil.clamp(
            appliedVolts, -RobotConstants.kNominalVoltage, RobotConstants.kNominalVoltage));
    flywheelSim.update(Robot.defaultPeriodSecs);

    // Update turn inputs
    inputs.connected = true;
    inputs.velocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(flywheelSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double output) {
    closedLoop = false;
    appliedVolts = output;
  }

  @Override
  public void setVelocity(LinearVelocity tangentialVelocity) {
    closedLoop = true;
    this.feedforwardVolts =
        RobotConstants.kNominalVoltage
            * tangentialVelocity.in(RadiansPerSecond)
            / maxAngularVelocity.in(RadiansPerSecond);
    velocityController.setSetpoint(tangentialVelocity.in(RadiansPerSecond));
  }
}
