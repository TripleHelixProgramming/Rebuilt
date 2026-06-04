package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.launcher.LauncherConstants.FlywheelConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;

public class FlywheelIOSimWPI implements FlywheelIO {
  private static final double KP_SIM = 0.1;

  private final DCMotorSim flywheelSim;

  private boolean closedLoop = false;
  private PIDController velocityController = new PIDController(KP_SIM, 0.0, 0.0);
  private double appliedVolts = 0.0;
  private double feedforwardVolts = 0.0;

  public FlywheelIOSimWPI() {
    flywheelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(GEARBOX, 0.004, MOTOR_REDUCTION), GEARBOX);
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
            appliedVolts, -RobotConstants.NOMINAL_VOLTAGE, RobotConstants.NOMINAL_VOLTAGE));
    flywheelSim.update(Robot.defaultPeriodSecs);

    // Update turn inputs
    inputs.connected = true;
    inputs.velocityMetersPerSec =
        flywheelSim.getAngularVelocityRadPerSec() * WHEEL_RADIUS.in(Meters);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(flywheelSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(Voltage volts) {
    closedLoop = false;
    appliedVolts = volts.in(Volts);
  }

  @Override
  public void setVelocity(LinearVelocity tangentialVelocity) {
    closedLoop = true;
    this.feedforwardVolts =
        RobotConstants.NOMINAL_VOLTAGE
            * tangentialVelocity.in(MetersPerSecond)
            / MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);
    velocityController.setSetpoint(tangentialVelocity.in(MetersPerSecond));
  }
}
