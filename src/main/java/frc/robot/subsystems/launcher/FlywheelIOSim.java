package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.launcher.LauncherConstants.FlywheelConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelIOSim implements FlywheelIO {
  private final DCMotorSim flywheelSim;

  private boolean closedLoop = false;
  private PIDController velocityController = new PIDController(kPSim, 0.0, 0.0);
  private double appliedVolts = 0.0;
  private double feedforwardVolts = 0.0;

  public FlywheelIOSim() {
    flywheelSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.004, motorReduction), gearbox);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Run closed-loop control
    if (closedLoop) {
      appliedVolts =
          velocityController.calculate(flywheelSim.getAngularPositionRad()) + feedforwardVolts;
    } else {
      velocityController.reset();
    }

    // Update simulation state
    flywheelSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    flywheelSim.update(0.02);

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
  public void setVelocity(AngularVelocity angularVelocity) {
    closedLoop = true;
    velocityController.setSetpoint(angularVelocity.in(RadiansPerSecond));
  }
}
