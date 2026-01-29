package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.launcher.LauncherConstants.FlywheelConstants.flywheelGearbox;
import static frc.robot.subsystems.launcher.LauncherConstants.FlywheelConstants.flywheelKpSim;
import static frc.robot.subsystems.launcher.LauncherConstants.FlywheelConstants.flywheelMotorReduction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelIOSim implements FlywheelIO {
  private final DCMotorSim flywheelSim;

  private boolean flywheelClosedLoop = false;
  private PIDController flywheelController = new PIDController(flywheelKpSim, 0.0, 0.0);
  private double flywheelAppliedVolts = 0.0;
  private double feedforwardVolts = 0.0;

  public FlywheelIOSim() {
    flywheelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(flywheelGearbox, 0.004, flywheelMotorReduction),
            flywheelGearbox);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Run closed-loop control
    if (flywheelClosedLoop) {
      flywheelAppliedVolts =
          flywheelController.calculate(flywheelSim.getAngularPositionRad()) + feedforwardVolts;
    } else {
      flywheelController.reset();
    }

    // Update simulation state
    flywheelSim.setInputVoltage(MathUtil.clamp(flywheelAppliedVolts, -12.0, 12.0));
    flywheelSim.update(0.02);

    // Update turn inputs
    inputs.connected = true;
    inputs.velocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = flywheelAppliedVolts;
    inputs.currentAmps = Math.abs(flywheelSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double output) {
    flywheelClosedLoop = false;
    flywheelAppliedVolts = output;
  }

  @Override
  public void setVelocity(AngularVelocity angularVelocity) {
    flywheelClosedLoop = true;
    flywheelController.setSetpoint(angularVelocity.in(RadiansPerSecond));
  }
}
