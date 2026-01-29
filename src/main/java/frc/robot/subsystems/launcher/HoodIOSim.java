package frc.robot.subsystems.launcher;

import static frc.robot.subsystems.launcher.LauncherConstants.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.RobotConstants;

public class HoodIOSim implements HoodIO {
  private static final AngularVelocityUnit RadiansPerSecond = null;

  private final DCMotorSim hoodSim;

  private boolean turnClosedLoop = false;
  private PIDController turnController = new PIDController(hoodKpSim, 0.0, hoodKdSim);
  private double turnAppliedVolts = 0.0;
  private double feedforwardVolts = 0.0;

  public HoodIOSim() {
    hoodSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(hoodGearbox, 0.004, hoodMotorReduction),
            hoodGearbox);

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Run closed-loop control
    if (turnClosedLoop) {
      turnAppliedVolts =
          turnController.calculate(hoodSim.getAngularPositionRad()) + feedforwardVolts;
    } else {
      turnController.reset();
    }

    // Update simulation state
    hoodSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    hoodSim.update(0.02);

    // Update turn inputs
    inputs.connected = true;
    inputs.position = new Rotation2d(hoodSim.getAngularPositionRad());
    inputs.velocityRadPerSec = hoodSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = turnAppliedVolts;
    inputs.currentAmps = Math.abs(hoodSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setPosition(Rotation2d rotation, AngularVelocity angularVelocity) {
    turnClosedLoop = true;
    this.feedforwardVolts =
        RobotConstants.kNominalVoltage
            * angularVelocity.in(RadiansPerSecond)
            / hoodMaxAngularVelocity.in(RadiansPerSecond);
    turnController.setSetpoint(rotation.getRadians());
  }
}
