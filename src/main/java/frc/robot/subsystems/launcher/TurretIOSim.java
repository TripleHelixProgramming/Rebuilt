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

public class TurretIOSim implements TurretIO {
  private final DCMotorSim turnSim;

  private boolean turnClosedLoop = false;
  private PIDController turnController = new PIDController(turnKpSim, 0.0, turnKdSim);
  private double turnAppliedVolts = 0.0;
  private double feedforwardVolts = 0.0;

  public TurretIOSim() {
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnGearbox, 0.004, turnMotorReduction),
            turnGearbox);

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Run closed-loop control
    if (turnClosedLoop) {
      turnAppliedVolts =
          turnController.calculate(turnSim.getAngularPositionRad()) + feedforwardVolts;
    } else {
      turnController.reset();
    }

    // Update simulation state
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    turnSim.update(0.02);

    // Update turn inputs
    inputs.connected = true;
    inputs.position = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.velocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = turnAppliedVolts;
    inputs.currentAmps = Math.abs(turnSim.getCurrentDrawAmps());
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
            / turnMaxAngularVelocity.in(RadiansPerSecond);
    turnController.setSetpoint(rotation.getRadians());
  }
}
