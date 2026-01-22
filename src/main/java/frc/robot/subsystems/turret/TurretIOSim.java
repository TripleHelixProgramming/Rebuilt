package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

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
    inputs.turnConnected = true;
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setTurnPosition(Rotation2d rotation, double feedforwardVolts) {
    turnClosedLoop = true;
    this.feedforwardVolts = feedforwardVolts;
    turnController.setSetpoint(rotation.getRadians());
  }
}
