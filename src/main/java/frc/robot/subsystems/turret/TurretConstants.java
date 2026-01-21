package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class TurretConstants {
  // Geometry
  public static final Transform2d chassisToTurretBase =
      new Transform2d(Inches.of(0), Inches.of(10), Rotation2d.kZero);
  public static final Rotation2d rotationOffset = new Rotation2d(0.44);

  // Motor controller
  public static final int port = 12;

  // Absolute encoder
  public static final double turnEncoderPositionFactor = 2 * Math.PI;
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0;

  // Position controller
  public static final double turnPIDMinInput = 0.0;
  public static final double turnPIDMaxInput = 2 * Math.PI;
  public static final double turnKp = 0.35;

  // Simulation
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);
  public static final double turnMotorReduction = 5.0;
  public static final double turnKpSim = 0.35;
}
