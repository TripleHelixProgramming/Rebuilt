package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretConstants {
  public static final Rotation2d rotationOffset = Rotation2d.kZero;
  public static final double turnPIDMinInput = 0.0;
  public static final double turnPIDMaxInput = 2 * Math.PI;
  public static final int port = 1;
  public static final double turnEncoderPositionFactor = 2 * Math.PI;
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0;
  public static final double turnKp = 1.0;
}
