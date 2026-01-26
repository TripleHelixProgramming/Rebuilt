package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.MotorConstants.NEO550Constants;

public final class LauncherConstants {

  public static final Rotation2d impactAngle = Rotation2d.fromDegrees(50);
  public static final Distance fuelRadius = Inches.of(3);
  public static final Distance ceilingHeight = Feet.of(11).plus(Inches.of(2));
  public static final double gravity = 9.81;
  public static final double fuelSpawnPeriod = 0.1; // seconds

  public static final class TurretConstants {
    // Geometry
    public static final Transform3d chassisToTurretBase =
        new Transform3d(Inches.of(0), Inches.of(10), Inches.of(22), Rotation3d.kZero);
    public static final Rotation2d rotationOffset = new Rotation2d(0.44);
    public static final LinearVelocity fuelVelocityRadial = MetersPerSecond.of(5);

    // Absolute encoder
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // Rad/sec

    // Position controller
    public static final double turnPIDMinInput = 0.0;
    public static final double turnPIDMaxInput = 2 * Math.PI;
    public static final double turnKp = 0.35;
    public static final Distance hubWidth = Inches.of(41.73);

    // Motor controller
    public static final int port = 12;
    public static final double turnMotorReduction = 5.0;
    public static final AngularVelocity turnMaxAngularVelocity =
        NEO550Constants.kFreeSpeed.div(turnMotorReduction);

    // Simulation
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);
    public static final double turnKpSim = 0.8;
    public static final double turnKdSim = 0.05;
  }
}
