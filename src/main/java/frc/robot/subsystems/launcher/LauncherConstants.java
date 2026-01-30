package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.MotorConstants.KrakenX60Constants;
import frc.robot.Constants.MotorConstants.NEO550Constants;

public final class LauncherConstants {

  public static final Rotation2d impactAngle = Rotation2d.fromDegrees(50);
  public static final Distance fuelRadius = Inches.of(3);
  public static final Distance ceilingHeight = Feet.of(11).plus(Inches.of(2));
  public static final double g = 9.81;

  // For logging
  public static final double fuelSpawnPeriod = 0.1; // seconds
  public static final String nominalKey = "Nominal";
  public static final String replannedKey = "Replanned";
  public static final String actualKey = "Actual";

  public static final class TurretConstants {
    // Geometry
    public static final Transform3d chassisToTurretBase =
        new Transform3d(Inches.of(0), Inches.of(10), Inches.of(22), Rotation3d.kZero);
    public static final Rotation2d rotationOffset = new Rotation2d(0.44);

    // Absolute encoder
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // Rad/sec

    // Position controller
    public static final double turnPIDMinInput = 0.0;
    public static final double turnPIDMaxInput = 2 * Math.PI;
    public static final double turnKp = 0.35;

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

  public static final class FlywheelConstants {
    public static final Distance wheelRadius = Inches.of(1.5);

    // Velocity Controller
    public static final Slot0Configs flywheelGains =
        new Slot0Configs()
            .withKP(1)
            .withKI(0)
            .withKD(1)
            .withKS(0)
            .withKV(0)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    // Motor controller
    public static final int flywheelPort = 2;
    public static final double flywheelMotorReduction = 1.0;
    public static final AngularVelocity flywheelMaxAngularVelocity =
        KrakenX60Constants.kFreeSpeed.div(flywheelMotorReduction);

    // Simulation
    public static final double flywheelKpSim = 1;
    public static final DCMotor flywheelGearbox = DCMotor.getKrakenX60(1);
  }

  public static final class HoodConstants {
    // Absolute encoder
    public static final double hoodEncoderPositionFactor = 2 * Math.PI; // Radians
    public static final double hoodEncoderVelocityFactor = (2 * Math.PI) / 60.0; // Rad/sec

    // Position controller
    public static final double hoodPIDMinInput = 0.0;
    public static final double hoodPIDMaxInput = 2 * Math.PI;
    public static final double hoodKp = 0.35;

    // Motor controller
    public static final int hoodPort = 1;
    public static final double hoodMotorReduction = 275.0;
    public static final AngularVelocity hoodMaxAngularVelocity =
        NEO550Constants.kFreeSpeed.div(hoodMotorReduction);

    // Simulation
    public static final double hoodKpSim = 1;
    public static final double hoodKdSim = 0;
    public static final DCMotor hoodGearbox = DCMotor.getNeo550(1);
  }
}
