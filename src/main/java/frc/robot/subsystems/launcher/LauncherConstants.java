package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants.KrakenX60Constants;
import frc.robot.Constants.MotorConstants.NEO550Constants;

public final class LauncherConstants {

  // Geometry
  public static final Rotation2d impactAngle = Rotation2d.fromDegrees(50);
  public static final Distance fuelRadius = Inches.of(3);
  public static final Distance ceilingHeight = Feet.of(11).plus(Inches.of(2));
  public static final double g = 9.81;

  // Logging / simulation periods
  public static final boolean logFuelTrajectories;
  public static final double fuelSpawnPeriod;
  public static final double ballisticSimPeriod;
  public static final double ballisticLogPeriod;

  static {
    switch (Constants.currentMode) {
      case REAL -> {
        logFuelTrajectories = true;
        fuelSpawnPeriod = 0.2;
        ballisticSimPeriod = 0.1;
        ballisticLogPeriod = 0.25;
      }

      case SIM -> {
        logFuelTrajectories = true;
        fuelSpawnPeriod = 0.1;
        ballisticSimPeriod = 0.05;
        ballisticLogPeriod = 0.1;
      }

      case REPLAY -> {
        logFuelTrajectories = false;
        fuelSpawnPeriod = 0.0;
        ballisticSimPeriod = 0.0;
        ballisticLogPeriod = 0.0;
      }

      default -> {
        logFuelTrajectories = true;
        fuelSpawnPeriod = 0.1;
        ballisticSimPeriod = 0.05;
        ballisticLogPeriod = 0.1;
      }
    }
  }

  public static final String nominalKey = "Nominal";
  public static final String replannedKey = "Replanned";
  public static final String actualKey = "Actual";

  public static final class TurretConstants {
    // Geometry
    public static final Transform3d chassisToTurretBase =
        new Transform3d(Inches.of(0), Inches.of(10), Inches.of(22), Rotation3d.kZero);
    public static final Rotation2d absEncoderOffset = new Rotation2d(5.3);
    public static final Rotation2d mechanismOffset = Rotation2d.k180deg;
    public static final Angle rangeOfMotion = Degrees.of(5);

    // Position controller
    public static final double kPReal = 0.5;

    // Motor controller
    public static final double motorReduction = 9.0 * 72.0 / 12.0;
    public static final AngularVelocity maxAngularVelocity =
        NEO550Constants.kFreeSpeed.div(motorReduction);
    public static final double encoderPositionFactor = (2 * Math.PI) / motorReduction; // Radians
    public static final double encoderVelocityFactor =
        (2 * Math.PI) / (60.0 * motorReduction); // Rad/sec

    // Simulation
    public static final DCMotor gearbox = DCMotor.getNeo550(1);
    public static final double kPSim = 0.8;
    public static final double kDSim = 0.05;
  }

  public static final class FlywheelConstants {
    public static final Distance wheelRadius = Inches.of(1.5);

    // Velocity Controller

    // Motor controller
    public static final double motorReduction = 1.0;
    public static final AngularVelocity maxAngularVelocity =
        KrakenX60Constants.kFreeSpeed.div(motorReduction);
    public static final Slot0Configs velocityVoltageGains =
        new Slot0Configs().withKP(0.11).withKI(0.0).withKD(0.0).withKS(0.1).withKV(0.12);
    public static final Slot1Configs velocityTorqueCurrentGains =
        new Slot1Configs().withKP(5).withKI(0.0).withKD(0.0).withKS(2.5);

    // Simulation
    public static final double kPSim = 0.1;
    public static final DCMotor gearbox = DCMotor.getKrakenX60(2);
  }

  public static final class HoodConstants {
    // Position controller
    public static final double kPReal = 0.35;
    public static final Angle minPosition = Degrees.of(60);
    public static final Angle maxPosition = Degrees.of(80);
    public static final double minPosRad = minPosition.in(Radians);
    public static final double maxPosRad = maxPosition.in(Radians);

    // Motor controller
    public static final double motorReduction = 5.0 * 256.0 / 16.0;
    public static final AngularVelocity maxAngularVelocity =
        NEO550Constants.kFreeSpeed.div(motorReduction);
    public static final double encoderPositionFactor = 2 * Math.PI / motorReduction; // Radians
    public static final double encoderVelocityFactor =
        (2 * Math.PI) / (60.0 * motorReduction); // Rad/sec

    // Simulation
    public static final double kPSim = 1.5;
    public static final double kDSim = 0.05;
    public static final DCMotor gearbox = DCMotor.getNeo550(1);
  }
}
