package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
    public static final Rotation2d absEncoderOffset = new Rotation2d(0.5);
    public static final Rotation2d mechanismOffset = Rotation2d.kCCW_Pi_2;
    public static final Angle rangeOfMotion = Degrees.of(240);

    // Position controller
    public static final double kPReal = 0.5;

    // Motor controller
    public static final double motorReduction = 5.0;
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
    public static final double gearboxRatio = 1.0;

    // Velocity Controller
    public static final Slot0Configs flywheelGains =
        new Slot0Configs()
            .withKP(1.0)
            .withKI(0.0)
            .withKD(1.0)
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    // Motor controller
    public static final double motorReduction = 1.0;
    public static final AngularVelocity maxAngularVelocity =
        KrakenX60Constants.kFreeSpeed.div(motorReduction);

    // Simulation
    public static final double kPSim = 0.1;
    public static final DCMotor gearbox = DCMotor.getKrakenX60(2);
  }

  public static final class HoodConstants {
    // Absolute encoder
    public static final double encoderPositionFactor = 2 * Math.PI; // Radians
    public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // Rad/sec

    // Position controller
    public static final double kPReal = 0.35;
    public static final double minValue = Units.degreesToRadians(60);
    public static final double maxValue = Units.degreesToRadians(80);

    // Motor controller
    public static final double motorReduction = 2.75;
    public static final AngularVelocity maxAngularVelocity =
        NEO550Constants.kFreeSpeed.div(motorReduction);

    // Simulation
    public static final double kPSim = 0.2;
    public static final double kDSim = 0.05;
    public static final DCMotor gearbox = DCMotor.getNeo550(1);
  }
}
