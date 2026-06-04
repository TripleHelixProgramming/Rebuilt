package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public final class LauncherConstants {

  // Geometry
  public static final Distance FUEL_RADIUS = Inches.of(3);
  public static final Distance CEILING_HEIGHT = Feet.of(11).plus(Inches.of(2));
  public static final double GRAVITY = 9.81;

  // Distance-based impact angle: steeper at close range, shallower at far range
  public static final Distance IMPACT_ANGLE_CLOSE_DISTANCE = Meters.of(2.0);
  public static final Distance IMPACT_ANGLE_FAR_DISTANCE = Meters.of(6.0);
  public static final Rotation2d IMPACT_ANGLE_CLOSE = Rotation2d.fromDegrees(55);
  public static final Rotation2d IMPACT_ANGLE_FAR = Rotation2d.fromDegrees(40);

  // Logging / simulation periods
  public static final boolean LOG_FUEL_TRAJECTORIES;
  public static final double FUEL_SPAWN_PERIOD;
  public static final double BALLISTIC_SIM_PERIOD;
  public static final double BALLISTIC_LOG_PERIOD;

  static {
    switch (Constants.currentMode) {
      case REAL -> {
        LOG_FUEL_TRAJECTORIES = true;
        FUEL_SPAWN_PERIOD = 0.2;
        BALLISTIC_SIM_PERIOD = 0.1;
        BALLISTIC_LOG_PERIOD = 0.25;
      }

      case SIM -> {
        LOG_FUEL_TRAJECTORIES = true;
        FUEL_SPAWN_PERIOD = 0.1;
        BALLISTIC_SIM_PERIOD = 0.05;
        BALLISTIC_LOG_PERIOD = 0.1;
      }

      case REPLAY -> {
        LOG_FUEL_TRAJECTORIES = false;
        FUEL_SPAWN_PERIOD = 0.0;
        BALLISTIC_SIM_PERIOD = 0.0;
        BALLISTIC_LOG_PERIOD = 0.0;
      }

      default -> {
        LOG_FUEL_TRAJECTORIES = true;
        FUEL_SPAWN_PERIOD = 0.1;
        BALLISTIC_SIM_PERIOD = 0.05;
        BALLISTIC_LOG_PERIOD = 0.1;
      }
    }
  }

  public static final String NOMINAL_KEY = "Nominal";
  public static final String REPLANNED_KEY = "Replanned";
  public static final String ACTUAL_KEY = "Actual";

  // Tolerance for isOnTarget() check (independent of motor controller allowable error)
  public static final Angle IS_ON_TARGET_TOLERANCE = Degrees.of(2.0);

  public static final class TurretConstants {
    // Geometry
    public static final Transform3d CHASSIS_TO_TURRET_BASE =
        new Transform3d(Inches.of(-4.000), Inches.of(6.500), Inches.of(16.331), Rotation3d.kZero);
    public static final Rotation2d ABS_ENCODER_OFFSET = new Rotation2d(5.157);
    public static final Rotation2d MECHANISM_OFFSET = Rotation2d.kZero;
    public static final double UPPER_LIMIT_RAD = Units.degreesToRadians(270);
    public static final double LOWER_LIMIT_RAD = Units.degreesToRadians(45);
    public static final double MARGIN_RAD = Units.degreesToRadians(5);

    // Position controller
    public static final double kP = 0.5;
    public static final double kD = 0.05;
    public static final Angle kAllowableError = Degrees.of(0.25);

    // Motor controller
    public static final double MOTOR_REDUCTION = 9.0 * 72.0 / 12.0;
    public static final DCMotor GEARBOX = DCMotor.getNeo550(1);
    public static final AngularVelocity MAX_ANGULAR_VELOCITY =
        RadiansPerSecond.of(GEARBOX.freeSpeedRadPerSec / MOTOR_REDUCTION);
    public static final double ENCODER_POSITION_FACTOR = (2 * Math.PI) / MOTOR_REDUCTION; // Radians
    public static final double ENCODER_VELOCITY_FACTOR =
        (2 * Math.PI) / (60.0 * MOTOR_REDUCTION); // Rad/sec
  }

  public static final class FlywheelConstants {

    public static final class FlywheelScaling {
      public static final double EXPONENT = 1.8;
      public static final double COEFFICIENT = 0.335;
    }

    public static final Distance WHEEL_RADIUS = Inches.of(1.5);

    // Velocity Controller
    public static final double MAX_ACCELERATION = 4000.0;
    public static final double MAX_JERK = 40000.0;

    // Motor controller
    public static final double MOTOR_REDUCTION = 1.0;
    public static final DCMotor GEARBOX = DCMotor.getKrakenX60(2);
    public static final AngularVelocity MAX_ANGULAR_VELOCITY =
        RadiansPerSecond.of(GEARBOX.freeSpeedRadPerSec / MOTOR_REDUCTION);
    public static final Slot0Configs VELOCITY_VOLTAGE_GAINS =
        new Slot0Configs().withKP(0.11).withKI(0.0).withKD(0.0).withKS(0.1).withKV(0.12);
    public static final Slot1Configs VELOCITY_TORQUE_CURRENT_GAINS =
        new Slot1Configs().withKP(12).withKI(0.0).withKD(0.0).withKS(2.5);
  }

  public static final class HoodConstants {
    public static final Rotation2d BALL_TO_HOOD_OFFSET = new Rotation2d(Degrees.of(0));
    public static final Angle kAllowableError = Degrees.of(0.25);

    // Position controller
    public static final double kPRealPos = 0.35;
    public static final double kPSimPos = 1.5;
    public static final double kDSimPos = 0.05;
    public static final Angle MIN_POSITION = Degrees.of(60);
    public static final Angle MAX_POSITION = Degrees.of(80);
    public static final double MIN_POS_RAD = MIN_POSITION.in(Radians);
    public static final double MAX_POS_RAD = MAX_POSITION.in(Radians);

    // Velocity controller
    public static final double kPRealVel = 0.2;

    // Motor controller
    public static final double MOTOR_REDUCTION = 5.0 * 256.0 / 16.0;
    public static final DCMotor GEARBOX = DCMotor.getNeo550(1);
    public static final AngularVelocity MAX_ANGULAR_VELOCITY =
        RadiansPerSecond.of(GEARBOX.freeSpeedRadPerSec / MOTOR_REDUCTION);
    public static final double ENCODER_POSITION_FACTOR = 2 * Math.PI / MOTOR_REDUCTION; // Radians
    public static final double ENCODER_VELOCITY_FACTOR =
        (2 * Math.PI) / (60.0 * MOTOR_REDUCTION); // Rad/sec
  }
}
