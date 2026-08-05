package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.CANBusPorts.CAN2;

public class IntakeConstants {
  /** Time (seconds) to wait after resolving an intake/hopper interlock before proceeding. */
  public static final double INTERLOCK_SETTLE_SECONDS = 1.0;

  public static class RollerConstants {
    public static final Distance RADIUS = Inches.of(0.85);

    // Motor controller
    public static final double MOTOR_REDUCTION = 1.0;
    public static final double MAX_ACCELERATION = 4000.0;
    public static final double MAX_JERK = 40000.0;

    // Encoder
    public static final double ENCODER_POSITION_FACTOR = 2.0 * Math.PI / MOTOR_REDUCTION; // Meters
    public static final double ENCODER_VELOCITY_FACTOR =
        ENCODER_POSITION_FACTOR / 60.0; // Meters/sec

    // Configs
    public record RollerConfig(int port, CANBus bus, boolean inverted) {}

    public static final RollerConfig UPPER_ROLLER_CONFIG =
        new RollerConfig(CAN2.INTAKE_ROLLER_UPPER, CAN2.BUS, true);
    public static final RollerConfig LOWER_ROLLER_CONFIG =
        new RollerConfig(CAN2.INTAKE_ROLLER_LOWER, CAN2.BUS, true);

    public static class SparkConfig {
      public static final DCMotor GEARBOX = DCMotor.getNeoVortex(1);
      public static final LinearVelocity MAX_TANGENTIAL_VELOCITY =
          MetersPerSecond.of(GEARBOX.freeSpeedRadPerSec * RADIUS.in(Meters) / MOTOR_REDUCTION);
      public static final double kP = 0.001;
      public static final double kD = 0.0;
    }

    public static class TalonConfig {
      public static final DCMotor GEARBOX = DCMotor.getKrakenX60(1);
      public static final Slot0Configs VELOCITY_VOLTAGE_GAINS =
          new Slot0Configs().withKP(0.11).withKI(0.0).withKD(0.0).withKS(0.1).withKV(0.12);
      public static final Slot1Configs VELOCITY_TORQUE_CURRENT_GAINS =
          new Slot1Configs().withKP(0.11).withKI(0.0).withKD(0.0).withKS(2.5);
    }
  }

  public static class ArmConstants {
    public static final double motorReduction = 15.0;

    public static final double encoderPositionFactor = 2 * Math.PI / motorReduction;
    public static final double encoderVelocityFactor = (2 * Math.PI) / (60.0 * motorReduction);

    public static final double absEncoderPositionFactor = 2 * Math.PI;
    public static final double absEncoderVelocityFactor = (2 * Math.PI) / 60.0;

    public static final double absEncoderOffset = 0;

    public static final DCMotor gearbox = DCMotor.getNEO(2);
    public static final AngularVelocity maxAngularVelocity =
        RadiansPerSecond.of(gearbox.freeSpeedRadPerSec / motorReduction);

    public static final Angle backlash = Degrees.of(0); // original was 25
    public static final Angle maxPos = Degrees.of(110.0);
    public static final Angle minPos = Degrees.of(0.0).minus(backlash);
    public static final double maxPosRad = maxPos.in(Radians);
    public static final double minPosRad = minPos.in(Radians);

    // Motion profile
    public static final double PROFILE_MAX_VELOCITY = maxAngularVelocity.in(RadiansPerSecond);
    public static final double PROFILE_MAX_ACCELERATION = 20.0; // rad/s^2 — assumed, tune on robot
    public static final double STOWED_TOLERANCE_RAD =
        Degrees.of(5.0).in(Radians); // assumed, tune on robot

    // Target positions. Motors and encoders are mounted inverted, so the raw range is flipped:
    // stowed reads as the top of the range and deployed reads as the bottom.
    public static final double STOWED_POS_RAD = maxPosRad;
    public static final double DEPLOYED_POS_RAD = minPosRad;

    // Configs
    public record ArmConfig(int port, CANBus bus, boolean inverted) {}

    public static final ArmConfig LEFT_ARM_CONFIG =
        new ArmConfig(CAN2.INTAKE_ARM_LEFT, CAN2.BUS, true);
    public static final ArmConfig RIGHT_ARM_CONFIG =
        new ArmConfig(CAN2.INTAKE_ARM_RIGHT, CAN2.BUS, false);
  }
}
