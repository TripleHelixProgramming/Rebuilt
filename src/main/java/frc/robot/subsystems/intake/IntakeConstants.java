package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import edu.wpi.first.math.system.plant.DCMotor;
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
}
