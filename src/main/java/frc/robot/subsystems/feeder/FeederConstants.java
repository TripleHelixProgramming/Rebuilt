package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public final class FeederConstants {
  public static final class SpindexerConstants {
    // Geometry
    public static final Distance RADIUS = Inches.of(3.0);

    // Motor
    public static final double MOTOR_REDUCTION = 1.0;
    public static final DCMotor GEARBOX = DCMotor.getNeoVortex(1);
    public static final LinearVelocity MAX_TANGENTIAL_VELOCITY =
        MetersPerSecond.of(GEARBOX.freeSpeedRadPerSec * RADIUS.in(Meters) / MOTOR_REDUCTION);

    // Encoder
    public static final double ENCODER_POSITION_FACTOR = 2.0 * Math.PI / MOTOR_REDUCTION; // Meters
    public static final double ENCODER_VELOCITY_FACTOR =
        ENCODER_POSITION_FACTOR / 60.0; // Meters/sec

    // Simulation
    public static final double kP = 0.005;
    public static final double kD = 0.005;
  }

  public static final class KickerConstants {
    // Geometry
    public static final Distance RADIUS = Inches.of(1.5);

    // Motor
    public static final double MOTOR_REDUCTION = 5.0;
    public static final DCMotor GEARBOX = DCMotor.getNeoVortex(1);
    public static final LinearVelocity MAX_TANGENTIAL_VELOCITY =
        MetersPerSecond.of(GEARBOX.freeSpeedRadPerSec * RADIUS.in(Meters) / MOTOR_REDUCTION);

    // Encoder
    public static final double ENCODER_POSITION_FACTOR = 2.0 * Math.PI / MOTOR_REDUCTION; // Meters
    public static final double ENCODER_VELOCITY_FACTOR =
        ENCODER_POSITION_FACTOR / 60.0; // Meters/sec

    // Simulation
    public static final double kP = 0.0025;
    public static final double kD = 0.0025;
  }
}
