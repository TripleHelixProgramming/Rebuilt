package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.CANBusPorts.CAN2;

public class IntakeConstants {
  /** Time (seconds) to wait after resolving an intake/hopper interlock before proceeding. */
  public static final double kInterlockSettleSeconds = 1.0;

  public static class RollerConstants {
    public static final Distance rollerRadius = Inches.of(0.85);

    // motor controller
    public static final double MOTOR_REDUCTION = 1.0;
    public static final int numMotors = 1;
    public static final double MAX_ACCELERATION = 4000.0;
    public static final double MAX_JERK = 40000.0;

    // roller constants
    public static final double ENCODER_POSITION_FACTOR = 2.0 * Math.PI / MOTOR_REDUCTION; // Meters
    public static final double ENCODER_VELOCITY_FACTOR =
        ENCODER_POSITION_FACTOR / 60.0; // Meters/sec

    // configs
    public static final RollerConfig upperRollerConfig =
        new RollerConfig(CAN2.intakeRollerUpper, CAN2.bus, true);
    public static final RollerConfig lowerRollerConfig =
        new RollerConfig(CAN2.intakeRollerLower, CAN2.bus, true);
  }

  public static class RollerConfig {
    public final int port;
    public final CANBus bus;
    public final boolean inverted;

    public RollerConfig(int port, CANBus bus, boolean inverted) {
      this.port = port;
      this.bus = bus;
      this.inverted = inverted;
    }
  }
}
