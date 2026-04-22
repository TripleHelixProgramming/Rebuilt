package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Constants.MotorConstants.NEOConstants;

public class IntakeConstants {
  /** Time (seconds) to wait after resolving an intake/hopper interlock before proceeding. */
  public static final double kInterlockSettleSeconds = 1.0;

  public static class RollerConstants {
    public static final Distance rollerRadius = Inches.of(0.85);

    // motor controller
    public static final double motorReduction = 1.0;
    public static final int numMotors = 1;
    public static final double maxAcceleration = 4000.0;
    public static final double maxJerk = 40000.0;

    // roller constants
    public static final double encoderPositionFactor = 2.0 * Math.PI / motorReduction; // Meters
    public static final double encoderVelocityFactor = encoderPositionFactor / 60.0; // Meters/sec

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

  public static class ArmConstants {
    public static final double motorReduction = 15.0;

    public static final double encoderPositionFactor = 2 * Math.PI / motorReduction;
    public static final double encoderVelocityFactor = (2 * Math.PI) / (60.0 * motorReduction);

    public static final AngularVelocity maxAngularVelocity =
        NEOConstants.kFreeSpeed.div(motorReduction);

    public static final Angle backlash = Degrees.of(25);
    public static final Angle maxPos = Degrees.of(90.0);
    public static final Angle minPos = Degrees.of(0.0).minus(backlash);
    public static final double maxPosRad = maxPos.in(Radians);
    public static final double minPosRad = minPos.in(Radians);

    public static final DCMotor gearbox = DCMotor.getNEO(2);
  }
}
