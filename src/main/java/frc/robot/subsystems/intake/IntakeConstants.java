package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.Constants.MotorConstants.KrakenX60Constants;

public class IntakeConstants {
  /** Time (seconds) to wait after resolving an intake/hopper interlock before proceeding. */
  public static final double kInterlockSettleSeconds = 1.0;

  public static class RollerConstants {
    public static final Distance rollerRadius = Inches.of(0.85);

    // motor controller
    public static final double motorReduction = 1.0;
    public static final AngularVelocity maxAngularVelocity =
        KrakenX60Constants.kFreeSpeed.div(motorReduction);
    public static final Slot0Configs velocityVoltageGains =
        new Slot0Configs().withKP(0.11).withKI(0.0).withKD(0.0).withKS(0.1).withKV(0.12);
    public static final Slot1Configs velocityTorqueCurrentGains =
        new Slot1Configs().withKP(5).withKI(0.0).withKD(0.0).withKS(2.5);

    public static final double maxAcceleration = 4000.0;
    public static final double maxJerk = 40000.0;

    // simulation
    public static final DCMotor gearbox = DCMotor.getKrakenX60(2);

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
    public static final double encoderPositionFactor = 1.0;
    public static final double encoderVelocityFactor = 1.0;

    public static final double kPSim = 1.0;
    public static final double kDSim = 1.0;

    public static final double kPRealPos = 1.0;
    public static final double kPRealVel = 1.0;

    public static final double maxPos = 1.0;
    public static final double minPos = 1.0;

    public static final double motorReduction = 1.0;

    public static final DCMotor gearbox = DCMotor.getNEO(2);
  }
}
