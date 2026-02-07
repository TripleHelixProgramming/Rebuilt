package frc.robot.subsystems.feeder;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.MotorConstants.NEOVortexConstants;

public final class FeederConstants {
  public static final class SpindexerConstants {
    // Motor Controller
    public static final double motorReduction = 1.0;
    public static final AngularVelocity maxAngularVelocity =
        NEOVortexConstants.kFreeSpeed.div(motorReduction);
    public static final int port = 50;

    // Encoder
    public static final double encoderPositionFactor = (2 * Math.PI); // Radians
    public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // Rad/sec
    public static final double kPSim = 1.0;
  }
}
